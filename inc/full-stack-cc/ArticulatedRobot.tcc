#include <fstream>
#include <sstream>
#include <iostream>

template <size_t N>
void readArticulatedConfigurationFromFile(const std::string& filename, std::vector<articulated_conf<N>>& confs) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        articulated_conf<N> conf;
        bool ok = true;
        for (size_t i = 0; i < N; ++i) {
            if (!(iss >> conf.joints[i])) {
                ok = false;
                break;
            }
        }
        if (ok) {
            confs.push_back(conf);
        }
    }
}

template <size_t N>
__host__ __device__ void forwardKinematics(const articulated_conf<N>& conf,
                                           const JointParams* joints,
                                           Eigen::Matrix3f* link_R,
                                           Eigen::Vector3f* link_T) {
    link_R[0] = Eigen::Matrix3f::Identity();
    link_T[0] = Eigen::Vector3f::Zero();

    for (size_t i = 0; i < N; ++i) {
        Eigen::Matrix3f Rj = axisAngleToRotation(joints[i].axis, conf[i]);
        link_R[i + 1] = link_R[i] * joints[i].origin_R * Rj;
        link_T[i + 1] = link_T[i] + link_R[i] * joints[i].origin_T;
    }
}

static __device__ __forceinline__ bool trianglesCollide(const Eigen::Matrix3f& link_R, const Eigen::Vector3f& link_T,
                                                 int link_vert_offset,
                                                 int rob_tri_idx, int obs_tri_idx,
                                                 const Eigen::Vector3f* pRob_verts, const Triangle* pRob_tris,
                                                 const Eigen::Vector3f* pObs_verts, const Triangle* pObs_tris) {
    Triangle rob_tri = pRob_tris[rob_tri_idx];
    Triangle obs_tri = pObs_tris[obs_tri_idx];

    Eigen::Vector3f rob_v0 = pRob_verts[link_vert_offset + rob_tri.v1];
    Eigen::Vector3f rob_v1 = pRob_verts[link_vert_offset + rob_tri.v2];
    Eigen::Vector3f rob_v2 = pRob_verts[link_vert_offset + rob_tri.v3];
    Eigen::Vector3f obs_v0 = pObs_verts[obs_tri.v1];
    Eigen::Vector3f obs_v1 = pObs_verts[obs_tri.v2];
    Eigen::Vector3f obs_v2 = pObs_verts[obs_tri.v3];

    rob_v0 = link_R * rob_v0 + link_T;
    rob_v1 = link_R * rob_v1 + link_T;
    rob_v2 = link_R * rob_v2 + link_T;

    return !triangles_valid(rob_v0, rob_v1, rob_v2, obs_v0, obs_v1, obs_v2);
}

static __device__ bool linkCollides(const Eigen::Matrix3f& link_R, const Eigen::Vector3f& link_T,
                             int rob_root, int link_vert_offset, int link_tri_offset,
                             const Eigen::Matrix3f* pRob_R, const Eigen::Vector3f* pRob_T,
                             const Eigen::Vector3f* pRob_dim, const int32_t* pRob_first_child,
                             const Eigen::Vector3f* pRob_verts, const Triangle* pRob_tris,
                             const Eigen::Matrix3f* pR_obs, const Eigen::Vector3f* pT_obs,
                             const Eigen::Vector3f* pObs_dim, const int32_t* pObs_first_child,
                             const Eigen::Vector3f* pObs_verts, const Triangle* pObs_tris,
                             unsigned long long* overflowCounter) {
    const int STACK_SIZE = 512;
    const int NUM_CHILDREN = 4;
    const float epsilon = 1e-6f;

    int stack_rob[STACK_SIZE];
    int stack_obs[STACK_SIZE];
    int sp = 0;

    stack_rob[sp] = rob_root;
    stack_obs[sp] = 0;
    ++sp;

    while (sp > 0) {
        --sp;
        int rob_node = stack_rob[sp];
        int obs_node = stack_obs[sp];

        int rob_fc = pRob_first_child[rob_node];
        int obs_fc = pObs_first_child[obs_node];

        Eigen::Matrix3f node_R = pRob_R[rob_node];
        Eigen::Vector3f node_T = pRob_T[rob_node];
        Eigen::Vector3f b = pRob_dim[rob_node];

        Eigen::Matrix3f world_R = link_R * node_R;
        Eigen::Vector3f world_T = link_T + link_R * node_T;

        Eigen::Matrix3f R_obs_abs = pR_obs[obs_node];
        Eigen::Vector3f T_obs_abs = pT_obs[obs_node];
        Eigen::Vector3f a = pObs_dim[obs_node];

        Eigen::Matrix3f B = R_obs_abs.transpose() * world_R;
        Eigen::Matrix3f Bf = B.cwiseAbs();
        Bf.array() += epsilon;
        Eigen::Vector3f T = (world_T - T_obs_abs).transpose() * R_obs_abs;

        if (!obbOverlap(a, b, B, Bf, T)) {
            continue;
        }

        if (rob_fc < 0 && obs_fc < 0) {
            int rob_tri = -rob_fc - 1 + link_tri_offset;
            int obs_tri = -obs_fc - 1;
            if (trianglesCollide(link_R, link_T, link_vert_offset, rob_tri, obs_tri,
                                 pRob_verts, pRob_tris, pObs_verts, pObs_tris)) {
                return true;
            }
        } else if (rob_fc < 0) {
            for (int k = 0; k < NUM_CHILDREN; ++k) {
                int ochild = obs_fc + k;
                if (pObs_first_child[ochild] == 0) continue;
                if (sp >= STACK_SIZE) {
                    atomicAdd(overflowCounter, 1ull);
                    return true;
                }
                stack_rob[sp] = rob_node;
                stack_obs[sp] = ochild;
                ++sp;
            }
        } else if (obs_fc < 0) {
            for (int k = 0; k < NUM_CHILDREN; ++k) {
                int rchild = rob_fc + k;
                if (pRob_first_child[rchild] == 0) continue;
                if (sp >= STACK_SIZE) {
                    atomicAdd(overflowCounter, 1ull);
                    return true;
                }
                stack_rob[sp] = rchild;
                stack_obs[sp] = obs_node;
                ++sp;
            }
        } else {
            for (int k = 0; k < NUM_CHILDREN; ++k) {
                int rchild = rob_fc + k;
                if (pRob_first_child[rchild] == 0) continue;
                for (int j = 0; j < NUM_CHILDREN; ++j) {
                    int ochild = obs_fc + j;
                    if (pObs_first_child[ochild] == 0) continue;
                    if (sp >= STACK_SIZE) {
                        atomicAdd(overflowCounter, 1ull);
                        return true;
                    }
                    stack_rob[sp] = rchild;
                    stack_obs[sp] = ochild;
                    ++sp;
                }
            }
        }
    }
    return false;
}

// Deferred leaf-pair triangle tests, run as a warp-uniform strided loop.
// Kept out-of-line so the register pressure of the (rare) fallback path does
// not inflate the traversal hot loop.
__device__ __noinline__ void d_articulated_tri_phase(
    const uint32_t* triRob, const uint32_t* triObs, const int numTri,
    const Eigen::Matrix3f& link_R, const Eigen::Vector3f& link_T,
    const int linkVertOff, const int linkTriOff,
    const Eigen::Matrix3f* pR_obs, const Eigen::Vector3f* pT_obs,
    const Eigen::Vector3f* pObs_dim, const int32_t* pObs_first_child,
    const Eigen::Vector3f* pObs_verts, const Triangle* pObs_tris, const float* pObs_a,
    const Eigen::Matrix3f* pRob_R, const Eigen::Vector3f* pRob_T,
    const Eigen::Vector3f* pRob_dim, const int32_t* pRob_first_child,
    const float* pRob_a,
    const Eigen::Vector3f* pRob_verts, const Triangle* pRob_tris,
    bool* s_collision) {
    const float epsilon = 1e-6f;
    const int lane = threadIdx.x & 31;
    for (int t = lane; t < numTri; t += 32) {
        if (*s_collision) {
            break;
        }
        const int rob_obb_idx = (int)triRob[t];
        const int obs_obb_idx = (int)triObs[t];

        const Eigen::Matrix3f R_obs_abs = pR_obs[obs_obb_idx];
        const Eigen::Vector3f T_obs_abs = pT_obs[obs_obb_idx];
        const Eigen::Matrix3f R_rob_abs = pRob_R[rob_obb_idx];
        const Eigen::Vector3f T_rob_abs = pRob_T[rob_obb_idx];

        Eigen::Matrix3f B, Bf;
        Eigen::Vector3f T;
        computeRelTransform(R_obs_abs, T_obs_abs, R_rob_abs, T_rob_abs,
                            link_R, link_T, epsilon, B, Bf, T);

        const int rob_tri = -pRob_first_child[rob_obb_idx] - 1 + linkTriOff;
        const int obs_tri = -pObs_first_child[obs_obb_idx] - 1;
        const Eigen::Vector3f& dimObs = pObs_dim[obs_obb_idx];
        const Eigen::Vector3f& dimRob = pRob_dim[rob_obb_idx];
        const int verdict = paperTriTri(dimObs(0), dimObs(1), pObs_a[obs_obb_idx],
                                        dimRob(0), dimRob(1), pRob_a[rob_obb_idx],
                                        B, T);
        if (verdict > 0) {
            *s_collision = true;
        } else if (verdict < 0) {
            const Triangle& rt = pRob_tris[rob_tri];
            const Triangle& ot = pObs_tris[obs_tri];
            const Eigen::Vector3f rv0 = link_R * pRob_verts[linkVertOff + rt.v1] + link_T;
            const Eigen::Vector3f rv1 = link_R * pRob_verts[linkVertOff + rt.v2] + link_T;
            const Eigen::Vector3f rv2 = link_R * pRob_verts[linkVertOff + rt.v3] + link_T;
            if (!triangles_valid_f(rv0, rv1, rv2,
                                   pObs_verts[ot.v1], pObs_verts[ot.v2], pObs_verts[ot.v3])) {
                *s_collision = true;
            }
        }
    }
}

template <size_t N>
__global__ void d_bvh_articulated(const Eigen::Matrix3f* pR_obs, const Eigen::Vector3f* pT_obs,
                                  const Eigen::Vector3f* pObs_dim, const int32_t* pObs_first_child,
                                  size_t num_obs_nodes,
                                  const Eigen::Vector3f* pObs_verts, const Triangle* pObs_tris,
                                  const float* pObs_a,
                                  const Eigen::Matrix3f* pRob_R, const Eigen::Vector3f* pRob_T,
                                  const Eigen::Vector3f* pRob_dim, const int32_t* pRob_first_child,
                                  const float* pRob_a,
                                  const Eigen::Vector3f* pRob_verts, const Triangle* pRob_tris,
                                  const int* pLinkOffset, const int* pLinkVertOffset, const int* pLinkTriOffset,
                                  int num_links,
                                  const JointParams* pJoints,
                                  const articulated_conf<N>* pConf, size_t num_confs,
                                  uint32_t* pdisjoint, uint32_t* g_next_conf,
                                  unsigned long long* d_phase, unsigned long long* overflowCounter) {
    // Config-per-warp design: 256-thread blocks, one configuration per lane
    // per batch pull. Each warp owns its 32 configurations and runs its
    // serial phases independently with __syncwarp (no block-wide barrier
    // serialization), so up to 8 configurations are in flight per block.
    //
    // Parallel outermost phase (one config per lane):
    //   - incremental FK + every link's root-OBB-vs-scene-root test
    //   - disjoint configs retire with a single warp-local OR
    //   - pending configs record (index, link mask, owner lane) in a
    //     warp-local queue
    //
    // Caching:
    //   - s_pend_mask: root-check verdicts; the serial phase never re-tests
    //     root boxes and skips disjoint links.
    //   - the FK transform matrices are computed ONCE per config by its
    //     owner lane (incremental accumulators held in registers) and
    //     broadcast to the whole warp with __shfl_sync at each link; no
    //     lane recomputes FK, and no shared memory is spent on transforms.
    //
    // Serial traversal per config: warp-cooperative box frontier (2 pairs x
    // 16 lanes, 4-ary children), leaf-leaf pairs deferred to a separate
    // warp-uniform triangle phase (paperTriTri + triangles_valid_f fallback)
    // flushed incrementally (every TRI_BATCH candidates) for early exit and
    // a tiny candidate buffer.
    //
    // overflowCounter counts conservative "buffer full -> report collision"
    // exits; the host asserts it is 0.

    if (num_obs_nodes == 0) {
        printf("Error: num_obs_nodes is zero. Exiting kernel.\n");
        return;
    }
    if (num_links > N + 1) {
        printf("Error: num_links exceeds N + 1. Exiting kernel.\n");
        return;
    }

    constexpr int BLOCK_SIZE = 256;
    constexpr int BATCH = BLOCK_SIZE; // configs per batch pull (one per lane)
    constexpr int NWARP = 8;
    constexpr int MAX_BUFFER = 256;  // per-warp box-pair frontier entries
    constexpr int MAX_TRI = 96;      // per-warp deferred candidates
    constexpr int TRI_BATCH = 64;    // flush the triangle phase at this many candidates
    const float epsilon = 1e-6f;

    __shared__ uint32_t s_batch_start;

    // Per-warp pending-config queues and counters.
    __shared__ uint32_t s_pend_idx[NWARP][32];
    __shared__ uint32_t s_pend_mask[NWARP][32];
    __shared__ uint32_t s_pend_lane[NWARP][32];
    __shared__ uint32_t s_num_pend[NWARP];
    __shared__ uint32_t s_disjoint[NWARP];

    // Per-warp box-pair frontier and deferred candidates.
    __shared__ uint32_t s_rob_pend[NWARP][MAX_BUFFER];
    __shared__ uint32_t s_obs_pend[NWARP][MAX_BUFFER];
    __shared__ int s_num_obb_pend[NWARP];
    __shared__ uint32_t s_tri_rob[NWARP][MAX_TRI];
    __shared__ uint32_t s_tri_obs[NWARP][MAX_TRI];
    __shared__ int s_num_tri[NWARP];
    __shared__ bool s_collision[NWARP];

    const int warp = threadIdx.x >> 5;
    const int lane = threadIdx.x & 31;

    const Eigen::Matrix3f R_obs_abs_root = pR_obs[0];
    const Eigen::Vector3f T_obs_abs_root = pT_obs[0];
    const Eigen::Vector3f a_root = pObs_dim[0];

    const int16_t conf_offset   = (lane >> 4) - 2;
    const int16_t rob_child_idx = (lane >> 2) & 0x3;
    const int16_t obs_child_idx = lane & 0x3;

    unsigned long long acc_init = 0, acc_trav = 0, acc_tri = 0;
    while (true) {
        __syncthreads();
        const unsigned long long t_phase0 = (lane == 0) ? globaltimer() : 0;
        if (threadIdx.x == 0) {
            s_batch_start = atomicAdd(g_next_conf, BATCH);
        }
        __syncthreads();
        const uint32_t batch_start = s_batch_start;
        if (batch_start >= num_confs) {
            if (lane == 0) {
                atomicAdd(&d_phase[0], acc_init);
                atomicAdd(&d_phase[1], acc_trav);
                atomicAdd(&d_phase[2], acc_tri);
                atomicAdd(&d_phase[3], 1);
            }
            return;
        }

        // ---- parallel outermost check: one config per lane ---------------
        {
            const uint32_t index = batch_start + threadIdx.x;
            uint32_t mask = 0;
            if (index < num_confs) {
                const articulated_conf<N> conf = pConf[index];

                Eigen::Matrix3f link_R = Eigen::Matrix3f::Identity();
                Eigen::Vector3f link_T = Eigen::Vector3f::Zero();
                for (int l = 0; l < num_links; ++l) {
                    if (pLinkOffset[l + 1] > pLinkOffset[l]) {
                        const int rob_root = pLinkOffset[l];
                        Eigen::Matrix3f B, Bf;
                        Eigen::Vector3f T;
                        computeRelTransform(R_obs_abs_root, T_obs_abs_root,
                                            pRob_R[rob_root], pRob_T[rob_root],
                                            link_R, link_T, epsilon, B, Bf, T);
                        if (obbOverlap(a_root, pRob_dim[rob_root], B, Bf, T)) {
                            mask |= (1u << l);
                        }
                    }
                    if (l + 1 < num_links) {
                        const JointParams& jp = pJoints[l];
                        const Eigen::Matrix3f Rj = axisAngleToRotation(jp.axis, conf[l]);
                        const Eigen::Matrix3f nextR = link_R * jp.origin_R * Rj;
                        const Eigen::Vector3f nextT = link_T + link_R * jp.origin_T;
                        link_R = nextR;
                        link_T = nextT;
                    }
                }

                if (mask == 0) {
                    atomicOr(&s_disjoint[warp], 1u << lane);
                } else {
                    const uint32_t pos = atomicAdd(&s_num_pend[warp], 1);
                    s_pend_idx[warp][pos] = index;
                    s_pend_mask[warp][pos] = mask;
                    s_pend_lane[warp][pos] = (uint32_t)lane;
                }
            }
            __syncwarp();
            if (lane == 0) {
                pdisjoint[(batch_start >> 5) + warp] |= s_disjoint[warp];
                s_disjoint[warp] = 0;
                acc_init += globaltimer() - t_phase0;
            }
        }

        // ---- per-warp serial phase: each warp traverses its pending configs
        for (uint32_t k = 0; k < s_num_pend[warp]; ++k) {
            const unsigned long long t_cfg = (lane == 0) ? globaltimer() : 0;
            const uint32_t index = s_pend_idx[warp][k];
            const uint32_t mask = s_pend_mask[warp][k];
            const uint32_t owner = s_pend_lane[warp][k];
            const articulated_conf<N> conf = pConf[index];
            unsigned long long tri_ns = 0;

            s_collision[warp] = false;
            __syncwarp();

            // FK accumulators: meaningful only on the owner lane; broadcast
            // to the whole warp at each link (transform matrices are computed
            // once per config, by its owner lane).
            Eigen::Matrix3f ownR = Eigen::Matrix3f::Identity();
            Eigen::Vector3f ownT = Eigen::Vector3f::Zero();

            for (int l = 0; l < num_links; ++l) {
                Eigen::Matrix3f link_R;
                Eigen::Vector3f link_T;
#pragma unroll
                for (int i = 0; i < 9; ++i) {
                    link_R.data()[i] = __shfl_sync(0xffffffffu, ownR.data()[i], owner, 32);
                }
#pragma unroll
                for (int i = 0; i < 3; ++i) {
                    link_T[i] = __shfl_sync(0xffffffffu, ownT[i], owner, 32);
                }

                const bool hasMesh = (pLinkOffset[l + 1] > pLinkOffset[l]);
                const bool rootHit = ((mask >> l) & 1u) != 0;

                // Seed the frontier with the (link root, scene root) pair;
                // root overlap was proven in the parallel phase (mask).
                if (lane == 0) {
                    s_num_obb_pend[warp] = 0;
                    s_num_tri[warp] = 0;
                    if (hasMesh && rootHit) {
                        s_rob_pend[warp][0] = (uint32_t)pLinkOffset[l];
                        s_obs_pend[warp][0] = 0;
                        s_num_obb_pend[warp] = 1;
                    }
                }
                __syncwarp();

                if (s_num_obb_pend[warp] > 0) {
                    while (true) {
                        __syncwarp();
                        if (s_collision[warp]) {
                            break;
                        }
                        // Incremental triangle phase: flush candidates once
                        // TRI_BATCH accumulate (bounds the buffer, gives
                        // early exit at this granularity).
                        if (s_num_tri[warp] >= TRI_BATCH) {
                            const unsigned long long t_tri0 = (lane == 0) ? globaltimer() : 0;
                            d_articulated_tri_phase(
                                s_tri_rob[warp], s_tri_obs[warp], s_num_tri[warp],
                                link_R, link_T,
                                pLinkVertOffset[l], pLinkTriOffset[l],
                                pR_obs, pT_obs, pObs_dim, pObs_first_child,
                                pObs_verts, pObs_tris, pObs_a,
                                pRob_R, pRob_T, pRob_dim, pRob_first_child, pRob_a,
                                pRob_verts, pRob_tris,
                                &s_collision[warp]);
                            if (lane == 0) {
                                tri_ns += globaltimer() - t_tri0;
                                s_num_tri[warp] = 0;
                            }
                            continue;
                        }
                        if (s_num_obb_pend[warp] == 0) {
                            break;
                        }
                        if (s_num_obb_pend[warp] >= MAX_BUFFER - 32) {
                            if (lane == 0) {
                                atomicAdd(overflowCounter, 1ull);
                                s_collision[warp] = true;
                            }
                            break;
                        }

                        // pend_idx from the pre-pop count: the two newest
                        // pairs are expanded by 16 lanes each. The count is
                        // already visible from the loop-top syncwarp; one
                        // syncwarp after lane 0's pop makes it visible to
                        // the expanders.
                        const int pend_idx = s_num_obb_pend[warp] + conf_offset;

                        if (lane == 0) {
                            if (s_num_obb_pend[warp] > 1) {
                                s_num_obb_pend[warp] -= 2;
                            } else {
                                s_num_obb_pend[warp] = 0;
                            }
                        }
                        __syncwarp();

                        if (pend_idx < 0) {
                            continue;
                        }

                        const int rob_obb_par_idx = (int)s_rob_pend[warp][pend_idx];
                        const int obs_obb_par_idx = (int)s_obs_pend[warp][pend_idx];
                        const int rob_fc_par = pRob_first_child[rob_obb_par_idx];
                        const int obs_fc_par = pObs_first_child[obs_obb_par_idx];

                        // Leaf-aware child selection: a leaf stays paired
                        // with each child of the other node (also handles
                        // leaf roots).
                        int rob_obb_idx = rob_obb_par_idx;
                        int obs_obb_idx = obs_obb_par_idx;
                        bool active = true;
                        if (rob_fc_par < 0 && obs_fc_par < 0) {
                            if (rob_child_idx != 0 || obs_child_idx != 0) {
                                active = false;
                            }
                        } else if (rob_fc_par < 0) {
                            if (rob_child_idx != 0) {
                                active = false;
                            } else {
                                obs_obb_idx = obs_fc_par + obs_child_idx;
                            }
                        } else if (obs_fc_par < 0) {
                            if (obs_child_idx != 0) {
                                active = false;
                            } else {
                                rob_obb_idx = rob_fc_par + rob_child_idx;
                            }
                        } else {
                            rob_obb_idx = rob_fc_par + rob_child_idx;
                            obs_obb_idx = obs_fc_par + obs_child_idx;
                        }
                        if (!active) {
                            continue;
                        }

                        const int rob_first_child_idx = pRob_first_child[rob_obb_idx];
                        const int obs_first_child_idx = pObs_first_child[obs_obb_idx];
                        if (rob_first_child_idx == 0 || obs_first_child_idx == 0) {
                            continue; // dummy (padding) node
                        }

                        const Eigen::Matrix3f R_obs_abs = pR_obs[obs_obb_idx];
                        const Eigen::Vector3f T_obs_abs = pT_obs[obs_obb_idx];
                        const Eigen::Matrix3f R_rob_abs = pRob_R[rob_obb_idx];
                        const Eigen::Vector3f T_rob_abs = pRob_T[rob_obb_idx];
                        const Eigen::Vector3f b = pRob_dim[rob_obb_idx];
                        const Eigen::Vector3f a = pObs_dim[obs_obb_idx];

                        Eigen::Matrix3f B, Bf;
                        Eigen::Vector3f T;
                        computeRelTransform(R_obs_abs, T_obs_abs, R_rob_abs, T_rob_abs,
                                            link_R, link_T, epsilon, B, Bf, T);
                        if (!obbOverlap(a, b, B, Bf, T)) {
                            continue;
                        }

                        if (rob_first_child_idx < 0 && obs_first_child_idx < 0) {
                            // Both leaves: defer to the triangle phase.
                            const int pos = atomicAdd(&s_num_tri[warp], 1);
                            if (pos < MAX_TRI) {
                                s_tri_rob[warp][pos] = (uint32_t)rob_obb_idx;
                                s_tri_obs[warp][pos] = (uint32_t)obs_obb_idx;
                            } else if (lane == 0) {
                                atomicAdd(overflowCounter, 1ull);
                                s_collision[warp] = true; // conservative
                            }
                        } else {
                            // One or both internal: expand (leaf-aware).
                            const int pos = atomicAdd(&s_num_obb_pend[warp], 1);
                            if (pos < MAX_BUFFER) {
                                s_obs_pend[warp][pos] = (uint32_t)obs_obb_idx;
                                s_rob_pend[warp][pos] = (uint32_t)rob_obb_idx;
                            } else if (lane == 0) {
                                atomicAdd(overflowCounter, 1ull);
                                s_collision[warp] = true; // conservative
                            }
                        }
                    } // OBB traversal of this link

                    __syncwarp();

                    // Final triangle phase for the leftover candidates.
                    if (!s_collision[warp] && s_num_tri[warp] > 0) {
                        const unsigned long long t_tri0 = (lane == 0) ? globaltimer() : 0;
                        d_articulated_tri_phase(
                            s_tri_rob[warp], s_tri_obs[warp], s_num_tri[warp],
                            link_R, link_T,
                            pLinkVertOffset[l], pLinkTriOffset[l],
                            pR_obs, pT_obs, pObs_dim, pObs_first_child,
                            pObs_verts, pObs_tris, pObs_a,
                            pRob_R, pRob_T, pRob_dim, pRob_first_child, pRob_a,
                            pRob_verts, pRob_tris,
                            &s_collision[warp]);
                        if (lane == 0) {
                            tri_ns += globaltimer() - t_tri0;
                        }
                    }
                    __syncwarp();
                    if (lane == 0) {
                        s_num_tri[warp] = 0;
                    }
                }
                __syncwarp();
                if (s_collision[warp]) {
                    break; // this link collides: the configuration is in collision
                }

                // Advance the owner lane's FK to the next link.
                if (l + 1 < num_links) {
                    const JointParams& jp = pJoints[l];
                    const Eigen::Matrix3f Rj = axisAngleToRotation(jp.axis, conf[l]);
                    const Eigen::Matrix3f nextR = ownR * jp.origin_R * Rj;
                    const Eigen::Vector3f nextT = ownT + ownR * jp.origin_T;
                    ownR = nextR;
                    ownT = nextT;
                }
            } // links

            __syncwarp();
            const unsigned long long t_end = (lane == 0) ? globaltimer() : 0;
            if (lane == 0) {
                acc_trav += (t_end - t_cfg) - tri_ns;
                acc_tri += tri_ns;
            }
            if (!s_collision[warp]) {
                if (lane == 0) {
                    s_disjoint[warp] |= 1u << (index & 31);
                }
            }
            __syncwarp();
        } // per-warp pending configurations

        __syncwarp();
        if (lane == 0) {
            pdisjoint[(batch_start >> 5) + warp] |= s_disjoint[warp];
            s_disjoint[warp] = 0;
            s_num_pend[warp] = 0;
        }
    }
    return;
}

template <size_t N>
double bvh_articulated(const std::string& robot_urdf_path,
                       const BVNode_soa<int32_t>& obs_BVH, const MeshData& obs_mesh,
                       const std::vector<articulated_conf<N>>& confs,
                       std::vector<bool>& valid, bool dry_run) {
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    const size_t num_confs = confs.size();
    valid.assign(num_confs, false);
    if (num_confs == 0) {
        return 0.0;
    }

    ArticulatedRobot robot;
    if (!parseSerialChainURDF(robot_urdf_path, robot)) {
        std::cerr << "Failed to parse URDF: " << robot_urdf_path << std::endl;
        return 0.0;
    }
    if (robot.num_joints != N) {
        std::cerr << "URDF has " << robot.num_joints
                  << " joints, but bvh_articulated was instantiated with N=" << N << std::endl;
        return 0.0;
    }
    const int num_links = (int)(robot.num_joints + 1);

    std::vector<Eigen::Matrix3f> rob_R;
    std::vector<Eigen::Vector3f> rob_T;
    std::vector<Eigen::Vector3f> rob_dim;
    std::vector<int32_t> rob_first_child;
    std::vector<float> rob_a;
    std::vector<Eigen::Vector3f> rob_verts;
    std::vector<Triangle> rob_tris;

    std::vector<int> link_offset(num_links + 1, 0);
    std::vector<int> link_vert_offset(num_links + 1, 0);
    std::vector<int> link_tri_offset(num_links + 1, 0);

    for (int l = 0; l < num_links; ++l) {
        link_offset[l] = static_cast<int>(rob_first_child.size());
        link_vert_offset[l] = static_cast<int>(rob_verts.size());
        link_tri_offset[l] = static_cast<int>(rob_tris.size());

        if (!robot.link_meshes[l].empty()) {
            BVNode_soa<int32_t> bvh = BVH_n_ary_hierarchy_from_mesh<int32_t>(robot.link_meshes[l].c_str(), 2);
            const int base = static_cast<int>(rob_first_child.size());
            for (size_t i = 0; i < bvh.size; ++i) {
                rob_R.push_back(bvh.pR[i]);
                rob_T.push_back(bvh.pT[i]);
                rob_dim.push_back(bvh.pDim[i]);
                // first_child pointers are indices relative to this link's own
                // BVH; rebase internal pointers onto the concatenated array.
                int32_t fc = (int32_t)bvh.first_child[i];
                if (fc > 0) {
                    fc += base;
                }
                rob_first_child.push_back(fc);
                rob_a.push_back(bvh.pA[i]);
            }

            MeshData md;
            loadOBJFile(robot.link_meshes[l], md.vertices, md.triangles);
            rob_verts.insert(rob_verts.end(), md.vertices.begin(), md.vertices.end());
            rob_tris.insert(rob_tris.end(), md.triangles.begin(), md.triangles.end());
        }
    }
    link_offset[num_links] = static_cast<int>(rob_first_child.size());
    link_vert_offset[num_links] = static_cast<int>(rob_verts.size());
    link_tri_offset[num_links] = static_cast<int>(rob_tris.size());

    std::vector<JointParams> joints = robot.joints;

    const int blockSize = 256;
    const size_t num_words = (num_confs + 31) / 32;

    // Persistent blocks pulling configs from a global work queue (same sizing
    // scheme as bvh_naive). No dynamic shared memory: first_child arrays are
    // read from global memory, so the static pend buffers bound occupancy.
    int device;
    checkCudaMem(cudaGetDevice(&device));
    int num_sms = 0;
    checkCudaMem(cudaDeviceGetAttribute(&num_sms, cudaDevAttrMultiProcessorCount, device));
    int blocks_per_sm = 0;
    checkCudaMem(cudaOccupancyMaxActiveBlocksPerMultiprocessor(&blocks_per_sm, d_bvh_articulated<N>, blockSize, 0));
    if (blocks_per_sm < 1) blocks_per_sm = 1;
    const int max_blocks = (int)((num_confs + blockSize - 1) / blockSize);
    const int persistent_blocks = num_sms * blocks_per_sm;
    const int gridSize = (max_blocks < persistent_blocks) ? max_blocks : persistent_blocks;

    Eigen::Matrix3f* d_R_obs;
    Eigen::Vector3f* d_T_obs;
    Eigen::Vector3f* d_Obs_dim;
    int32_t* d_Obs_first_child;
    Eigen::Vector3f* d_Obs_verts;
    Triangle* d_Obs_tris;
    float* d_Obs_a;

    Eigen::Matrix3f* d_Rob_R;
    Eigen::Vector3f* d_Rob_T;
    Eigen::Vector3f* d_Rob_dim;
    int32_t* d_Rob_first_child;
    float* d_Rob_a;
    Eigen::Vector3f* d_Rob_verts;
    Triangle* d_Rob_tris;

    int* d_LinkOffset;
    int* d_LinkVertOffset;
    int* d_LinkTriOffset;
    JointParams* d_Joints;
    articulated_conf<N>* d_Conf;
    uint32_t* d_disjoint;
    uint32_t* d_next_conf;
    unsigned long long* d_phase;
    unsigned long long* d_overflow;

    cudaEventRecord(start, 0);

    cudaMalloc((void**)&d_R_obs, obs_BVH.size * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_T_obs, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_dim, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_first_child, obs_BVH.size * sizeof(int32_t));
    cudaMalloc((void**)&d_Obs_verts, obs_mesh.vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_tris, obs_mesh.triangles.size() * sizeof(Triangle));
    cudaMalloc((void**)&d_Obs_a, obs_BVH.size * sizeof(float));

    cudaMalloc((void**)&d_Rob_R, rob_R.size() * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_Rob_T, rob_T.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_dim, rob_dim.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_first_child, rob_first_child.size() * sizeof(int32_t));
    cudaMalloc((void**)&d_Rob_a, rob_a.size() * sizeof(float));
    cudaMalloc((void**)&d_Rob_verts, rob_verts.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_tris, rob_tris.size() * sizeof(Triangle));

    cudaMalloc((void**)&d_LinkOffset, (num_links + 1) * sizeof(int));
    cudaMalloc((void**)&d_LinkVertOffset, (num_links + 1) * sizeof(int));
    cudaMalloc((void**)&d_LinkTriOffset, (num_links + 1) * sizeof(int));
    cudaMalloc((void**)&d_Joints, N * sizeof(JointParams));
    cudaMalloc((void**)&d_Conf, num_confs * sizeof(articulated_conf<N>));
    cudaMalloc((void**)&d_disjoint, num_words * sizeof(uint32_t));
    cudaMalloc((void**)&d_next_conf, sizeof(uint32_t));
    cudaMalloc((void**)&d_phase, 4 * sizeof(unsigned long long));
    cudaMalloc((void**)&d_overflow, sizeof(unsigned long long));

    checkCudaMem(cudaMemcpy(d_R_obs, obs_BVH.pR, obs_BVH.size * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_T_obs, obs_BVH.pT, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_dim, obs_BVH.pDim, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_first_child, obs_BVH.first_child, obs_BVH.size * sizeof(int32_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_verts, obs_mesh.vertices.data(), obs_mesh.vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_tris, obs_mesh.triangles.data(), obs_mesh.triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_a, obs_BVH.pA, obs_BVH.size * sizeof(float), cudaMemcpyHostToDevice));

    checkCudaMem(cudaMemcpy(d_Rob_R, rob_R.data(), rob_R.size() * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_T, rob_T.data(), rob_T.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_dim, rob_dim.data(), rob_dim.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_first_child, rob_first_child.data(), rob_first_child.size() * sizeof(int32_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_a, rob_a.data(), rob_a.size() * sizeof(float), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_verts, rob_verts.data(), rob_verts.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_tris, rob_tris.data(), rob_tris.size() * sizeof(Triangle), cudaMemcpyHostToDevice));

    checkCudaMem(cudaMemcpy(d_LinkOffset, link_offset.data(), (num_links + 1) * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_LinkVertOffset, link_vert_offset.data(), (num_links + 1) * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_LinkTriOffset, link_tri_offset.data(), (num_links + 1) * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Joints, joints.data(), N * sizeof(JointParams), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Conf, confs.data(), num_confs * sizeof(articulated_conf<N>), cudaMemcpyHostToDevice));
    cudaDeviceSynchronize();

    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float duration = 0;
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Articulated BVH allocation and transfer to GPU took " << duration << " ms." << std::endl;

    auto launch = [&](int blocks, size_t launchConfs) {
        d_bvh_articulated<N><<<blocks, blockSize>>>(
            d_R_obs, d_T_obs, d_Obs_dim, d_Obs_first_child, obs_BVH.size,
            d_Obs_verts, d_Obs_tris, d_Obs_a,
            d_Rob_R, d_Rob_T, d_Rob_dim, d_Rob_first_child, d_Rob_a,
            d_Rob_verts, d_Rob_tris,
            d_LinkOffset, d_LinkVertOffset, d_LinkTriOffset,
            num_links,
            d_Joints, d_Conf, launchConfs,
            d_disjoint, d_next_conf, d_phase, d_overflow);
    };

    if (dry_run) {
        checkCudaMem(cudaMemset(d_next_conf, 0, sizeof(uint32_t)));
        checkCudaMem(cudaMemset(d_phase, 0, 4 * sizeof(unsigned long long)));
        checkCudaMem(cudaMemset(d_disjoint, 0, num_words * sizeof(uint32_t)));
        checkCudaMem(cudaMemset(d_overflow, 0, sizeof(unsigned long long)));
        // Dry run: warm the kernel with a single batch (256 configs) like
        // bvh_naive, instead of letting one block drain the whole queue.
        const size_t dryConfs = (num_confs < 256) ? num_confs : 256;
        launch(1, dryConfs);
        checkCudaMem(cudaGetLastError());
        checkCudaMem(cudaDeviceSynchronize());
        std::cout << "Articulated BVH dry run completed successfully." << std::endl;
    }

    checkCudaMem(cudaMemset(d_next_conf, 0, sizeof(uint32_t)));
    checkCudaMem(cudaMemset(d_phase, 0, 4 * sizeof(unsigned long long)));
    checkCudaMem(cudaMemset(d_disjoint, 0, num_words * sizeof(uint32_t)));
    checkCudaMem(cudaMemset(d_overflow, 0, sizeof(unsigned long long)));
    cudaEventRecord(start, 0);
    launch(gridSize, num_confs);
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Articulated BVH GPU kernel took " << duration << " ms for "
              << num_confs << " configurations." << std::endl;

    uint64_t h_phase[4];
    checkCudaMem(cudaMemcpy(h_phase, d_phase, 4 * sizeof(uint64_t), cudaMemcpyDeviceToHost));
    const double ns_per_ms = 1e6;
    const double nblocks = (h_phase[3] > 0) ? (double)h_phase[3] : 1.0;
    std::cout << "PHASES ms (avg per block, " << h_phase[3] << " blocks, block-serial, not wall-clock): init="
              << (double)h_phase[0] / ns_per_ms / nblocks
              << " traversal=" << (double)h_phase[1] / ns_per_ms / nblocks
              << " triangles=" << (double)h_phase[2] / ns_per_ms / nblocks << std::endl;

    unsigned long long h_overflow = 0;
    checkCudaMem(cudaMemcpy(&h_overflow, d_overflow, sizeof(unsigned long long), cudaMemcpyDeviceToHost));
    if (h_overflow != 0) {
        std::cerr << "WARNING: " << h_overflow
                  << " conservative pending-list overflows (potential false positives)"
                  << std::endl;
    }

    std::unique_ptr<uint32_t[]> disjoint(new uint32_t[num_words]);
    checkCudaMem(cudaMemcpy(disjoint.get(), d_disjoint, num_words * sizeof(uint32_t), cudaMemcpyDeviceToHost));
    cudaDeviceSynchronize();

    // Bit-packed results, same semantics as d_bvh_naive: 1 = disjoint (free).
    for (size_t i = 0; i < num_confs; ++i) {
        valid[i] = ((disjoint[i >> 5] >> (i & 31)) & 1u) != 0;
    }

    cudaFree(d_R_obs);
    cudaFree(d_T_obs);
    cudaFree(d_Obs_dim);
    cudaFree(d_Obs_first_child);
    cudaFree(d_Obs_verts);
    cudaFree(d_Obs_tris);
    cudaFree(d_Obs_a);
    cudaFree(d_Rob_R);
    cudaFree(d_Rob_T);
    cudaFree(d_Rob_dim);
    cudaFree(d_Rob_first_child);
    cudaFree(d_Rob_a);
    cudaFree(d_Rob_verts);
    cudaFree(d_Rob_tris);
    cudaFree(d_LinkOffset);
    cudaFree(d_LinkVertOffset);
    cudaFree(d_LinkTriOffset);
    cudaFree(d_Joints);
    cudaFree(d_Conf);
    cudaFree(d_disjoint);
    cudaFree(d_next_conf);
    cudaFree(d_phase);
    cudaFree(d_overflow);

    return duration;
}
