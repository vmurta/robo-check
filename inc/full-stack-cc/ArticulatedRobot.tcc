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
//
// Narrow phase = Chang & Kim 2009 paperTriTri in rectangle-local coordinates
// (verdict > 0 hit / 0 free / < 0 unreliable); unreliable pairs fall back to
// the full world-frame test triangles_valid_f. Templated on the child/queue
// index types. The QuatSAT layout converts the link quaternion to a matrix
// once per flush (not per node pair); other layouts receive the FK matrix
// directly. All layouts use the matrix narrow phase.
template <typename RobChildT, typename ObsChildT, typename QRob, typename QObs, NodeLayout LAYOUT>
static __device__ __noinline__ void d_articulated_tri_phase(
    const QRob* triRob, const QObs* triObs, const int numTri,
    const Eigen::Matrix3f& link_R, const Quat& q_link, const Eigen::Vector3f& link_T,
    const int linkVertOff, const int linkTriOff,
    const ObstacleSoA<ObsChildT, LAYOUT> obs, const RobotSoA<RobChildT, LAYOUT> rob,
    bool* s_collision) {
    const int lane = threadIdx.x & 31;
    for (int t = lane; t < numTri; t += 32) {
        if (*s_collision) {
            break;
        }
        const int rob_obb_idx = (int)triRob[t];
        const int obs_obb_idx = (int)triObs[t];

        if constexpr (LAYOUT == NodeLayout::QuatSAT) {
            // Narrow phase from the link matrix: the broad phase keeps the
            // quaternion, but the narrow phase converts the link quat once
            // per flush (not per node pair) and reuses the matrix form.
            const Eigen::Matrix3f link_Rm = quatToMatrix(q_link);
            Eigen::Matrix3f R_obs_abs, R_rob_abs;
            Eigen::Vector3f T_obs_abs, T_rob_abs, dimObs, dimRob;
            float aObs, aRob;
            loadNode(obs.nodes, obs_obb_idx, R_obs_abs, T_obs_abs, dimObs, aObs);
            loadNode(rob.nodes, rob_obb_idx, R_rob_abs, T_rob_abs, dimRob, aRob);

            Eigen::Matrix3f B;
            Eigen::Vector3f T;
            computeRelTransformNoBf(R_obs_abs, T_obs_abs, R_rob_abs, T_rob_abs,
                                    link_Rm, link_T, B, T);

            const int rob_tri = -rob.first_child[rob_obb_idx] - 1 + linkTriOff;
            const int obs_tri = -obs.first_child[obs_obb_idx] - 1;
            const int verdict = paperTriTri(dimObs(0), dimObs(1), aObs,
                                            dimRob(0), dimRob(1), aRob,
                                            B, T);
            if (verdict > 0) {
                *s_collision = true;
            } else if (verdict < 0) {
                const Triangle& rt = rob.tris[rob_tri];
                const Triangle& ot = obs.tris[obs_tri];
                const Eigen::Vector3f rv0 = link_Rm * rob.verts[linkVertOff + rt.v1] + link_T;
                const Eigen::Vector3f rv1 = link_Rm * rob.verts[linkVertOff + rt.v2] + link_T;
                const Eigen::Vector3f rv2 = link_Rm * rob.verts[linkVertOff + rt.v3] + link_T;
                if (!triangles_valid_f(rv0, rv1, rv2,
                                       obs.verts[ot.v1], obs.verts[ot.v2], obs.verts[ot.v3])) {
                    *s_collision = true;
                }
            }
        } else {
            Eigen::Matrix3f R_obs_abs, R_rob_abs;
            Eigen::Vector3f T_obs_abs, T_rob_abs, dimObs, dimRob;
            float aObs, aRob;
            loadNode(obs.nodes, obs_obb_idx, R_obs_abs, T_obs_abs, dimObs, aObs);
            loadNode(rob.nodes, rob_obb_idx, R_rob_abs, T_rob_abs, dimRob, aRob);

            Eigen::Matrix3f B;
            Eigen::Vector3f T;
            computeRelTransformNoBf(R_obs_abs, T_obs_abs, R_rob_abs, T_rob_abs,
                                    link_R, link_T, B, T);

            const int rob_tri = -rob.first_child[rob_obb_idx] - 1 + linkTriOff;
            const int obs_tri = -obs.first_child[obs_obb_idx] - 1;
            const int verdict = paperTriTri(dimObs(0), dimObs(1), aObs,
                                            dimRob(0), dimRob(1), aRob,
                                            B, T);
            if (verdict > 0) {
                *s_collision = true;
            } else if (verdict < 0) {
                const Triangle& rt = rob.tris[rob_tri];
                const Triangle& ot = obs.tris[obs_tri];
                const Eigen::Vector3f rv0 = link_R * rob.verts[linkVertOff + rt.v1] + link_T;
                const Eigen::Vector3f rv1 = link_R * rob.verts[linkVertOff + rt.v2] + link_T;
                const Eigen::Vector3f rv2 = link_R * rob.verts[linkVertOff + rt.v3] + link_T;
                if (!triangles_valid_f(rv0, rv1, rv2,
                                       obs.verts[ot.v1], obs.verts[ot.v2], obs.verts[ot.v3])) {
                    *s_collision = true;
                }
            }
        }
    }
}

template <size_t N, typename RobChildT, typename ObsChildT, bool RESTRICT_PTRS, NodeLayout LAYOUT>
__device__ __forceinline__ void d_bvh_articulated_body(const ObstacleSoA<ObsChildT, LAYOUT> obs,
                                  const RobotSoA<RobChildT, LAYOUT> rob,
                                  const articulated_conf<N>* pConf, size_t num_confs,
                                  const KernelOut out) {
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
    //   - s_pend_root_mask: per-link root-overlap verdicts from the parallel
    //     phase (bit l = link l's root OBB overlaps the scene root OBB -- a
    //     prefilter, NOT a collision verdict). The serial phase never
    //     re-tests root boxes: links with a clear bit are provably
    //     collision-free (the root OBBs contain all their geometry) and are
    //     skipped; links with a set bit get the full traversal.
    //   - the FK transform is computed ONCE per config by its owner lane
    //     (incremental accumulators held in registers) and broadcast to the
    //     whole warp with __shfl_sync at each link; no lane recomputes FK,
    //     and no shared memory is spent on transforms. The QuatSAT layout
    //     keeps a matrix accumulator for the FK advance (wide ILP, cheap
    //     translation update) plus an incremental link quaternion for the
    //     pure-quat broad phase (4-float broadcast, no matrixToQuat); other
    //     layouts use the matrix form alone.
    //
    // Serial traversal per config: warp-cooperative box frontier (2 pairs x
    // 16 lanes, 4-ary children), leaf-leaf pairs deferred to a separate
    // warp-uniform triangle phase (paperTriTri + triangles_valid_f fallback)
    // flushed incrementally (every TRI_BATCH candidates) for early exit and
    // a tiny candidate buffer.
    //
    // out.overflowCounter counts conservative "buffer full -> report collision"
    // exits; the host asserts it is 0.

    if (obs.num_nodes == 0) {
        printf("Error: obs.num_nodes is zero. Exiting kernel.\n");
        return;
    }
    // The link count is a compile-time constant (serial chain: N joints ->
    // N+1 links), so the per-link loops can be flattened/unrolled.
    constexpr int num_links = (int)N + 1;

    constexpr int BLOCK_SIZE = 256;
    constexpr int WARP_BATCH = 32;   // configs per warp pull (one per lane)
    constexpr int NWARP = 8;
    constexpr int MAX_BUFFER = 256;  // per-warp box-pair frontier entries
    constexpr int MAX_TRI = 96;      // per-warp deferred candidates
    constexpr int TRI_BATCH = 64;    // flush the triangle phase at this many candidates
    const float epsilon = 1e-6f;

    typedef typename ArticQueueIdx<RobChildT>::Q QRob;
    typedef typename ArticQueueIdx<ObsChildT>::Q QObs;

    // Per-warp pending-config queues and counters. s_pend_root_mask holds one
    // bit per link (at most N+1 <= 8 links): bit l = link l's root OBB
    // overlaps the scene root OBB (parallel-phase prefilter, NOT a collision
    // verdict). s_pend_lane holds the owner lane (0-31); both are uint8_t to
    // shrink the static shared footprint.
    static_assert(N + 1 <= 8, "s_pend_root_mask is uint8_t: at most 8 links supported");
    __shared__ uint32_t s_pend_idx[NWARP][32];
    __shared__ uint8_t s_pend_root_mask[NWARP][32];
    __shared__ uint8_t s_pend_lane[NWARP][32];
    __shared__ uint32_t s_num_pend[NWARP];
    __shared__ uint32_t s_disjoint[NWARP];

    // Per-warp box-pair frontier and deferred candidates.
    __shared__ QRob s_rob_pend[NWARP][MAX_BUFFER];
    __shared__ QObs s_obs_pend[NWARP][MAX_BUFFER];
    __shared__ int s_num_obb_pend[NWARP];
    __shared__ QRob s_tri_rob[NWARP][MAX_TRI];
    __shared__ QObs s_tri_obs[NWARP][MAX_TRI];
    __shared__ int s_num_tri[NWARP];
    __shared__ bool s_collision[NWARP];

    const int warp = threadIdx.x >> 5;
    const int lane = threadIdx.x & 31;

    Eigen::Matrix3f R_obs_abs_root;
    Eigen::Vector3f T_obs_abs_root, a_root;
    float a_root_unused;
    Quat q_obs_root;
    if (LAYOUT == NodeLayout::QuatSAT) {
        loadNodeQuat(obs.nodes, 0, q_obs_root, T_obs_abs_root, a_root, a_root_unused);
    } else {
        loadNode(obs.nodes, 0, R_obs_abs_root, T_obs_abs_root, a_root, a_root_unused);
    }

    const int16_t conf_offset   = (lane >> 4) - 2;
    const int16_t rob_child_idx = (lane >> 2) & 0x3;
    const int16_t obs_child_idx = lane & 0x3;

    while (true) {
        // Per-warp batch pull: each warp grabs one 32-config word straight
        // from the global queue and broadcasts it within the warp. No
        // block-wide shared state, so no __syncthreads anywhere in the
        // kernel: warps run fully independently (fine-grained pulling also
        // balances the work, since a warp never waits for siblings).
        uint32_t warp_start = 0;
        if (lane == 0) {
            warp_start = atomicAdd(out.g_next_conf, WARP_BATCH);
        }
        warp_start = __shfl_sync(0xffffffffu, warp_start, 0);
        if (warp_start >= num_confs) {
            return;
        }

        // ---- parallel outermost check: one config per lane ---------------
        {
            const uint32_t index = warp_start + lane;
            uint8_t root_mask = 0;
            if (index < num_confs) {
                const articulated_conf<N> conf = pConf[index];

                // FK accumulators: matrix + incremental quaternion for
                // QuatSAT (hybrid), matrix form otherwise.
                Eigen::Matrix3f link_R;
                Eigen::Vector3f link_T = Eigen::Vector3f::Zero();
                Quat link_Q;
                if constexpr (LAYOUT == NodeLayout::QuatSAT) {
                    link_Q = Quat{0.0f, 0.0f, 0.0f, 1.0f};
                    link_R = Eigen::Matrix3f::Identity();
                } else {
                    link_R = Eigen::Matrix3f::Identity();
                }
                for (int l = 0; l < num_links; ++l) {
                    if (rob.linkOffset[l + 1] > rob.linkOffset[l]) {
                        const int rob_root = rob.linkOffset[l];
                        if (obs.gate_count == 1) {
                            // Fast path: single preloaded root node (default).
                            if constexpr (LAYOUT == NodeLayout::QuatSAT) {
                                Quat qRoot, qB;
                                Eigen::Vector3f robRootT, robRootDim, T;
                                float robRootA;
                                loadNodeQuat(rob.nodes, rob_root, qRoot, robRootT, robRootDim, robRootA);
                                computeRelTransformQuat(q_obs_root, T_obs_abs_root,
                                                        qRoot, robRootT,
                                                        link_Q, link_T, qB, T);
                                if (obbOverlapQuat(qB, T, a_root, robRootDim, epsilon)) {
                                    root_mask |= (1u << l);
                                }
                            } else {
                                Eigen::Matrix3f robRootR, B;
                                Eigen::Vector3f robRootT, robRootDim, T;
                                float robRootA;
                                loadNode(rob.nodes, rob_root, robRootR, robRootT, robRootDim, robRootA);
                                computeRelTransformNoBf(R_obs_abs_root, T_obs_abs_root,
                                                        robRootR, robRootT,
                                                        link_R, link_T, B, T);
                                if (obbOverlapAbs(a_root, robRootDim, B, epsilon, T)) {
                                    root_mask |= (1u << l);
                                }
                            }
                        } else {
                            // Multi-node gate (merged-BVH cut or per-obstacle
                            // roots): bit l = link root OBB overlaps >=1 gate
                            // node. Bit-clear links are provably collision-free
                            // (the gate covers every scene primitive) and skip
                            // the serial traversal. Early exit on the first
                            // overlap keeps proximal links cheap.
                            bool any = false;
                            if constexpr (LAYOUT == NodeLayout::QuatSAT) {
                                Quat qRoot, qG, qB;
                                Eigen::Vector3f robRootT, robRootDim, T, T_gate, a_gate;
                                float robRootA, aG;
                                loadNodeQuat(rob.nodes, rob_root, qRoot, robRootT, robRootDim, robRootA);
                                for (int g = 0; g < obs.gate_count; ++g) {
                                    loadNodeQuat(obs.gate_nodes, g, qG, T_gate, a_gate, aG);
                                    computeRelTransformQuat(qG, T_gate,
                                                            qRoot, robRootT,
                                                            link_Q, link_T, qB, T);
                                    if (obbOverlapQuat(qB, T, a_gate, robRootDim, epsilon)) {
                                        any = true;
                                        break;
                                    }
                                }
                            } else {
                                Eigen::Matrix3f robRootR, B, R_gate;
                                Eigen::Vector3f robRootT, robRootDim, T, T_gate, a_gate;
                                float robRootA, aG;
                                loadNode(rob.nodes, rob_root, robRootR, robRootT, robRootDim, robRootA);
                                for (int g = 0; g < obs.gate_count; ++g) {
                                    loadNode(obs.gate_nodes, g, R_gate, T_gate, a_gate, aG);
                                    computeRelTransformNoBf(R_gate, T_gate,
                                                            robRootR, robRootT,
                                                            link_R, link_T, B, T);
                                    if (obbOverlapAbs(a_gate, robRootDim, B, epsilon, T)) {
                                        any = true;
                                        break;
                                    }
                                }
                            }
                            if (any) {
                                root_mask |= (1u << l);
                            }
                        }
                    }
                    if (l + 1 < num_links) {
                        if constexpr (LAYOUT == NodeLayout::QuatSAT) {
                            // Hybrid FK: matrix accumulator (wide-ILP advance
                            // + cheap translation update) plus an incremental
                            // link quaternion for the pure-quat broad phase.
                            // The joint rotation matrix is built from the
                            // folded quat (c*A + s*B), so no Rodrigues and no
                            // origin_R matrix from global memory.
                            const JointParamsQuat& jp = rob.jointsQ[l];
                            const float h = 0.5f * conf[l];
                            const float c = cosf(h);
                            const float s = sinf(h);
                            Quat qj;
                            qj.x = c * jp.A.x + s * jp.B.x;
                            qj.y = c * jp.A.y + s * jp.B.y;
                            qj.z = c * jp.A.z + s * jp.B.z;
                            qj.w = c * jp.A.w + s * jp.B.w;
                            const Eigen::Matrix3f Rj = quatToMatrix(qj);
                            const Eigen::Vector3f nextT = link_T + link_R * jp.origin_T;
                            link_R = link_R * Rj;
                            link_Q = quatMul(link_Q, qj);
                            link_T = nextT;
                        } else {
                            const JointParams& jp = rob.joints[l];
                            const Eigen::Matrix3f Rj = axisAngleToRotation(jp.axis, conf[l]);
                            const Eigen::Matrix3f nextR = link_R * jp.origin_R * Rj;
                            const Eigen::Vector3f nextT = link_T + link_R * jp.origin_T;
                            link_R = nextR;
                            link_T = nextT;
                        }
                    }
                }

                if (root_mask == 0) {
                    atomicOr(&s_disjoint[warp], 1u << lane);
                } else {
                    const uint32_t pos = atomicAdd(&s_num_pend[warp], 1);
                    s_pend_idx[warp][pos] = index;
                    s_pend_root_mask[warp][pos] = (uint8_t)root_mask;
                    s_pend_lane[warp][pos] = (uint8_t)lane;
                }
            }
            __syncwarp();
            if (lane == 0) {
                out.pdisjoint[warp_start >> 5] |= s_disjoint[warp];
                s_disjoint[warp] = 0;
            }
        }

        // ---- per-warp serial phase: each warp traverses its pending configs
        for (uint32_t k = 0; k < s_num_pend[warp]; ++k) {
            const uint32_t index = s_pend_idx[warp][k];
            const uint32_t root_mask = s_pend_root_mask[warp][k];
            const uint32_t owner = s_pend_lane[warp][k];
            const articulated_conf<N> conf = pConf[index];

            if (lane == 0){
                s_collision[warp] = false;
            }
            __syncwarp();

            // FK accumulators: meaningful only on the owner lane; broadcast
            // to the whole warp at each link (computed once per config, by
            // its owner lane). Quaternion form for QuatSAT (no matrix
            // materialized, no matrixToQuat), matrix form otherwise.
            Eigen::Matrix3f ownR;
            Eigen::Vector3f ownT = Eigen::Vector3f::Zero();
            Quat ownQ;
            if constexpr (LAYOUT == NodeLayout::QuatSAT) {
                ownQ = Quat{0.0f, 0.0f, 0.0f, 1.0f};
                ownR = Eigen::Matrix3f::Identity();
            } else {
                ownR = Eigen::Matrix3f::Identity();
            }

            for (int l = 0; l < num_links; ++l) {
                const bool hasMesh = (rob.linkOffset[l + 1] > rob.linkOffset[l]);
                const bool rootHit = ((root_mask >> l) & 1u) != 0;

                // Links that are provably collision-free (no mesh, or the
                // root pair missed) skip the FK broadcast + traversal, but
                // the FK accumulators must still advance so later links see
                // the correct parent transform.
                if (!(hasMesh && rootHit)) {
                    if (l + 1 < num_links) {
                        if constexpr (LAYOUT == NodeLayout::QuatSAT) {
                            const JointParamsQuat& jp = rob.jointsQ[l];
                            const float h = 0.5f * conf[l];
                            const float c = cosf(h);
                            const float s = sinf(h);
                            Quat qj;
                            qj.x = c * jp.A.x + s * jp.B.x;
                            qj.y = c * jp.A.y + s * jp.B.y;
                            qj.z = c * jp.A.z + s * jp.B.z;
                            qj.w = c * jp.A.w + s * jp.B.w;
                            const Eigen::Matrix3f Rj = quatToMatrix(qj);
                            const Eigen::Vector3f nextT = ownT + ownR * jp.origin_T;
                            ownR = ownR * Rj;
                            ownQ = quatMul(ownQ, qj);
                            ownT = nextT;
                        } else {
                            const JointParams& jp = rob.joints[l];
                            const Eigen::Matrix3f Rj = axisAngleToRotation(jp.axis, conf[l]);
                            const Eigen::Matrix3f nextR = ownR * jp.origin_R * Rj;
                            const Eigen::Vector3f nextT = ownT + ownR * jp.origin_T;
                            ownR = nextR;
                            ownT = nextT;
                        }
                    }
                    continue;
                }
                Eigen::Matrix3f link_R;
                Eigen::Vector3f link_T;
                Quat q_link;
                if constexpr (LAYOUT == NodeLayout::QuatSAT) {
                    // FK is accumulated in quaternion form; broadcast only
                    // the quat + translation (no matrix materialization).
#pragma unroll
                    for (int i = 0; i < 4; ++i) {
                        (&q_link.x)[i] = __shfl_sync(0xffffffffu, (&ownQ.x)[i], owner, 32);
                    }
#pragma unroll
                    for (int i = 0; i < 3; ++i) {
                        link_T[i] = __shfl_sync(0xffffffffu, ownT[i], owner, 32);
                    }
                } else {
                    //TODO: remove hardcoded numbers here
#pragma unroll
                    for (int i = 0; i < 9; ++i) {
                        link_R.data()[i] = __shfl_sync(0xffffffffu, ownR.data()[i], owner, 32);
                    }
#pragma unroll
                    for (int i = 0; i < 3; ++i) {
                        link_T[i] = __shfl_sync(0xffffffffu, ownT[i], owner, 32);
                    }
                }
                //TODO: what is the point of the hash mesh thing here?

                // Seed the frontier with the (link root, scene root) pair;
                // root overlap was proven in the parallel phase (mask).
                if (lane == 0) {
                    s_num_tri[warp] = 0;
                    s_rob_pend[warp][0] = (QRob)rob.linkOffset[l];
                    s_obs_pend[warp][0] = 0;
                    s_num_obb_pend[warp] = 1;
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
                            d_articulated_tri_phase<RobChildT, ObsChildT, QRob, QObs, LAYOUT>(
                                s_tri_rob[warp], s_tri_obs[warp], s_num_tri[warp],
                                link_R, q_link, link_T,
                                rob.linkVertOffset[l], rob.linkTriOffset[l],
                                obs, rob,
                                &s_collision[warp]);
                            if (lane == 0) {
                                s_num_tri[warp] = 0;
                            }
                            continue;
                        }
                        if (s_num_obb_pend[warp] == 0) {
                            break;
                        }
                        if (s_num_obb_pend[warp] >= MAX_BUFFER - 32) {
                            if (lane == 0) {
                                atomicAdd(out.overflowCounter, 1ull);
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

                        //TODO: shouldn't there be a syncwarp here
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
                        const int rob_fc_par = rob.first_child[rob_obb_par_idx];
                        const int obs_fc_par = obs.first_child[obs_obb_par_idx];

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

                        const int rob_first_child_idx = rob.first_child[rob_obb_idx];
                        const int obs_first_child_idx = obs.first_child[obs_obb_idx];
                        if (rob_first_child_idx == 0 || obs_first_child_idx == 0) {
                            continue; // dummy (padding) node
                        }

                        if (LAYOUT == NodeLayout::QuatSAT) {
                            Quat qObs, qRob, qB;
                            Eigen::Vector3f T_obs_abs, T_rob_abs, b, a;
                            float aObs, aRob;
                            loadNodeQuat(obs.nodes, obs_obb_idx, qObs, T_obs_abs, a, aObs);
                            loadNodeQuat(rob.nodes, rob_obb_idx, qRob, T_rob_abs, b, aRob);
                            Eigen::Vector3f T;
                            computeRelTransformQuat(qObs, T_obs_abs, qRob, T_rob_abs,
                                                    q_link, link_T, qB, T);
                            if (!obbOverlapQuat(qB, T, a, b, epsilon)) {
                                continue;
                            }
                        } else {
                            Eigen::Matrix3f R_obs_abs, R_rob_abs;
                            Eigen::Vector3f T_obs_abs, T_rob_abs, b, a;
                            float aObs, aRob;
                            loadNode(obs.nodes, obs_obb_idx, R_obs_abs, T_obs_abs, a, aObs);
                            loadNode(rob.nodes, rob_obb_idx, R_rob_abs, T_rob_abs, b, aRob);

                            Eigen::Matrix3f B;
                            Eigen::Vector3f T;
                            computeRelTransformNoBf(R_obs_abs, T_obs_abs, R_rob_abs, T_rob_abs,
                                                    link_R, link_T, B, T);
                            if (!obbOverlapAbs(a, b, B, epsilon, T)) {
                                continue;
                            }
                        }

                        if (rob_first_child_idx < 0 && obs_first_child_idx < 0) {
                            // Both leaves: defer to the triangle phase.
                            const int pos = atomicAdd(&s_num_tri[warp], 1);
                            if (pos < MAX_TRI) {
                                s_tri_rob[warp][pos] = (QRob)rob_obb_idx;
                                s_tri_obs[warp][pos] = (QObs)obs_obb_idx;
                            } else if (lane == 0) {
                                atomicAdd(out.overflowCounter, 1ull);
                                s_collision[warp] = true; // conservative
                            }
                        } else {
                            // One or both internal: expand (leaf-aware).
                            const int pos = atomicAdd(&s_num_obb_pend[warp], 1);
                            if (pos < MAX_BUFFER) {
                                s_obs_pend[warp][pos] = (QObs)obs_obb_idx;
                                s_rob_pend[warp][pos] = (QRob)rob_obb_idx;
                            } else if (lane == 0) {
                                atomicAdd(out.overflowCounter, 1ull);
                                s_collision[warp] = true; // conservative
                            }
                        }
                    } // OBB traversal of this link

                    __syncwarp();

                    // Final triangle phase for the leftover candidates.
                    if (!s_collision[warp] && s_num_tri[warp] > 0) {
                        d_articulated_tri_phase<RobChildT, ObsChildT, QRob, QObs, LAYOUT>(
                                s_tri_rob[warp], s_tri_obs[warp], s_num_tri[warp],
                                link_R, q_link, link_T,
                                rob.linkVertOffset[l], rob.linkTriOffset[l],
                                obs, rob,
                                &s_collision[warp]);
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
                    if constexpr (LAYOUT == NodeLayout::QuatSAT) {
                        const JointParamsQuat& jp = rob.jointsQ[l];
                        const float h = 0.5f * conf[l];
                        const float c = cosf(h);
                        const float s = sinf(h);
                        Quat qj;
                        qj.x = c * jp.A.x + s * jp.B.x;
                        qj.y = c * jp.A.y + s * jp.B.y;
                        qj.z = c * jp.A.z + s * jp.B.z;
                        qj.w = c * jp.A.w + s * jp.B.w;
                        const Eigen::Matrix3f Rj = quatToMatrix(qj);
                        const Eigen::Vector3f nextT = ownT + ownR * jp.origin_T;
                        ownR = ownR * Rj;
                        ownQ = quatMul(ownQ, qj);
                        ownT = nextT;
                    } else {
                        const JointParams& jp = rob.joints[l];
                        const Eigen::Matrix3f Rj = axisAngleToRotation(jp.axis, conf[l]);
                        const Eigen::Matrix3f nextR = ownR * jp.origin_R * Rj;
                        const Eigen::Vector3f nextT = ownT + ownR * jp.origin_T;
                        ownR = nextR;
                        ownT = nextT;
                    }
                }
            } // links

            __syncwarp();
            if (!s_collision[warp]) {
                if (lane == 0) {
                    s_disjoint[warp] |= 1u << (index & 31);
                }
            }
            __syncwarp();
        } // per-warp pending configurations

        __syncwarp();
        if (lane == 0) {
            out.pdisjoint[warp_start >> 5] |= s_disjoint[warp];
            s_disjoint[warp] = 0;
            s_num_pend[warp] = 0;
        }
    }
    return;
}

// Two entry wrappers around d_bvh_articulated_body:
//   d_bvh_articulated        = unconstrained registers (1 block/SM at 190 regs)
//   d_bvh_articulated_2bsm   = __launch_bounds__(256, 2) caps registers at 128
//                              so 2 blocks co-reside per SM. Only worth it when
//                              the grid exceeds the SM count (e.g. Orin Nano).
template <size_t N, typename RobChildT, typename ObsChildT, bool RESTRICT_PTRS, NodeLayout LAYOUT>
__global__ void d_bvh_articulated(const ObstacleSoA<ObsChildT, LAYOUT> obs,
                                  const RobotSoA<RobChildT, LAYOUT> rob,
                                  const articulated_conf<N>* pConf, size_t num_confs,
                                  const KernelOut out) {
    d_bvh_articulated_body<N, RobChildT, ObsChildT, RESTRICT_PTRS, LAYOUT>(
        obs, rob, pConf, num_confs, out);
}

template <size_t N, typename RobChildT, typename ObsChildT, bool RESTRICT_PTRS, NodeLayout LAYOUT>
__global__ void __launch_bounds__(256, 2) d_bvh_articulated_2bsm(const ObstacleSoA<ObsChildT, LAYOUT> obs,
                                  const RobotSoA<RobChildT, LAYOUT> rob,
                                  const articulated_conf<N>* pConf, size_t num_confs,
                                  const KernelOut out) {
    d_bvh_articulated_body<N, RobChildT, ObsChildT, RESTRICT_PTRS, LAYOUT>(
        obs, rob, pConf, num_confs, out);
}



template <size_t N>
double bvh_articulated(const std::string& robot_urdf_path,
                       const BVNode_soa<int32_t>& obs_BVH, const MeshData& obs_mesh,
                       const std::vector<articulated_conf<N>>& confs,
                       std::vector<bool>& valid, bool dry_run, int kernel_mode,
                       const std::vector<Eigen::Matrix3f>* gate_R,
                       const std::vector<Eigen::Vector3f>* gate_T,
                       const std::vector<Eigen::Vector3f>* gate_dim) {
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

    // ---- A/B/C/BC variant selection (int16 applied per side where it fits) --
    const bool want16 = (kernel_mode & 1) != 0;
    const bool want_restrict = (kernel_mode & 2) != 0;
    // bit 6 (64) = pure-quaternion broad phase (NodeLayout::QuatSAT): QuatTD
    // storage, but the quaternion is never converted to a matrix during
    // traversal; the 15 SAT B entries come straight from the quat components.
    const NodeLayout layout = (kernel_mode & 64)
        ? NodeLayout::QuatSAT
        : static_cast<NodeLayout>((kernel_mode >> 2) & 3);
    const bool rob16 = want16 && (rob_first_child.size() < 32768);
    const bool obs16 = want16 && (obs_BVH.size < 32768);
    if (want16 && !(rob16 && obs16)) {
        std::cout << "bvh_articulated: int16 requested but node counts exceed 15 bits"
                  << " (obs=" << obs_BVH.size << ", rob=" << rob_first_child.size()
                  << "); using int16 only where it fits." << std::endl;
    }
    // QuatTD/QuatSAT pack T/dim/a into the TD float4s: the split arrays are
    // never read by those kernels, so they are neither allocated nor copied
    // (memory minimization).
    const bool needsSplitArrays = (layout != NodeLayout::QuatTD && layout != NodeLayout::QuatSAT);
    static const char* kLayoutName[5] = {"matrix", "quat", "vecR", "quat+TD", "quatSAT"};
    // 2-blocks/SM (128-reg cap) only pays off when the grid can exceed the SM
    // count; bit 4 forces it for benchmarking.
    const bool force2bsm = (kernel_mode & 16) != 0;
    std::cout << "bvh_articulated: kernel variant = rob<"
              << (rob16 ? "int16" : "int32") << "> obs<"
              << (obs16 ? "int16" : "int32") << "> layout="
              << kLayoutName[static_cast<int>(layout)]
              << (want_restrict ? " +restrict" : "") << std::endl;

    std::vector<int16_t> obs_first_child16, rob_first_child16;
    if (obs16) {
        obs_first_child16.reserve(obs_BVH.size);
        for (size_t i = 0; i < obs_BVH.size; ++i) {
            obs_first_child16.push_back((int16_t)obs_BVH.first_child[i]);
        }
    }
    if (rob16) {
        rob_first_child16.reserve(rob_first_child.size());
        for (size_t i = 0; i < rob_first_child.size(); ++i) {
            rob_first_child16.push_back((int16_t)rob_first_child[i]);
        }
    }

    // ---- Node-data packing for layouts 1-4 --------------------------------
    std::vector<float4> obs_Rq, obs_Rp, obs_TD, rob_Rq, rob_Rp, rob_TD;
    if (layout == NodeLayout::Quat || layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        obs_Rq.reserve(obs_BVH.size);
        for (size_t i = 0; i < obs_BVH.size; ++i) {
            const Eigen::Quaternionf q(obs_BVH.pR[i]);
            obs_Rq.push_back(make_float4(q.x(), q.y(), q.z(), q.w()));
        }
        rob_Rq.reserve(rob_R.size());
        for (size_t i = 0; i < rob_R.size(); ++i) {
            const Eigen::Quaternionf q(rob_R[i]);
            rob_Rq.push_back(make_float4(q.x(), q.y(), q.z(), q.w()));
        }
    }
    if (layout == NodeLayout::VecR) {
        obs_Rp.reserve(obs_BVH.size * 3);
        for (size_t i = 0; i < obs_BVH.size; ++i) {
            const Eigen::Matrix3f& R = obs_BVH.pR[i];
            obs_Rp.push_back(make_float4(R(0, 0), R(0, 1), R(0, 2), 0.0f));
            obs_Rp.push_back(make_float4(R(1, 0), R(1, 1), R(1, 2), 0.0f));
            obs_Rp.push_back(make_float4(R(2, 0), R(2, 1), R(2, 2), 0.0f));
        }
        rob_Rp.reserve(rob_R.size() * 3);
        for (size_t i = 0; i < rob_R.size(); ++i) {
            const Eigen::Matrix3f& R = rob_R[i];
            rob_Rp.push_back(make_float4(R(0, 0), R(0, 1), R(0, 2), 0.0f));
            rob_Rp.push_back(make_float4(R(1, 0), R(1, 1), R(1, 2), 0.0f));
            rob_Rp.push_back(make_float4(R(2, 0), R(2, 1), R(2, 2), 0.0f));
        }
    }
    if (layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        obs_TD.reserve(obs_BVH.size * 2);
        for (size_t i = 0; i < obs_BVH.size; ++i) {
            const Eigen::Vector3f& T = obs_BVH.pT[i];
            const Eigen::Vector3f& d = obs_BVH.pDim[i];
            obs_TD.push_back(make_float4(T.x(), T.y(), T.z(), d.x()));
            obs_TD.push_back(make_float4(d.y(), d.z(), obs_BVH.pA[i], 0.0f));
        }
        rob_TD.reserve(rob_R.size() * 2);
        for (size_t i = 0; i < rob_R.size(); ++i) {
            const Eigen::Vector3f& T = rob_T[i];
            const Eigen::Vector3f& d = rob_dim[i];
            rob_TD.push_back(make_float4(T.x(), T.y(), T.z(), d.x()));
            rob_TD.push_back(make_float4(d.y(), d.z(), rob_a[i], 0.0f));
        }
    }
    // ---- Parallel-phase gate nodes (default: obs root only) ----------------
    // A link is gated iff its root OBB overlaps >=1 gate node; links that
    // overlap none skip the serial traversal. The caller guarantees the gate
    // covers every scene primitive (BVH cut at level k, or per-obstacle
    // roots). Node list order is preserved (pass largest-first for early
    // positive exit on proximal links).
    std::vector<Eigen::Matrix3f> gate_R_vec;
    std::vector<Eigen::Vector3f> gate_T_vec, gate_dim_vec;
    if (gate_R && !gate_R->empty()) {
        gate_R_vec = *gate_R;
        gate_T_vec = *gate_T;
        gate_dim_vec = *gate_dim;
    } else {
        gate_R_vec.push_back(obs_BVH.pR[0]);
        gate_T_vec.push_back(obs_BVH.pT[0]);
        gate_dim_vec.push_back(obs_BVH.pDim[0]);
    }
    const int gate_count = (int)gate_R_vec.size();
    std::cout << "bvh_articulated: parallel-phase gate = " << gate_count
              << " node(s)" << (gate_count == 1 ? " (root)" : "") << std::endl;

    std::vector<float4> gate_Rq, gate_Rp, gate_TD;
    std::vector<Eigen::Vector3f> gate_T_split, gate_dim_split;
    std::vector<float> gate_a;
    if (layout == NodeLayout::Quat || layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        gate_Rq.reserve(gate_count);
        for (int i = 0; i < gate_count; ++i) {
            const Eigen::Quaternionf q(gate_R_vec[i]);
            gate_Rq.push_back(make_float4(q.x(), q.y(), q.z(), q.w()));
        }
    }
    if (layout == NodeLayout::VecR) {
        gate_Rp.reserve(gate_count * 3);
        for (int i = 0; i < gate_count; ++i) {
            const Eigen::Matrix3f& R = gate_R_vec[i];
            gate_Rp.push_back(make_float4(R(0, 0), R(0, 1), R(0, 2), 0.0f));
            gate_Rp.push_back(make_float4(R(1, 0), R(1, 1), R(1, 2), 0.0f));
            gate_Rp.push_back(make_float4(R(2, 0), R(2, 1), R(2, 2), 0.0f));
        }
    }
    if (layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        gate_TD.reserve(gate_count * 2);
        for (int i = 0; i < gate_count; ++i) {
            const Eigen::Vector3f& T = gate_T_vec[i];
            const Eigen::Vector3f& d = gate_dim_vec[i];
            gate_TD.push_back(make_float4(T.x(), T.y(), T.z(), d.x()));
            // a is unused by the gate SAT (Chang-Kim leaf parameter only
            // matters in the narrow phase).
            gate_TD.push_back(make_float4(d.y(), d.z(), 0.0f, 0.0f));
        }
    }
    if (needsSplitArrays) {
        gate_T_split = gate_T_vec;
        gate_dim_split = gate_dim_vec;
        gate_a.assign(gate_count, 0.0f);
    }

    // Matrix-free joint data for QuatSAT: no Eigen matrix in global memory.
    std::vector<JointParamsQuat> joints_q;
    if (layout == NodeLayout::QuatSAT) {
        joints_q.reserve(joints.size());
        for (const auto& jp : joints) {
            const Eigen::Quaternionf q(jp.origin_R);
            const Quat origin_Q = Quat{q.x(), q.y(), q.z(), q.w()};
            JointParamsQuat jq;
            jq.A = origin_Q;
            jq.B = quatMul(origin_Q, Quat{jp.axis.x(), jp.axis.y(), jp.axis.z(), 0.0f});
            jq.origin_T = jp.origin_T;
            joints_q.push_back(jq);
        }
    }

    const int blockSize = 256;
    const size_t num_words = (num_confs + 31) / 32;

    // Persistent blocks pulling configs from a global work queue (same sizing
    // scheme as bvh_naive). No dynamic shared memory: first_child arrays are
    // read from global memory, so the static pend buffers bound occupancy.
    int device;
    checkCudaMem(cudaGetDevice(&device));
    int num_sms = 0;
    checkCudaMem(cudaDeviceGetAttribute(&num_sms, cudaDevAttrMultiProcessorCount, device));
    // 2-blocks/SM (128-reg cap) only pays off when the grid can exceed the SM
    // count (e.g. Orin Nano: 33 batches > 8 SMs).
    const bool force1bsm = (kernel_mode & 32) != 0;
    const bool use2bsm = force2bsm || (!force1bsm && (int)((num_confs + 255) / 256) > num_sms);
    std::cout << "bvh_articulated: occupancy mode = "
              << (use2bsm ? "2 blocks/SM (128-reg cap)" : "1 block/SM (uncapped)") << std::endl;
    int blocks_per_sm = 0;
    {
        const auto occForLayout = [&](auto layoutTag) {
            constexpr NodeLayout L = decltype(layoutTag)::value;
            const auto occ = [&](auto kernel) {
                cudaFuncSetAttribute(kernel, cudaFuncAttributePreferredSharedMemoryCarveout, 100);
                checkCudaMem(cudaOccupancyMaxActiveBlocksPerMultiprocessor(
                    &blocks_per_sm, kernel, blockSize, 0));
            };
            const auto pickOcc = [&](auto k1bsm, auto k2bsm) {
                if (use2bsm) occ(k2bsm);
                else occ(k1bsm);
            };
            if (rob16 && obs16) {
                if (want_restrict) pickOcc(d_bvh_articulated<N, int16_t, int16_t, true, L>, d_bvh_articulated_2bsm<N, int16_t, int16_t, true, L>);
                else pickOcc(d_bvh_articulated<N, int16_t, int16_t, false, L>, d_bvh_articulated_2bsm<N, int16_t, int16_t, false, L>);
            } else if (rob16) {
                if (want_restrict) pickOcc(d_bvh_articulated<N, int16_t, int32_t, true, L>, d_bvh_articulated_2bsm<N, int16_t, int32_t, true, L>);
                else pickOcc(d_bvh_articulated<N, int16_t, int32_t, false, L>, d_bvh_articulated_2bsm<N, int16_t, int32_t, false, L>);
            } else if (obs16) {
                if (want_restrict) pickOcc(d_bvh_articulated<N, int32_t, int16_t, true, L>, d_bvh_articulated_2bsm<N, int32_t, int16_t, true, L>);
                else pickOcc(d_bvh_articulated<N, int32_t, int16_t, false, L>, d_bvh_articulated_2bsm<N, int32_t, int16_t, false, L>);
            } else {
                if (want_restrict) pickOcc(d_bvh_articulated<N, int32_t, int32_t, true, L>, d_bvh_articulated_2bsm<N, int32_t, int32_t, true, L>);
                else pickOcc(d_bvh_articulated<N, int32_t, int32_t, false, L>, d_bvh_articulated_2bsm<N, int32_t, int32_t, false, L>);
            }
        };
        switch (layout) {
            case NodeLayout::Matrix: occForLayout(std::integral_constant<NodeLayout, NodeLayout::Matrix>()); break;
            case NodeLayout::Quat:   occForLayout(std::integral_constant<NodeLayout, NodeLayout::Quat>()); break;
            case NodeLayout::VecR:   occForLayout(std::integral_constant<NodeLayout, NodeLayout::VecR>()); break;
            case NodeLayout::QuatSAT: occForLayout(std::integral_constant<NodeLayout, NodeLayout::QuatSAT>()); break;
            default:                 occForLayout(std::integral_constant<NodeLayout, NodeLayout::QuatTD>()); break;
        }
    }
    if (blocks_per_sm < 1) blocks_per_sm = 1;
    const int max_blocks = (int)((num_confs + blockSize - 1) / blockSize);
    const int persistent_blocks = num_sms * blocks_per_sm;
    const int gridSize = (max_blocks < persistent_blocks) ? max_blocks : persistent_blocks;

    Eigen::Matrix3f* d_R_obs = nullptr;
    Eigen::Vector3f* d_T_obs = nullptr;
    Eigen::Vector3f* d_Obs_dim = nullptr;
    int32_t* d_Obs_first_child32;
    int16_t* d_Obs_first_child16;
    Eigen::Vector3f* d_Obs_verts;
    Triangle* d_Obs_tris;
    float* d_Obs_a = nullptr;

    Eigen::Matrix3f* d_Rob_R = nullptr;
    Eigen::Vector3f* d_Rob_T = nullptr;
    Eigen::Vector3f* d_Rob_dim = nullptr;
    int32_t* d_Rob_first_child32;
    int16_t* d_Rob_first_child16;
    float* d_Rob_a = nullptr;
    Eigen::Vector3f* d_Rob_verts;
    Triangle* d_Rob_tris;

    float4* d_Obs_Rq;
    float4* d_Obs_Rp;
    float4* d_Obs_TD;
    float4* d_Rob_Rq;
    float4* d_Rob_Rp;
    float4* d_Rob_TD;

    float4* d_Gate_Rq;
    float4* d_Gate_Rp;
    float4* d_Gate_TD;
    Eigen::Matrix3f* d_Gate_R;
    Eigen::Vector3f* d_Gate_T;
    Eigen::Vector3f* d_Gate_dim;
    float* d_Gate_a;

    int* d_LinkOffset;
    int* d_LinkVertOffset;
    int* d_LinkTriOffset;
    JointParams* d_Joints = nullptr;
    JointParamsQuat* d_JointsQ = nullptr;
    articulated_conf<N>* d_Conf;
    uint32_t* d_disjoint;
    uint32_t* d_next_conf;
    unsigned long long* d_overflow;

    cudaEventRecord(start, 0);

    if (layout == NodeLayout::Matrix) {
        cudaMalloc((void**)&d_R_obs, obs_BVH.size * sizeof(Eigen::Matrix3f));
    }
    if (needsSplitArrays) {
        cudaMalloc((void**)&d_T_obs, obs_BVH.size * sizeof(Eigen::Vector3f));
        cudaMalloc((void**)&d_Obs_dim, obs_BVH.size * sizeof(Eigen::Vector3f));
        cudaMalloc((void**)&d_Obs_a, obs_BVH.size * sizeof(float));
    }
    if (obs16) {
        cudaMalloc((void**)&d_Obs_first_child16, obs_BVH.size * sizeof(int16_t));
    } else {
        cudaMalloc((void**)&d_Obs_first_child32, obs_BVH.size * sizeof(int32_t));
    }
    cudaMalloc((void**)&d_Obs_verts, obs_mesh.vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_tris, obs_mesh.triangles.size() * sizeof(Triangle));

    if (layout == NodeLayout::Matrix) {
        cudaMalloc((void**)&d_Rob_R, rob_R.size() * sizeof(Eigen::Matrix3f));
    }
    if (needsSplitArrays) {
        cudaMalloc((void**)&d_Rob_T, rob_T.size() * sizeof(Eigen::Vector3f));
        cudaMalloc((void**)&d_Rob_dim, rob_dim.size() * sizeof(Eigen::Vector3f));
        cudaMalloc((void**)&d_Rob_a, rob_a.size() * sizeof(float));
    }
    if (rob16) {
        cudaMalloc((void**)&d_Rob_first_child16, rob_first_child.size() * sizeof(int16_t));
    } else {
        cudaMalloc((void**)&d_Rob_first_child32, rob_first_child.size() * sizeof(int32_t));
    }
    cudaMalloc((void**)&d_Rob_verts, rob_verts.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_tris, rob_tris.size() * sizeof(Triangle));

    d_Obs_Rq = d_Obs_Rp = d_Obs_TD = nullptr;
    d_Rob_Rq = d_Rob_Rp = d_Rob_TD = nullptr;
    if (layout == NodeLayout::Quat || layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        cudaMalloc((void**)&d_Obs_Rq, obs_Rq.size() * sizeof(float4));
        cudaMalloc((void**)&d_Rob_Rq, rob_Rq.size() * sizeof(float4));
    }
    if (layout == NodeLayout::VecR) {
        cudaMalloc((void**)&d_Obs_Rp, obs_Rp.size() * sizeof(float4));
        cudaMalloc((void**)&d_Rob_Rp, rob_Rp.size() * sizeof(float4));
    }
    if (layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        cudaMalloc((void**)&d_Obs_TD, obs_TD.size() * sizeof(float4));
        cudaMalloc((void**)&d_Rob_TD, rob_TD.size() * sizeof(float4));
    }

    d_Gate_Rq = d_Gate_Rp = d_Gate_TD = nullptr;
    d_Gate_R = nullptr;
    d_Gate_T = d_Gate_dim = nullptr;
    d_Gate_a = nullptr;
    if (layout == NodeLayout::Quat || layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        cudaMalloc((void**)&d_Gate_Rq, gate_Rq.size() * sizeof(float4));
    }
    if (layout == NodeLayout::VecR) {
        cudaMalloc((void**)&d_Gate_Rp, gate_Rp.size() * sizeof(float4));
    }
    if (layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        cudaMalloc((void**)&d_Gate_TD, gate_TD.size() * sizeof(float4));
    }
    if (layout == NodeLayout::Matrix) {
        cudaMalloc((void**)&d_Gate_R, gate_count * sizeof(Eigen::Matrix3f));
    }
    if (needsSplitArrays) {
        cudaMalloc((void**)&d_Gate_T, gate_count * sizeof(Eigen::Vector3f));
        cudaMalloc((void**)&d_Gate_dim, gate_count * sizeof(Eigen::Vector3f));
        cudaMalloc((void**)&d_Gate_a, gate_count * sizeof(float));
    }

    cudaMalloc((void**)&d_LinkOffset, (num_links + 1) * sizeof(int));
    cudaMalloc((void**)&d_LinkVertOffset, (num_links + 1) * sizeof(int));
    cudaMalloc((void**)&d_LinkTriOffset, (num_links + 1) * sizeof(int));
    if (layout == NodeLayout::QuatSAT) {
        cudaMalloc((void**)&d_JointsQ, N * sizeof(JointParamsQuat));
    } else {
        cudaMalloc((void**)&d_Joints, N * sizeof(JointParams));
    }
    cudaMalloc((void**)&d_Conf, num_confs * sizeof(articulated_conf<N>));
    cudaMalloc((void**)&d_disjoint, num_words * sizeof(uint32_t));
    cudaMalloc((void**)&d_next_conf, sizeof(uint32_t));
    cudaMalloc((void**)&d_overflow, sizeof(unsigned long long));

    if (layout == NodeLayout::Matrix) {
        checkCudaMem(cudaMemcpy(d_R_obs, obs_BVH.pR, obs_BVH.size * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    }
    if (needsSplitArrays) {
        checkCudaMem(cudaMemcpy(d_T_obs, obs_BVH.pT, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_Obs_dim, obs_BVH.pDim, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_Obs_a, obs_BVH.pA, obs_BVH.size * sizeof(float), cudaMemcpyHostToDevice));
    }
    if (obs16) {
        checkCudaMem(cudaMemcpy(d_Obs_first_child16, obs_first_child16.data(), obs_BVH.size * sizeof(int16_t), cudaMemcpyHostToDevice));
    } else {
        checkCudaMem(cudaMemcpy(d_Obs_first_child32, obs_BVH.first_child, obs_BVH.size * sizeof(int32_t), cudaMemcpyHostToDevice));
    }
    checkCudaMem(cudaMemcpy(d_Obs_verts, obs_mesh.vertices.data(), obs_mesh.vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_tris, obs_mesh.triangles.data(), obs_mesh.triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));

    if (layout == NodeLayout::Matrix) {
        checkCudaMem(cudaMemcpy(d_Rob_R, rob_R.data(), rob_R.size() * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    }
    if (needsSplitArrays) {
        checkCudaMem(cudaMemcpy(d_Rob_T, rob_T.data(), rob_T.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_Rob_dim, rob_dim.data(), rob_dim.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_Rob_a, rob_a.data(), rob_a.size() * sizeof(float), cudaMemcpyHostToDevice));
    }
    if (rob16) {
        checkCudaMem(cudaMemcpy(d_Rob_first_child16, rob_first_child16.data(), rob_first_child.size() * sizeof(int16_t), cudaMemcpyHostToDevice));
    } else {
        checkCudaMem(cudaMemcpy(d_Rob_first_child32, rob_first_child.data(), rob_first_child.size() * sizeof(int32_t), cudaMemcpyHostToDevice));
    }
    checkCudaMem(cudaMemcpy(d_Rob_verts, rob_verts.data(), rob_verts.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_tris, rob_tris.data(), rob_tris.size() * sizeof(Triangle), cudaMemcpyHostToDevice));

    if (layout == NodeLayout::Quat || layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        checkCudaMem(cudaMemcpy(d_Obs_Rq, obs_Rq.data(), obs_Rq.size() * sizeof(float4), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_Rob_Rq, rob_Rq.data(), rob_Rq.size() * sizeof(float4), cudaMemcpyHostToDevice));
    }
    if (layout == NodeLayout::VecR) {
        checkCudaMem(cudaMemcpy(d_Obs_Rp, obs_Rp.data(), obs_Rp.size() * sizeof(float4), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_Rob_Rp, rob_Rp.data(), rob_Rp.size() * sizeof(float4), cudaMemcpyHostToDevice));
    }
    if (layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        checkCudaMem(cudaMemcpy(d_Obs_TD, obs_TD.data(), obs_TD.size() * sizeof(float4), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_Rob_TD, rob_TD.data(), rob_TD.size() * sizeof(float4), cudaMemcpyHostToDevice));
    }

    if (layout == NodeLayout::Quat || layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        checkCudaMem(cudaMemcpy(d_Gate_Rq, gate_Rq.data(), gate_Rq.size() * sizeof(float4), cudaMemcpyHostToDevice));
    }
    if (layout == NodeLayout::VecR) {
        checkCudaMem(cudaMemcpy(d_Gate_Rp, gate_Rp.data(), gate_Rp.size() * sizeof(float4), cudaMemcpyHostToDevice));
    }
    if (layout == NodeLayout::QuatTD || layout == NodeLayout::QuatSAT) {
        checkCudaMem(cudaMemcpy(d_Gate_TD, gate_TD.data(), gate_TD.size() * sizeof(float4), cudaMemcpyHostToDevice));
    }
    if (layout == NodeLayout::Matrix) {
        checkCudaMem(cudaMemcpy(d_Gate_R, gate_R_vec.data(), gate_count * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    }
    if (needsSplitArrays) {
        checkCudaMem(cudaMemcpy(d_Gate_T, gate_T_split.data(), gate_count * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_Gate_dim, gate_dim_split.data(), gate_count * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
        checkCudaMem(cudaMemcpy(d_Gate_a, gate_a.data(), gate_count * sizeof(float), cudaMemcpyHostToDevice));
    }

    checkCudaMem(cudaMemcpy(d_LinkOffset, link_offset.data(), (num_links + 1) * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_LinkVertOffset, link_vert_offset.data(), (num_links + 1) * sizeof(int), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_LinkTriOffset, link_tri_offset.data(), (num_links + 1) * sizeof(int), cudaMemcpyHostToDevice));
    if (layout == NodeLayout::QuatSAT) {
        checkCudaMem(cudaMemcpy(d_JointsQ, joints_q.data(), N * sizeof(JointParamsQuat), cudaMemcpyHostToDevice));
    } else {
        checkCudaMem(cudaMemcpy(d_Joints, joints.data(), N * sizeof(JointParams), cudaMemcpyHostToDevice));
    }
    checkCudaMem(cudaMemcpy(d_Conf, confs.data(), num_confs * sizeof(articulated_conf<N>), cudaMemcpyHostToDevice));
    cudaDeviceSynchronize();

    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float duration = 0;
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Articulated BVH allocation and transfer to GPU took " << duration << " ms." << std::endl;

    KernelOut kout;
    kout.pdisjoint = d_disjoint;
    kout.g_next_conf = d_next_conf;
    kout.overflowCounter = d_overflow;

    // Per-layout SoA construction: the layout-specialized NodePtrsFor<L>
    // bundles hold exactly the pointers that layout uses (no nulls).
    const auto makeObs = [&](auto layoutTag, const auto* fc) {
        constexpr NodeLayout L = decltype(layoutTag)::value;
        ObstacleSoA<std::decay_t<decltype(*fc)>, L> o;
        if constexpr (L == NodeLayout::Matrix) {
            o.nodes.R = d_R_obs;
            o.gate_nodes.R = d_Gate_R;
        } else if constexpr (L == NodeLayout::Quat) {
            o.nodes.Rq = d_Obs_Rq;
            o.gate_nodes.Rq = d_Gate_Rq;
        } else if constexpr (L == NodeLayout::VecR) {
            o.nodes.Rp = d_Obs_Rp;
            o.gate_nodes.Rp = d_Gate_Rp;
        } else {
            o.nodes.Rq = d_Obs_Rq;
            o.nodes.TD = d_Obs_TD;
            o.gate_nodes.Rq = d_Gate_Rq;
            o.gate_nodes.TD = d_Gate_TD;
        }
        if constexpr (L != NodeLayout::QuatTD && L != NodeLayout::QuatSAT) {
            o.nodes.T = d_T_obs;
            o.nodes.dim = d_Obs_dim;
            o.nodes.a = d_Obs_a;
            o.gate_nodes.T = d_Gate_T;
            o.gate_nodes.dim = d_Gate_dim;
            o.gate_nodes.a = d_Gate_a;
        }
        o.first_child = fc;
        o.num_nodes = obs_BVH.size;
        o.gate_count = gate_count;
        o.verts = d_Obs_verts;
        o.tris = d_Obs_tris;
        return o;
    };
    const auto makeRob = [&](auto layoutTag, const auto* fc) {
        constexpr NodeLayout L = decltype(layoutTag)::value;
        RobotSoA<std::decay_t<decltype(*fc)>, L> r;
        if constexpr (L == NodeLayout::Matrix) {
            r.nodes.R = d_Rob_R;
        } else if constexpr (L == NodeLayout::Quat) {
            r.nodes.Rq = d_Rob_Rq;
        } else if constexpr (L == NodeLayout::VecR) {
            r.nodes.Rp = d_Rob_Rp;
        } else {
            r.nodes.Rq = d_Rob_Rq;
            r.nodes.TD = d_Rob_TD;
        }
        if constexpr (L != NodeLayout::QuatTD && L != NodeLayout::QuatSAT) {
            r.nodes.T = d_Rob_T;
            r.nodes.dim = d_Rob_dim;
            r.nodes.a = d_Rob_a;
        }
        r.first_child = fc;
        r.linkOffset = d_LinkOffset;
        r.linkVertOffset = d_LinkVertOffset;
        r.linkTriOffset = d_LinkTriOffset;
        r.joints = d_Joints;
        r.jointsQ = d_JointsQ;
        r.verts = d_Rob_verts;
        r.tris = d_Rob_tris;
        return r;
    };

    auto launch = [&](int blocks, size_t launchConfs) {
        const auto doLaunch = [&](auto kernel, const auto& obsS, const auto& robS) {
            // Max out the shared-memory carveout so shared doesn't limit
            // occupancy (2 blocks/SM needs 2x26.7KB = 53.5KB shared/SM).
            cudaFuncSetAttribute(kernel, cudaFuncAttributePreferredSharedMemoryCarveout, 100);
            kernel<<<blocks, blockSize>>>(
                obsS, robS, d_Conf, launchConfs, kout);
        };
        const auto doLaunchForLayout = [&](auto layoutTag) {
            constexpr NodeLayout L = decltype(layoutTag)::value;
            const auto pickLaunch = [&](auto k1bsm, auto k2bsm, const auto& obsS, const auto& robS) {
                if (use2bsm) doLaunch(k2bsm, obsS, robS);
                else doLaunch(k1bsm, obsS, robS);
            };
            if (rob16) {
                if (obs16) {
                    const auto obsS = makeObs(layoutTag, d_Obs_first_child16);
                    const auto robS = makeRob(layoutTag, d_Rob_first_child16);
                    if (want_restrict) pickLaunch(d_bvh_articulated<N, int16_t, int16_t, true, L>, d_bvh_articulated_2bsm<N, int16_t, int16_t, true, L>, obsS, robS);
                    else               pickLaunch(d_bvh_articulated<N, int16_t, int16_t, false, L>, d_bvh_articulated_2bsm<N, int16_t, int16_t, false, L>, obsS, robS);
                } else {
                    const auto obsS = makeObs(layoutTag, d_Obs_first_child32);
                    const auto robS = makeRob(layoutTag, d_Rob_first_child16);
                    if (want_restrict) pickLaunch(d_bvh_articulated<N, int16_t, int32_t, true, L>, d_bvh_articulated_2bsm<N, int16_t, int32_t, true, L>, obsS, robS);
                    else               pickLaunch(d_bvh_articulated<N, int16_t, int32_t, false, L>, d_bvh_articulated_2bsm<N, int16_t, int32_t, false, L>, obsS, robS);
                }
            } else {
                if (obs16) {
                    const auto obsS = makeObs(layoutTag, d_Obs_first_child16);
                    const auto robS = makeRob(layoutTag, d_Rob_first_child32);
                    if (want_restrict) pickLaunch(d_bvh_articulated<N, int32_t, int16_t, true, L>, d_bvh_articulated_2bsm<N, int32_t, int16_t, true, L>, obsS, robS);
                    else               pickLaunch(d_bvh_articulated<N, int32_t, int16_t, false, L>, d_bvh_articulated_2bsm<N, int32_t, int16_t, false, L>, obsS, robS);
                } else {
                    const auto obsS = makeObs(layoutTag, d_Obs_first_child32);
                    const auto robS = makeRob(layoutTag, d_Rob_first_child32);
                    if (want_restrict) pickLaunch(d_bvh_articulated<N, int32_t, int32_t, true, L>, d_bvh_articulated_2bsm<N, int32_t, int32_t, true, L>, obsS, robS);
                    else               pickLaunch(d_bvh_articulated<N, int32_t, int32_t, false, L>, d_bvh_articulated_2bsm<N, int32_t, int32_t, false, L>, obsS, robS);
                }
            }
        };
        switch (layout) {
            case NodeLayout::Matrix: doLaunchForLayout(std::integral_constant<NodeLayout, NodeLayout::Matrix>()); break;
            case NodeLayout::Quat:   doLaunchForLayout(std::integral_constant<NodeLayout, NodeLayout::Quat>()); break;
            case NodeLayout::VecR:   doLaunchForLayout(std::integral_constant<NodeLayout, NodeLayout::VecR>()); break;
            case NodeLayout::QuatSAT: doLaunchForLayout(std::integral_constant<NodeLayout, NodeLayout::QuatSAT>()); break;
            default:                 doLaunchForLayout(std::integral_constant<NodeLayout, NodeLayout::QuatTD>()); break;
        }
    };

    if (dry_run) {
        checkCudaMem(cudaMemset(d_next_conf, 0, sizeof(uint32_t)));
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
    checkCudaMem(cudaMemset(d_disjoint, 0, num_words * sizeof(uint32_t)));
    checkCudaMem(cudaMemset(d_overflow, 0, sizeof(unsigned long long)));
    cudaEventRecord(start, 0);
    launch(gridSize, num_confs);
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Articulated BVH GPU kernel took " << duration << " ms for "
              << num_confs << " configurations." << std::endl;

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

    if (layout == NodeLayout::Matrix) cudaFree(d_R_obs);
    if (needsSplitArrays) {
        cudaFree(d_T_obs);
        cudaFree(d_Obs_dim);
        cudaFree(d_Obs_a);
    }
    if (obs16) {
        cudaFree(d_Obs_first_child16);
    } else {
        cudaFree(d_Obs_first_child32);
    }
    cudaFree(d_Obs_verts);
    cudaFree(d_Obs_tris);
    if (layout == NodeLayout::Matrix) cudaFree(d_Rob_R);
    if (needsSplitArrays) {
        cudaFree(d_Rob_T);
        cudaFree(d_Rob_dim);
        cudaFree(d_Rob_a);
    }
    if (rob16) {
        cudaFree(d_Rob_first_child16);
    } else {
        cudaFree(d_Rob_first_child32);
    }
    cudaFree(d_Rob_verts);
    cudaFree(d_Rob_tris);
    if (d_Obs_Rq) cudaFree(d_Obs_Rq);
    if (d_Obs_Rp) cudaFree(d_Obs_Rp);
    if (d_Obs_TD) cudaFree(d_Obs_TD);
    if (d_Rob_Rq) cudaFree(d_Rob_Rq);
    if (d_Rob_Rp) cudaFree(d_Rob_Rp);
    if (d_Rob_TD) cudaFree(d_Rob_TD);
    cudaFree(d_LinkOffset);
    cudaFree(d_LinkVertOffset);
    cudaFree(d_LinkTriOffset);
    if (d_Joints) cudaFree(d_Joints);
    if (d_JointsQ) cudaFree(d_JointsQ);
    cudaFree(d_Conf);
    cudaFree(d_disjoint);
    cudaFree(d_next_conf);
    cudaFree(d_overflow);

    return duration;
}
