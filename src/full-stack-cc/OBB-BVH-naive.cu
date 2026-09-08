#include "OBB-BVH-naive.hu"

//TODO: Make a custom data type for this struct
BVNode_soa BVH_fcl_hierarchy_from_mesh(const char* mesh_path, size_t power_of_2){
    // Load Robot
    std::vector<fcl::Vector3f> rob_vertices;
    std::vector<fcl::Triangle> rob_triangles;

    loadOBJFileFCL(mesh_path, rob_vertices, rob_triangles);

    // why is this a pointer???
    std::shared_ptr<fcl::BVHModel<fcl::OBB<float>>> rob_mesh(new fcl::BVHModel<fcl::OBB<float>>);
    rob_mesh->beginModel(rob_triangles.size(), rob_vertices.size());
    rob_mesh->addSubModel(rob_vertices, rob_triangles);
    rob_mesh->endModel();

    // Access OBB data from rob_mesh
    // rob_mesh->getNumBVs() gives the number of OBBs in the hierarchy
    size_t num_boxes = rob_mesh->getNumBVs();


    std::vector<size_t> leaf_depths = getBVHTreeDepths(*rob_mesh);
    size_t max_depth = *std::max_element(leaf_depths.begin(), leaf_depths.end());
    size_t reduced_depth = (max_depth + power_of_2 - 1) / power_of_2; // ceiling division to get number of n-ary levels

    std::vector<fcl::OBB<float>> flattened_n_ary;
    std::vector<int16_t> first_children;

    fcl::BVNode<fcl::OBB<float>> focus_node = rob_mesh->getBV(0); // start at root
    size_t parent_depth = 1;
    size_t curr_max_depth = parent_depth + power_of_2; // each n-ary level corresponds to power_of_2 binary levels

    std::vector<std::pair<int, size_t>> curr_frontier;
    curr_frontier.push_back(std::make_pair(0, parent_depth)); // pair of (node index, depth)

    size_t num_dummies = 0;
    size_t num_total_leafs = 0;
    for (size_t i = 0 ; i < reduced_depth; ++i){
        // turn current frontier into a stack for processing
        std::queue<std::pair<int, size_t>> node_queue;

        // push the current frontier nodes into the flattened structure
        for (const auto& pair : curr_frontier){
            focus_node = rob_mesh->getBV(pair.first);
            flattened_n_ary.push_back(focus_node.bv);
        }

        // update the first_children structure
        ////////////////////////////////////////////////////////////////////////////////////////////////
        // prev_first_primitive is used to track primitive index of the previous node in the loop
        // if we see the same primitive index twice in a row, we know it's a dummy node
        
        //TODO: very hacky, do beter at determining which nodes are dummy nodes
        int prev_first_primitive = 80000;
        size_t num_parent_nodes = 0;

        for (const auto& pair : curr_frontier){
            focus_node = rob_mesh->getBV(pair.first);
            if (focus_node.isLeaf()){
                if (prev_first_primitive == focus_node.first_primitive){
                    // this is a dummy node, indicate as such
                    first_children.push_back(0);
                    num_dummies++;
                } else {
                    // real leaf node
                    first_children.push_back(focus_node.first_child); 
                    prev_first_primitive = focus_node.first_primitive;
                    num_total_leafs++;
                }
            } else {
                prev_first_primitive = 80000; // reset

                first_children.push_back(flattened_n_ary.size() + (num_parent_nodes << power_of_2)); // index of first child in flattened structure
                num_parent_nodes++;
                node_queue.push(pair);
            }
        }

        curr_frontier.clear();

        while (!node_queue.empty()){
            auto [node_idx, depth] = node_queue.front();
            node_queue.pop();
            focus_node = rob_mesh->getBV(node_idx);
            if (depth >= curr_max_depth){
                // this node becomes a great^n grandchild
                curr_frontier.push_back(std::make_pair(node_idx, depth));

            } else {
                // expand this node's children
                if (focus_node.isLeaf()){
                    int num_copies = 2;

                    //NOTE: this part seems very error prone, need to check
                    for (int i = 0; i < num_copies; ++i){
                        node_queue.push(std::make_pair(node_idx, depth + 1));
                    }
                } else {
                    int left  = focus_node.leftChild();
                    int right = focus_node.rightChild();
                    node_queue.push(std::make_pair(left, depth + 1));
                    node_queue.push(std::make_pair(right, depth + 1));
                }
            }
        }
        parent_depth = curr_max_depth;
        curr_max_depth += power_of_2;
    }

    // update after the last iteration
    // turn current frontier into a stack for processing
    // push the current frontier nodes into the flattened structure
    for (const auto& pair : curr_frontier){
        focus_node = rob_mesh->getBV(pair.first);
        flattened_n_ary.push_back(focus_node.bv);
    }

    // update the first_children structure
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // prev_first_primitive is used to track primitive index of the previous node in the loop
    // if we see the same primitive index twice in a row, we know it's a dummy node
    int prev_first_primitive = -1;

    for (const auto& pair : curr_frontier){
        //flush the completed fronter nodes into the flattened structure
        focus_node = rob_mesh->getBV(pair.first);
        if (focus_node.isLeaf()){
            if (prev_first_primitive == focus_node.first_primitive){
                first_children.push_back(0); // dummy node
                num_dummies++;
            } else {
                // real leaf node
                first_children.push_back(focus_node.first_child); // indicate leaf with negative numbers
                prev_first_primitive = focus_node.first_primitive;
                num_total_leafs++;
            }
        } else {
            std::cout << "Uh oh, shouldn't have parents anymore :(" <<std::endl;
            prev_first_primitive = -1; // reset

        }
    }

    Eigen::Matrix3f rotation;
    Eigen::Vector3f translation;
    Eigen::Vector3f half_dimensions;
    BVNode_soa result(flattened_n_ary.size());
    for (int i = 0; i < result.size; ++i) {
        fcl::OBB<float> obb = flattened_n_ary[i];
        rotation = obb.axis;
        translation = obb.To;
        half_dimensions = obb.extent;

        result.set(i, rotation, translation, half_dimensions, first_children[i]);
    }

    // delete rob_mesh manually to free memory
    rob_mesh.reset();

    return result;
}

// ===========================================================================
// Hand-rolled OBB BVH builder.
//
// Leaves are the minimum bounding rectangles of single triangles in the style
// of Chang & Kim 2009 ("Efficient triangle-triangle intersection test for
// OBB-based collision detection"): the rectangle shares the triangle's longest
// edge, the rectangle plane is the triangle plane, and the triangle's third
// vertex sits at local coordinate (a, dy, 0) with the shared edge at y = -dy.
// That makes the paper's cheap triangle test applicable later, with the
// relative OBB transform from the traversal reused instead of recomputed.
//
// Internal nodes are tight OBBs fitted by PCA over their children's corners
// (conservative by construction: extents are the min/max projections of all
// corner points). The hierarchy is a balanced binary tree (median split on
// the longest axis of the triangle centroid AABB); every two binary levels
// become one n-ary level, so each internal n-ary node has exactly 4 child
// slots and leaves only sit at the deepest level (a shallow binary leaf
// expands into [leaf, dummy], matching the kernel's traversal).
//
// Node encoding matches fcl::BVNodeBase and the kernel's expectations:
//   first_child > 0 : index of the first child (children consecutive)
//   first_child < 0 : -(triangle index + 1)
//   first_child = 0 : dummy (padding)
// ===========================================================================
namespace {

struct LeafRect {
    Eigen::Matrix3f R;
    Eigen::Vector3f T;
    Eigen::Vector3f dim;
    float a;
    Eigen::Vector3f centroid;
    Eigen::Vector3f p1, p2, p3; // world-space triangle vertices
};

// Chang & Kim minimum rectangle for one triangle. The longest edge is chosen
// as the shared edge (valid for any triangle; minimum-area for obtuse ones).
LeafRect makeLeafRect(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3) {
    Eigen::Vector3f va = p1, vb = p2, vc = p3;
    const float l12 = (p2 - p1).squaredNorm();
    const float l23 = (p3 - p2).squaredNorm();
    const float l31 = (p1 - p3).squaredNorm();
    if (l23 > l12 && l23 >= l31) {
        va = p2; vb = p3; vc = p1;
    } else if (l31 > l12) {
        va = p3; vb = p1; vc = p2;
    }

    const Eigen::Vector3f edge = vb - va;
    const float edge_len = edge.norm();
    const Eigen::Vector3f rx = edge / edge_len;

    Eigen::Vector3f rz = (vb - va).cross(vc - va);
    const float nlen = rz.norm();
    if (nlen > 1e-12f) {
        rz /= nlen;
    } else {
        // Degenerate (zero-area) triangle: build an arbitrary right-handed
        // frame from rx so the pipeline still gets a valid (flat) rectangle.
        Eigen::Vector3f aux = (fabsf(rx(2)) < 0.9f) ? Eigen::Vector3f(0, 0, 1) : Eigen::Vector3f(1, 0, 0);
        rz = rx.cross(aux).normalized();
    }
    Eigen::Vector3f ry = rz.cross(rx);

    const Eigen::Vector3f center = (va + vb) * 0.5f;
    float h = (vc - center).dot(ry);
    if (h < 0.0f) {
        ry = -ry;
        h = -h;
    }

    LeafRect r;
    r.R.col(0) = rx;
    r.R.col(1) = ry;
    r.R.col(2) = rz;
    r.T = center;
    r.dim = Eigen::Vector3f(edge_len * 0.5f, h, 0.0f);
    r.a = (vc - center).dot(rx);
    r.centroid = (p1 + p2 + p3) / 3.0f;
    r.p1 = p1;
    r.p2 = p2;
    r.p3 = p3;
    return r;
}

struct BinNode {
    Eigen::Matrix3f R;
    Eigen::Vector3f T;
    Eigen::Vector3f dim;
    float a = 0.0f;
    int left = -1;
    int right = -1;
    int tri = -1;
};

int buildBinary(const std::vector<LeafRect>& rects, std::vector<int>& order,
                int begin, int end, std::vector<BinNode>& nodes) {
    const int idx = (int)nodes.size();
    nodes.emplace_back();
    BinNode& n = nodes[idx];

    if (end - begin == 1) {
        const LeafRect& r = rects[order[begin]];
        n.R = r.R;
        n.T = r.T;
        n.dim = r.dim;
        n.a = r.a;
        n.tri = order[begin];
        return idx;
    }

    // Binned SAH split along the fitted OBB's longest local axis: the OBB is
    // fitted first (same vertex-PCA scheme as FCL), triangles are binned by
    // their centroids projected onto that axis, and the split minimizes
    // SA(left bins) * nL + SA(right bins) * nR with per-bin world AABBs.
    // Falls back to a median split for tiny or degenerate nodes.
    const int count = end - begin;

    // Fit the internal OBB from the subtree's triangle vertices (same scheme
    // as FCL's eigen fit): tighter than corner-PCA of the two child boxes.
    const int npts = count * 3;
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (int i = begin; i < end; ++i) {
        const LeafRect& r = rects[order[i]];
        mean += r.p1 + r.p2 + r.p3;
    }
    mean /= (float)npts;

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (int i = begin; i < end; ++i) {
        const LeafRect& r = rects[order[i]];
        const Eigen::Vector3f* v[3] = {&r.p1, &r.p2, &r.p3};
        for (int k = 0; k < 3; ++k) {
            const Eigen::Vector3f d = *v[k] - mean;
            for (int rr = 0; rr < 3; ++rr)
                for (int cc = 0; cc < 3; ++cc)
                    cov(rr, cc) += d(rr) * d(cc);
        }
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
    n.R = es.eigenvectors();
    if (n.R.determinant() < 0.0f) n.R.col(2) *= -1.0f;

    Eigen::Vector3f plo(FLT_MAX, FLT_MAX, FLT_MAX), phi(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    for (int i = begin; i < end; ++i) {
        const LeafRect& r = rects[order[i]];
        const Eigen::Vector3f* v[3] = {&r.p1, &r.p2, &r.p3};
        for (int k = 0; k < 3; ++k) {
            const Eigen::Vector3f local = n.R.transpose() * (*v[k] - mean);
            plo = plo.cwiseMin(local);
            phi = phi.cwiseMax(local);
        }
    }
    n.dim = (phi - plo) * 0.5f;
    n.T = mean + n.R * ((plo + phi) * 0.5f);

    // Split axis: the OBB local axis with the largest extent.
    int axis = 0;
    if (n.dim(1) > n.dim(axis)) axis = 1;
    if (n.dim(2) > n.dim(axis)) axis = 2;
    const Eigen::Vector3f split_dir = n.R.col(axis);

    std::sort(order.begin() + begin, order.begin() + end, [&](int x, int y) {
        return rects[x].centroid.dot(split_dir) < rects[y].centroid.dot(split_dir);
    });

    int mid = (begin + end) / 2;
    if (count > 8) {
        float lo = FLT_MAX, hi = -FLT_MAX;
        for (int i = begin; i < end; ++i) {
            const float p = rects[order[i]].centroid.dot(split_dir);
            lo = fminf(lo, p);
            hi = fmaxf(hi, p);
        }
        const float span = hi - lo;

        if (span > 0.0f) {
            constexpr int NBINS = 12;
            struct Bin {
                int cnt = 0;
                Eigen::Vector3f lo{FLT_MAX, FLT_MAX, FLT_MAX}, hi{-FLT_MAX, -FLT_MAX, -FLT_MAX};
            };
            Bin bins[NBINS];
            const float inv_span = (float)NBINS / span;
            for (int i = begin; i < end; ++i) {
                const LeafRect& r = rects[order[i]];
                int b = (int)((r.centroid.dot(split_dir) - lo) * inv_span);
                if (b < 0) b = 0;
                if (b >= NBINS) b = NBINS - 1;
                bins[b].cnt++;
                bins[b].lo = bins[b].lo.cwiseMin(r.p1).cwiseMin(r.p2).cwiseMin(r.p3);
                bins[b].hi = bins[b].hi.cwiseMax(r.p1).cwiseMax(r.p2).cwiseMax(r.p3);
            }

            float best_cost = FLT_MAX;
            int best_split = -1;
            for (int s = 0; s < NBINS - 1; ++s) {
                Eigen::Vector3f llo(FLT_MAX, FLT_MAX, FLT_MAX), lhi(-FLT_MAX, -FLT_MAX, -FLT_MAX);
                Eigen::Vector3f rlo(FLT_MAX, FLT_MAX, FLT_MAX), rhi(-FLT_MAX, -FLT_MAX, -FLT_MAX);
                int nl = 0, nr = 0;
                for (int b = 0; b <= s; ++b) {
                    if (!bins[b].cnt) continue;
                    llo = llo.cwiseMin(bins[b].lo); lhi = lhi.cwiseMax(bins[b].hi); nl += bins[b].cnt;
                }
                for (int b = s + 1; b < NBINS; ++b) {
                    if (!bins[b].cnt) continue;
                    rlo = rlo.cwiseMin(bins[b].lo); rhi = rhi.cwiseMax(bins[b].hi); nr += bins[b].cnt;
                }
                if (nl == 0 || nr == 0) continue;
                const Eigen::Vector3f le = lhi - llo, re = rhi - rlo;
                const float saL = 2.0f * (le(0) * le(1) + le(1) * le(2) + le(2) * le(0));
                const float saR = 2.0f * (re(0) * re(1) + re(1) * re(2) + re(2) * re(0));
                const float cost = saL * (float)nl + saR * (float)nr;
                if (cost < best_cost) {
                    best_cost = cost;
                    best_split = s;
                }
            }

            if (best_split >= 0) {
                const float split_val = lo + span * (float)(best_split + 1) / (float)NBINS;
                mid = begin;
                while (mid < end && rects[order[mid]].centroid.dot(split_dir) < split_val) ++mid;
                if (mid <= begin) mid = begin + 1;
                if (mid >= end) mid = end - 1;
            }
        }
    }

    n.left = buildBinary(rects, order, begin, mid, nodes);
    n.right = buildBinary(rects, order, mid, end, nodes);
    return idx;
}

struct NarySlot {
    Eigen::Matrix3f R;
    Eigen::Vector3f T;
    Eigen::Vector3f dim;
    float a = 0.0f;
    int16_t first_child = 0;
    bool internal = false;
};

} // namespace

BVNode_soa BVH_n_ary_hierarchy_from_mesh(const char* mesh_path, size_t power_of_2) {
    if (power_of_2 != 2) {
        std::cerr << "BVH_n_ary_hierarchy_from_mesh: only power_of_2 == 2 (4-ary) is supported by the kernel" << std::endl;
        exit(1);
    }

    std::vector<Eigen::Vector3f> vertices;
    std::vector<Triangle> triangles;
    loadOBJFile(mesh_path, vertices, triangles);
    const int num_tris = (int)triangles.size();
    if (num_tris == 0) {
        return BVNode_soa(0);
    }

    // One paper rectangle per triangle, in mesh triangle order (triangle
    // indices must match the kernel's mesh arrays).
    std::vector<LeafRect> rects(num_tris);
    for (int t = 0; t < num_tris; ++t) {
        rects[t] = makeLeafRect(vertices[triangles[t].v1],
                                vertices[triangles[t].v2],
                                vertices[triangles[t].v3]);
    }

    // Balanced binary tree (median split, OBBs fitted bottom-up).
    std::vector<int> order(num_tris);
    for (int i = 0; i < num_tris; ++i) order[i] = i;
    std::vector<BinNode> bnodes;
    bnodes.reserve(2 * num_tris);
    const int broot = buildBinary(rects, order, 0, num_tris, bnodes);

    // Flatten to 4-ary: n-ary level k = binary depth 2k. Each internal binary
    // node at level k contributes 4 slots: for each binary child c, either
    // [c.left slot, c.right slot] (c internal) or [leaf slot, dummy] (c leaf).
    std::vector<std::vector<NarySlot>> levels(1);
    std::vector<std::vector<int>> bin_of(1); // binary node index of each slot

    NarySlot root;
    root.R = bnodes[broot].R;
    root.T = bnodes[broot].T;
    root.dim = bnodes[broot].dim;
    root.a = bnodes[broot].a;
    if (bnodes[broot].tri >= 0) {
        root.first_child = (int16_t)(-(bnodes[broot].tri + 1));
        root.internal = false;
    } else {
        root.internal = true;
    }
    levels[0].push_back(root);
    bin_of[0].push_back(broot);

    for (size_t k = 0; k < levels.size(); ++k) {
        bool any_internal = false;
        for (const NarySlot& s : levels[k]) {
            if (s.internal) { any_internal = true; break; }
        }
        if (!any_internal) break;

        std::vector<NarySlot>& next = levels.emplace_back();
        std::vector<int>& next_bin = bin_of.emplace_back();
        int internal_count = 0;
        for (NarySlot& s : levels[k]) {
            if (!s.internal) continue;
            s.first_child = (int16_t)(next.size() + internal_count * 4);
            ++internal_count;
        }
        for (size_t i = 0; i < levels[k].size(); ++i) {
            const NarySlot& s = levels[k][i];
            if (!s.internal) continue;
            const BinNode& b = bnodes[bin_of[k][i]];
            const int bchildren[2] = {b.left, b.right};
            for (int c = 0; c < 2; ++c) {
                const int16_t bc = bchildren[c];
                if (bc < 0) {
                    next.emplace_back();
                    next_bin.push_back(-1);
                    next.emplace_back();
                    next_bin.push_back(-1);
                    continue;
                }
                const BinNode& child = bnodes[bc];
                if (child.tri >= 0) {
                    NarySlot leaf;
                    leaf.R = child.R;
                    leaf.T = child.T;
                    leaf.dim = child.dim;
                    leaf.a = child.a;
                    leaf.first_child = (int16_t)(-(child.tri + 1));
                    leaf.internal = false;
                    next.push_back(leaf);
                    next_bin.push_back(bc);
                    next.emplace_back();
                    next_bin.push_back(-1);
                } else {
                    const int gc[2] = {child.left, child.right};
                    for (int g = 0; g < 2; ++g) {
                        const BinNode& gchild = bnodes[gc[g]];
                        NarySlot slot;
                        slot.R = gchild.R;
                        slot.T = gchild.T;
                        slot.dim = gchild.dim;
                        slot.a = gchild.a;
                        if (gchild.tri >= 0) {
                            slot.first_child = (int16_t)(-(gchild.tri + 1));
                            slot.internal = false;
                        } else {
                            slot.internal = true;
                        }
                        next.push_back(slot);
                        next_bin.push_back(gc[g]);
                    }
                }
            }
        }
    }

    // Serialize level order into the SoA layout. Internal nodes reindex their
    // level-local first_child (which may be 0; 0 is the kernel's dummy
    // sentinel, so it cannot be used as an internal pointer); leaves (< 0)
    // and dummies (0) pass through unchanged.
    size_t total = 0;
    for (const auto& lv : levels) total += lv.size();
    if (total > 32767) {
        std::cerr << "BVH_n_ary_hierarchy_from_mesh: " << total
                  << " nodes exceeds int16_t first_child capacity (32767)" << std::endl;
        exit(1);
    }

    BVNode_soa result(total);
    std::vector<size_t> level_base(levels.size());
    size_t out = 0;
    for (size_t k = 0; k < levels.size(); ++k) {
        level_base[k] = out;
        out += levels[k].size();
    }
    for (size_t k = 0; k < levels.size(); ++k) {
        for (size_t i = 0; i < levels[k].size(); ++i) {
            const NarySlot& n = levels[k][i];
            int16_t fc = n.internal ? (int16_t)(n.first_child + level_base[k + 1]) : n.first_child;
            result.set(level_base[k] + i, n.R, n.T, n.dim, fc, n.a);
        }
    }

    // Validate the flattened tree structure (host-side, cheap).
    for (size_t i = 0; i < total; ++i) {
        const int16_t fc = result.first_child[i];
        if (fc > 0) {
            if (fc + 4 > (int16_t)total) {
                std::cerr << "BVH validate: node " << i << " first_child " << fc
                          << " + 4 exceeds " << total << std::endl;
                exit(1);
            }
        } else if (fc < 0) {
            const int tri = -fc - 1;
            if (tri < 0 || tri >= num_tris) {
                std::cerr << "BVH validate: node " << i << " leaf tri " << tri << " out of range" << std::endl;
                exit(1);
            }
        }
    }

    return result;
}


constexpr int BLOCK_SIZE = 32;

// Nanosecond wall-clock timer (%globaltimer), immune to SM clock throttling.
__device__ __forceinline__ unsigned long long globaltimer() {
    unsigned long long t;
    asm volatile("mov.u64 %0, %%globaltimer;" : "=l"(t));
    return t;
}

// Chang & Kim 2009 triangle-triangle test in rectangle-local coordinates.
// Rect 1 = obstacle leaf (its z-axis is the triangle plane normal), rect 2 =
// robot leaf at relative rotation B and translation T, both taken straight
// from the OBB overlap test computed just before this leaf pair was found.
//
// Returns  1: triangles intersect
//          0: disjoint
//         -1: unreliable (near-plane/tangent/coplanar): caller must fall back
//             to the full world-frame test.
__device__ __forceinline__ int paperTriTri(
    const float dx1, const float dy1, const float a1,
    const float dx2, const float dy2, const float a2,
    const Eigen::Matrix3f& B, const Eigen::Vector3f& T) {

    const float rz0 = B(0, 2), rz1 = B(1, 2), rz2 = B(2, 2);

    // Signed distances of the obs (rect 1) vertices from the rob plane.
    // Vertex layout in the rectangle frame (center at the shared edge
    // midpoint): p1 = (-dx1, 0, 0), p2 = (dx1, 0, 0), p3 = (a1, dy1, 0).
    const float d = T(0) * rz0 + T(1) * rz1 + T(2) * rz2;
    const float dp1 = -dx1 * rz0 - d;
    const float dp2 =  dx1 * rz0 - d;
    const float dp3 =  a1  * rz0 + dy1 * rz1 - d;

    // Signed distances of the rob (rect 2) vertices from the obs plane
    // (z = 0 in the obs rectangle frame); same vertex layout for rect 2.
    // These are the z-components of the rob vertices in obs coordinates:
    // B(2,0) and B(2,1) are the z-components of the rob rectangle's rx/ry.
    const float dq1 = T(2) - dx2 * B(2, 0);
    const float dq2 = T(2) + dx2 * B(2, 0);
    const float dq3 = T(2) + a2  * B(2, 0) + dy2 * B(2, 1);

    // Near-plane (and coplanar) configurations are delegated to the full test.
    const float m1 = fmaxf(fmaxf(fabsf(dp1), fabsf(dp2)), fabsf(dp3));
    const float mn1 = fminf(fminf(fabsf(dp1), fabsf(dp2)), fabsf(dp3));
    if (m1 == 0.0f || mn1 < 1e-5f * m1) return -1;

    const float m2 = fmaxf(fmaxf(fabsf(dq1), fabsf(dq2)), fabsf(dq3));
    const float mn2 = fminf(fminf(fabsf(dq1), fabsf(dq2)), fabsf(dq3));
    if (m2 == 0.0f || mn2 < 1e-5f * m2) return -1;

    // Reject when either triangle lies entirely on one side of the other's
    // plane.
    if (dp1 > 0.0f && dp2 > 0.0f && dp3 > 0.0f) return 0;
    if (dp1 < 0.0f && dp2 < 0.0f && dp3 < 0.0f) return 0;
    if (dq1 > 0.0f && dq2 > 0.0f && dq3 > 0.0f) return 0;
    if (dq1 < 0.0f && dq2 < 0.0f && dq3 < 0.0f) return 0;

    // Both triangles cross both planes: compare the two intersection segments
    // on the common line L = P x Q. The direction of L is z x rz =
    // (-rz1, rz0, 0), so project onto the axis with the larger component.
    const bool useX = fabsf(rz1) > fabsf(rz0);

    float px1, px2, px3; // obs vertex coordinates on the projection axis
    float qx1, qx2, qx3; // rob vertex coordinates on the projection axis
    if (useX) {
        px1 = -dx1; px2 = dx1; px3 = a1;
        qx1 = T(0) - dx2 * B(0, 0);
        qx2 = T(0) + dx2 * B(0, 0);
        qx3 = T(0) + a2  * B(0, 0) + dy2 * B(0, 1);
    } else {
        px1 = 0.0f; px2 = 0.0f; px3 = dy1;
        qx1 = T(1) - dx2 * B(1, 0);
        qx2 = T(1) + dx2 * B(1, 0);
        qx3 = T(1) + a2  * B(1, 0) + dy2 * B(1, 1);
    }

    // For each triangle, the vertex whose signed distance has the unique sign
    // is the isolated vertex; the segment endpoints lie on its two incident
    // edges. Endpoint = (d_i x_j - d_j x_i) / (d_i - d_j). The denominators
    // are nonzero here (opposite signs, |d| >= mn > 0), and d1*d2 > 0,
    // d3*d4 > 0, so all four endpoints can be scaled by the common positive
    // factor d1*d2*d3*d4 to remove every division.
    const float dp[3] = {dp1, dp2, dp3};
    const float dq[3] = {dq1, dq2, dq3};
    const float px[3] = {px1, px2, px3};
    const float qx[3] = {qx1, qx2, qx3};

    int pIso;
    if ((dp[0] > 0.0f) == (dp[1] > 0.0f)) pIso = 2;
    else if ((dp[0] > 0.0f) == (dp[2] > 0.0f)) pIso = 1;
    else pIso = 0;
    int qIso;
    if ((dq[0] > 0.0f) == (dq[1] > 0.0f)) qIso = 2;
    else if ((dq[0] > 0.0f) == (dq[2] > 0.0f)) qIso = 1;
    else qIso = 0;

    const int pO1 = (pIso + 1) % 3, pO2 = (pIso + 2) % 3;
    const int qO1 = (qIso + 1) % 3, qO2 = (qIso + 2) % 3;

    const float d1 = dp[pIso] - dp[pO1];
    const float d2 = dp[pIso] - dp[pO2];
    const float d3 = dq[qIso] - dq[qO1];
    const float d4 = dq[qIso] - dq[qO2];

    const float n1 = dp[pIso] * px[pO1] - dp[pO1] * px[pIso];
    const float n2 = dp[pIso] * px[pO2] - dp[pO2] * px[pIso];
    const float n3 = dq[qIso] * qx[qO1] - dq[qO1] * qx[qIso];
    const float n4 = dq[qIso] * qx[qO2] - dq[qO2] * qx[qIso];

    const float e1 = n1 * d2 * d3 * d4;
    const float e2 = n2 * d1 * d3 * d4;
    const float e3 = n3 * d1 * d2 * d4;
    const float e4 = n4 * d1 * d2 * d3;

    const float lo1 = fminf(e1, e2), hi1 = fmaxf(e1, e2);
    const float lo2 = fminf(e3, e4), hi2 = fmaxf(e3, e4);

    const float gap  = fminf(hi1, hi2) - fmaxf(lo1, lo2);
    const float span = fmaxf(hi1 - lo1, hi2 - lo2);
    if (fabsf(gap) < 1e-5f * span + 1e-30f) return -1; // tangency: defer
    return (gap >= 0.0f) ? 1 : 0;
}

__global__ void d_bvh_naive   ( const Eigen::Matrix3f* __restrict__ pR_obs, const Eigen::Vector3f* __restrict__ pT_obs,
                                const Eigen::Matrix3f* __restrict__ pR_rob, const Eigen::Vector3f* __restrict__ pT_rob,
                                const Eigen::Vector3f* __restrict__ pObs_dim, const Eigen::Vector3f* __restrict__ pRob_dim,
                                const Eigen::Matrix3f* __restrict__ pRob_conf_rot, const Eigen::Vector3f* __restrict__ pRob_conf_trans,
                                const int16_t* __restrict__ pObs_first_child, const int16_t* __restrict__ pRob_first_child,
                                const float* __restrict__ pObs_a, const float* __restrict__ pRob_a,
                                const Eigen::Vector3f * __restrict__ pRob_verts, const Triangle * __restrict__ pRob_tris, size_t num_rob_nodes,
                                const Eigen::Vector3f * __restrict__ pObs_verts, const Triangle * __restrict__ pObs_tris, size_t num_obs_nodes,
                                uint32_t* __restrict__ pdisjoint, size_t num_confs, uint32_t* __restrict__ g_next_conf,
                                unsigned long long* __restrict__ d_phase) {

    if (num_obs_nodes == 0) {
        printf("Error: num_obs_nodes is zero. Exiting kernel.\n");
        return;
    }

    extern __shared__ char shared_mem[];

    size_t smem_offset = 0;
    int16_t* sObs_first_child = reinterpret_cast<int16_t*>(shared_mem + smem_offset);
    smem_offset += num_obs_nodes * sizeof(int16_t);
    int16_t* sRob_first_child = reinterpret_cast<int16_t*>(shared_mem + smem_offset);

    // Load the first_child arrays into shared memory once per block; they are
    // read-only for the lifetime of the kernel.
    for (uint16_t i = threadIdx.x; i < num_rob_nodes; i += blockDim.x){
        sRob_first_child[i] = pRob_first_child[i];
    }
    for (uint16_t i = threadIdx.x; i < num_obs_nodes; i += blockDim.x){
        sObs_first_child[i] = pObs_first_child[i];
    }
    __syncthreads();

    Eigen::Matrix3f R_obs_abs_root = pR_obs[0]; // rotation of B wrt origin
    Eigen::Vector3f T_obs_abs_root = pT_obs[0]; // translation of B wrt origin

    Eigen::Matrix3f R_rob_abs_root = pR_rob[0]; // rotation of A wrt origin
    Eigen::Vector3f T_rob_abs_root = pT_rob[0]; // translation of A wrt origin

    Eigen::Vector3f b_root = pRob_dim[0]; // half dimensions of box A
    Eigen::Vector3f a_root = pObs_dim[0]; // half dimensions of box B

    const float epsilon = 1e-6f; // small value to avoid numerical issues

    Eigen::Matrix3f R_conf; // rotation of robot wrt world
    Eigen::Vector3f T_conf; // translation of robot wrt world
    Eigen::Matrix3f R_obs_abs; // rotation of B wrt origin (per node)
    Eigen::Vector3f T_obs_abs;
    Eigen::Matrix3f R_rob_abs;
    Eigen::Vector3f T_rob_abs;
    Eigen::Vector3f b; // half dimensions of box A (per node)
    Eigen::Vector3f a; // half dimensions of box B (per node)
    Eigen::Matrix3f B;      // rotation of A wrt B
    Eigen::Matrix3f Bf;     // absolute value of B (plus epsilon)
    Eigen::Vector3f T;      // translation of A wrt B

    // intent: for each i in rob_obb_pend, need to check all children of rob_obb_pend[i] against all children of obs_obb_pend[j]
    // maybe should do 32 * num layers, back of envelope says that should be an upper limit
    constexpr int MAX_BUFFER = BLOCK_SIZE * 128;
    __shared__ uint16_t rob_obb_pend[MAX_BUFFER]; // arbitrary buffer size, should experiment with this
    __shared__ uint16_t obs_obb_pend[MAX_BUFFER];
    __shared__ int num_obb_pend;

    // Set (from any thread, all writers set true) when the paper triangle
    // test (or its fallback) finds an intersecting leaf pair for the current
    // configuration; the block-serial traversal breaks early on it.
    __shared__ bool s_collision;

    //TODO: need failsafe if this overflows

    constexpr int BATCH = BLOCK_SIZE; // configs pulled per queue transaction (one per thread)
    __shared__ uint32_t s_batch_start;
    __shared__ uint32_t s_num_pend;
    __shared__ Eigen::Matrix3f s_pend_rot[BATCH];
    __shared__ Eigen::Vector3f s_pend_trans[BATCH];
    __shared__ uint32_t s_pend_idx[BATCH];

    // Bitpacked result word for this batch: one bit per config. The batch is
    // always 32-aligned (BATCH == BLOCK_SIZE), so an entire batch maps to a
    // single uint32 word owned exclusively by this block; thread 0 flushes it.
    __shared__ uint32_t s_disjoint_word;

    int16_t conf_offset = (threadIdx.x >> 4)-2; // divide by 16 to see if thread works on the 0th pair or 1st pair of pending boxes
    int16_t rob_child_idx = (threadIdx.x >> 2) & 0x3; // divide by 4, then mod by 4to see which child of the robot box this thread is assigned to
    int16_t obs_child_idx = threadIdx.x & 0x3; // mod 4 to see which child of the obstacle box this thread is assigned to

    // Persistent-block work queue: each block pulls a batch of configuration
    // indices from a global atomic counter until the queue is exhausted. The
    // batch's outermost OBB check runs in parallel (one config per thread),
    // then the block traverses the pending configs serially. Dynamic pulls
    // balance load across blocks instead of static contiguous chunking.
    //
    // Phase timers (profiling): thread 0 accumulates %globaltimer deltas for the
    // block-serial phases into d_phase[0..2] (pull+initial check, OBB
    // traversal, triangle tests). Each block adds its own totals once at
    // termination, so these are SUMS ACROSS BLOCKS (not wall-clock); the host
    // divides by the contributing block count in d_phase[3].
    unsigned long long acc_init = 0, acc_trav = 0, acc_tri = 0;
    while (true) {
        __syncthreads();
        const unsigned long long t_phase0 = (threadIdx.x == 0) ? globaltimer() : 0;
        if (threadIdx.x == 0) {
            s_batch_start = atomicAdd(g_next_conf, BATCH);
            s_num_pend = 0;
            s_disjoint_word = 0;
        }
        __syncthreads();
        const uint32_t batch_start = s_batch_start;
        if (batch_start >= num_confs) {
            if (threadIdx.x == 0) {
                atomicAdd(&d_phase[0], acc_init);
                atomicAdd(&d_phase[1], acc_trav);
                atomicAdd(&d_phase[2], acc_tri);
                atomicAdd(&d_phase[3], 1);
            }
            return;
        }

        // parallel outermost OBB check: one config per thread
        const uint32_t index = batch_start + threadIdx.x;
        if (index < num_confs) {
            R_conf = pRob_conf_rot[index];
            T_conf = pRob_conf_trans[index];

            //Calculate relative rotation of B wrt A
            //TODO: precompute inverse rotations of A
            computeRelTransform(R_obs_abs_root, T_obs_abs_root,
                                R_rob_abs_root, T_rob_abs_root,
                                R_conf, T_conf, epsilon, B, Bf, T);

            //initial per conf outermost bounding box check
            if (!obbOverlap(a_root, b_root, B, Bf, T)) {
                atomicOr(&s_disjoint_word, 1u << threadIdx.x);
            } else {
                uint32_t pos = atomicAdd(&s_num_pend, 1);
                s_pend_rot[pos] = R_conf;
                s_pend_trans[pos] = T_conf;
                s_pend_idx[pos] = index;
            }
        } // end outermost OBB check
        __syncthreads();

        // Flush the bitpacked results of the outermost check before any early
        // continue: bits are only ever set, so re-ORing below is idempotent.
        if (threadIdx.x == 0) {
            pdisjoint[batch_start >> 5] |= s_disjoint_word;
        }

        // profiling
        if (threadIdx.x == 0) {
            acc_init += globaltimer() - t_phase0;
        }

        if (s_num_pend == 0) {
            continue;
        }

        for (uint32_t i = 0; i < s_num_pend; i++) {
            __syncthreads();
            const unsigned long long t_cfg = (threadIdx.x == 0) ? globaltimer() : 0;
            R_conf = s_pend_rot[i];
            T_conf = s_pend_trans[i];
            const uint32_t index = s_pend_idx[i];

            //reset pending OBB lists
            if (threadIdx.x == 0){
                num_obb_pend = 1;
                s_collision = false;
                rob_obb_pend[0] = 0; // root
                obs_obb_pend[0] = 0; // root
            }

            __syncthreads();

            // intent: for each i in rob_obb_pend, need to check all children of rob_obb_pend[i] against all children of obs_obb_pend[j]
            // these should be 4-ary trees, meaning we grab up to two pending boxes at a time so that we have 16 threads doing the children 
            // of each pair of boxes. 
            while(true){
                __syncthreads();
                if (s_collision) {
                    break;
                }
                if (num_obb_pend == 0){
                    break;
                }

                int pend_idx = num_obb_pend + conf_offset;
                if (num_obb_pend >= MAX_BUFFER - 32){
                    if (threadIdx.x == 0) {
                        printf("obb overflow with %d\n boxes on configuration with rotation matrix \n\r \
                                %f, %f, %f, \n %f, %f, %f, \n %f, %f, %f, \n and translation \n \n with block index %d\n \
                                %f, %f, %f \n", 
                                num_obb_pend, R_conf(0, 0), R_conf(0, 1), R_conf(0, 2),
                                R_conf(1, 0), R_conf(1, 1), R_conf(1, 2),
                                R_conf(2, 0), R_conf(2, 1), R_conf(2, 2),
                                T_conf[0], T_conf[1], T_conf[2], blockIdx.x);
                    }
                    break;
                }

                __syncthreads();

                if(threadIdx.x == 0){
                    if (num_obb_pend > 1) {num_obb_pend -= 2;} 
                    else {num_obb_pend = 0;}
                }

                __syncthreads();
                if (pend_idx < 0) {
                    continue;
                }

                unsigned mask = __activemask();
                
                int rob_obb_par_idx = rob_obb_pend[pend_idx];
                int obs_obb_par_idx = obs_obb_pend[pend_idx];
                
                int rob_obb_idx = sRob_first_child[rob_obb_par_idx] + rob_child_idx;
                int obs_obb_idx = sObs_first_child[obs_obb_par_idx] + obs_child_idx;

                int rob_first_child_idx = sRob_first_child[rob_obb_idx];
                int obs_first_child_idx = sObs_first_child[obs_obb_idx];

                __syncwarp(mask);

                // if either is a dummy node, skip check
                //TODO: see if this is safe to do
                if (rob_first_child_idx == 0 || obs_first_child_idx == 0){
                    continue;
                }

                R_obs_abs = pR_obs[obs_obb_idx];
                T_obs_abs = pT_obs[obs_obb_idx];
                R_rob_abs = pR_rob[rob_obb_idx];
                T_rob_abs = pT_rob[rob_obb_idx];
                b = pRob_dim[rob_obb_idx];
                a = pObs_dim[obs_obb_idx];

                computeRelTransform(R_obs_abs, T_obs_abs, R_rob_abs, T_rob_abs, R_conf, T_conf, epsilon, B, Bf, T);

                if (!obbOverlap(a, b, B, Bf, T)) {
                    continue;
                }

                if (rob_first_child_idx > 0 && obs_first_child_idx > 0) {
                    // add children to pending lists
                    int pos = atomicAdd(&num_obb_pend, 1);
                    obs_obb_pend[pos] = obs_obb_idx;
                    rob_obb_pend[pos] = rob_obb_idx;
                } 

                //TODO: when one box is a leaf and the other is not, the leaf's
                // parent is pushed back into the pending list, so the same
                // parent can be pushed multiple times for the same non-leaf
                // box (duplicate work, but harmless for correctness).
                else if (rob_first_child_idx < 0) {
                    // both are leaves
                    if (obs_first_child_idx < 0) {
                        // Chang & Kim triangle test in rectangle-local
                        // coordinates, reusing this thread's B/T from the OBB
                        // overlap test above.
                        const int rob_tri = -(rob_first_child_idx + 1);
                        const int obs_tri = -(obs_first_child_idx + 1);
                        const Eigen::Vector3f& dimObs = pObs_dim[obs_obb_idx];
                        const Eigen::Vector3f& dimRob = pRob_dim[rob_obb_idx];
                        const int verdict = paperTriTri(dimObs(0), dimObs(1), pObs_a[obs_obb_idx],
                                                        dimRob(0), dimRob(1), pRob_a[rob_obb_idx],
                                                        B, T);
                        if (verdict > 0) {
                            s_collision = true;
                        } else if (verdict < 0) {
                            // borderline/coplanar: full world-frame test
                            const Triangle& rt = pRob_tris[rob_tri];
                            const Triangle& ot = pObs_tris[obs_tri];
                            const Eigen::Vector3f rv0 = R_conf * pRob_verts[rt.v1] + T_conf;
                            const Eigen::Vector3f rv1 = R_conf * pRob_verts[rt.v2] + T_conf;
                            const Eigen::Vector3f rv2 = R_conf * pRob_verts[rt.v3] + T_conf;
                            if (!triangles_valid_f(rv0, rv1, rv2,
                                                   pObs_verts[ot.v1], pObs_verts[ot.v2], pObs_verts[ot.v3])) {
                                s_collision = true;
                            }
                        }
                    }
                    // robot is leaf, obstacle is not
                    else {
                        int pos = atomicAdd(&num_obb_pend, 1);
                        obs_obb_pend[pos] = obs_obb_idx;
                        rob_obb_pend[pos] = rob_obb_par_idx; 
                    }
                } 
                // obstacle is leaf, robot is not
                else if (obs_first_child_idx < 0) {

                    int pos = atomicAdd(&num_obb_pend, 1);
                    obs_obb_pend[pos] = obs_obb_par_idx; // just shove the parent back in, recurse only on robot children
                    rob_obb_pend[pos] = rob_obb_idx;
                }
            } // end while true over single configuration

            __syncthreads();
            const unsigned long long t_tri = (threadIdx.x == 0) ? globaltimer() : 0;
            if (threadIdx.x == 0) {
                acc_trav += t_tri - t_cfg;
            }
            if (s_collision) {
                continue;
            }
            if (threadIdx.x == 0) {
                s_disjoint_word |= 1u << (index & 31);
            }
            __syncthreads();
            if (threadIdx.x == 0) {
                acc_tri += globaltimer() - t_tri;
            }
        } // end for over pending configurations in batch

        // Flush the batch's bitpacked word once (one 4-byte write instead of
        // 32 threads storing the same byte).
        if (threadIdx.x == 0) {
            pdisjoint[batch_start >> 5] |= s_disjoint_word;
        }
    }
    return;
}

double bvh_naive(const BVNode_soa& rob_BVH, const BVNode_soa& obs_BVH,
                 const MeshData& rob_mesh, const MeshData& obs_mesh,
                 const std::vector<Configuration>& confs,
                 std::vector<bool>& valid, bool dry_run) {
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    std::cout << "Rob BVH size: " << rob_BVH.size << std::endl;
    std::cout << "Obs BVH size: " << obs_BVH.size << std::endl;

    const int num_confs = confs.size();
    const size_t num_words = (num_confs + 31) / 32;
    valid.assign(num_confs, false);
    if (num_confs == 0) {
        return 0.0;
    }

    // Bitpacked results: bit i of disjoint[i >> 5] is the result of config i.
    std::unique_ptr<uint32_t[]> disjoint(new uint32_t[num_words]());


    std::vector<Eigen::Matrix3f> rob_conf_r(num_confs);
    std::vector<Eigen::Vector3f> rob_conf_t(num_confs);
    for (int i = 0; i < num_confs; ++i) {
        rob_conf_r[i] = createRotationMatrix(confs[i]);
        rob_conf_t[i] = Eigen::Vector3f(confs[i].x, confs[i].y, confs[i].z);
    }

    const int blockSize = 32;
    // Persistent blocks pulling configs from a global work queue. Size the grid
    // so every SM is fully occupied (the kernel fits multiple blocks per SM).
    int device;
    cudaGetDevice(&device);
    int num_sms = 0;
    cudaDeviceGetAttribute(&num_sms, cudaDevAttrMultiProcessorCount, device);

    // Dynamic shared memory: just the two first_child arrays (read-only work set).
    const size_t smem_size = (obs_BVH.size + rob_BVH.size) * sizeof(int16_t);

    int blocks_per_sm = 0;
    cudaOccupancyMaxActiveBlocksPerMultiprocessor(&blocks_per_sm, d_bvh_naive, blockSize, smem_size);
    if (blocks_per_sm < 1) blocks_per_sm = 1;
    const int max_blocks = (num_confs + blockSize - 1) / blockSize;
    const int persistent_blocks = num_sms * blocks_per_sm;
    const int gridSize = (max_blocks < persistent_blocks) ? max_blocks : persistent_blocks;


    Eigen::Matrix3f* d_R_obs;
    Eigen::Vector3f* d_T_obs;
    Eigen::Matrix3f* d_R_rob;
    Eigen::Vector3f* d_T_rob;
    Eigen::Vector3f* d_Rob_dim;
    Eigen::Vector3f* d_Obs_dim;
    Eigen::Matrix3f* d_Rob_conf_rot;
    Eigen::Vector3f* d_Rob_conf_trans;
    Eigen::Vector3f* d_Rob_vertices;
    Eigen::Vector3f* d_Obs_vertices;
    Triangle * d_Rob_triangles;
    Triangle * d_Obs_triangles;
    int16_t* d_Obs_first_child;
    int16_t* d_Rob_first_child;
    float* d_Obs_a;
    float* d_Rob_a;
    uint32_t* pdisjoint;
    uint32_t* d_next_conf;
    uint64_t* d_phase;

    // Allocate memory for device pointers
    cudaEventRecord(start, 0);
    cudaMalloc((void**)&d_R_obs, obs_BVH.size * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_T_obs, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_R_rob, rob_BVH.size * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_T_rob, rob_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_dim, rob_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_dim, obs_BVH.size * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_conf_rot, num_confs * sizeof(Eigen::Matrix3f));
    cudaMalloc((void**)&d_Rob_conf_trans, num_confs * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_first_child, obs_BVH.size * sizeof(int16_t));
    cudaMalloc((void**)&d_Rob_first_child, rob_BVH.size * sizeof(int16_t));
    cudaMalloc((void**)&d_Obs_a, obs_BVH.size * sizeof(float));
    cudaMalloc((void**)&d_Rob_a, rob_BVH.size * sizeof(float));
    cudaMalloc((void**)&d_Rob_vertices, rob_mesh.vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Obs_vertices, obs_mesh.vertices.size() * sizeof(Eigen::Vector3f));
    cudaMalloc((void**)&d_Rob_triangles, rob_mesh.triangles.size() * sizeof(Triangle));
    cudaMalloc((void**)&d_Obs_triangles, obs_mesh.triangles.size() * sizeof(Triangle));
    cudaMalloc((void**)&pdisjoint, num_words * sizeof(uint32_t));
    cudaMalloc((void**)&d_next_conf, sizeof(uint32_t));
    checkCudaMem(cudaMemset(d_next_conf, 0, sizeof(uint32_t)));
    cudaMalloc((void**)&d_phase, 4 * sizeof(uint64_t));
    checkCudaMem(cudaMemset(d_phase, 0, 4 * sizeof(uint64_t)));

    cudaDeviceSynchronize();
    checkCudaMem(cudaMemcpy(d_R_obs, obs_BVH.pR, obs_BVH.size * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_T_obs, obs_BVH.pT, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_R_rob, rob_BVH.pR, rob_BVH.size * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_T_rob, rob_BVH.pT, rob_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_dim, rob_BVH.pDim, rob_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_dim, obs_BVH.pDim, obs_BVH.size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_first_child, rob_BVH.first_child, rob_BVH.size * sizeof(int16_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_first_child, obs_BVH.first_child, obs_BVH.size * sizeof(int16_t), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_a, obs_BVH.pA, obs_BVH.size * sizeof(float), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_a, rob_BVH.pA, rob_BVH.size * sizeof(float), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_vertices, rob_mesh.vertices.data(), rob_mesh.vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_vertices, obs_mesh.vertices.data(), obs_mesh.vertices.size() * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_triangles, rob_mesh.triangles.data(), rob_mesh.triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Obs_triangles, obs_mesh.triangles.data(), obs_mesh.triangles.size() * sizeof(Triangle), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemset(pdisjoint, 0, num_words * sizeof(uint32_t)));
    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float duration = 0;
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Initial allocation and transfer to GPU took " << duration << " ms." << std::endl;

    // Copy configurations over separately
    cudaEventRecord(start, 0);
    checkCudaMem(cudaMemcpy(d_Rob_conf_rot, rob_conf_r.data(), num_confs * sizeof(Eigen::Matrix3f), cudaMemcpyHostToDevice));
    checkCudaMem(cudaMemcpy(d_Rob_conf_trans, rob_conf_t.data(), num_confs * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));
    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Copying configurations to GPU took " << duration << " ms." << std::endl;

    std::cout << "obs_BVH.size: " << obs_BVH.size << ", rob_BVH.size: " << rob_BVH.size << std::endl;

    auto launch_bvh_naive = [&]() {
        d_bvh_naive<<<gridSize, blockSize, smem_size>>>(
                                                d_R_obs, d_T_obs,
                                                d_R_rob, d_T_rob,
                                                d_Obs_dim, d_Rob_dim,
                                                d_Rob_conf_rot, d_Rob_conf_trans,
                                                d_Obs_first_child, d_Rob_first_child,
                                                d_Obs_a, d_Rob_a,
                                                d_Rob_vertices, d_Rob_triangles, rob_BVH.size,
                                                d_Obs_vertices, d_Obs_triangles, obs_BVH.size,
                                                pdisjoint, static_cast<size_t>(num_confs), d_next_conf, (unsigned long long*)d_phase);
    };

    // Dry run: single-block launch over one batch, untimed, purely to get the
    // kernel loaded onto the device (the work queue would otherwise drain the
    // entire workload in this block).
    if (dry_run) {
        const size_t dry_confs = (num_confs < (size_t)blockSize) ? num_confs : (size_t)blockSize;
        d_bvh_naive<<<1, blockSize, smem_size>>>(
                                                d_R_obs, d_T_obs,
                                                d_R_rob, d_T_rob,
                                                d_Obs_dim, d_Rob_dim,
                                                d_Rob_conf_rot, d_Rob_conf_trans,
                                                d_Obs_first_child, d_Rob_first_child,
                                                d_Obs_a, d_Rob_a,
                                                d_Rob_vertices, d_Rob_triangles, rob_BVH.size,
                                                d_Obs_vertices, d_Obs_triangles, obs_BVH.size,
                                                pdisjoint, dry_confs, d_next_conf, (unsigned long long*)d_phase);
        checkCudaMem(cudaGetLastError());
        checkCudaMem(cudaDeviceSynchronize());
        std::cout << "BVH Naive dry run completed successfully." << std::endl;
        // Reset the work queue so the timed launch starts from configuration 0
        checkCudaMem(cudaMemset(d_next_conf, 0, sizeof(uint32_t)));
        checkCudaMem(cudaMemset(d_phase, 0, 4 * sizeof(uint64_t)));
        checkCudaMem(cudaMemset(pdisjoint, 0, num_words * sizeof(uint32_t)));
    }

    cudaEventRecord(start, 0);
    launch_bvh_naive();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "BVH Naive GPU kernel took " << duration << " ms for " << num_confs << " configurations." << std::endl;

    // Profiling: phase totals are summed across blocks (each block reports
    // once at termination); report the per-block average, which is a
    // block-serial view and NOT comparable to the wall-clock kernel time.
    uint64_t h_phase[4];
    checkCudaMem(cudaMemcpy(h_phase, d_phase, 4 * sizeof(uint64_t), cudaMemcpyDeviceToHost));
    const double ns_per_ms = 1e6;
    const double nblocks = (h_phase[3] > 0) ? (double)h_phase[3] : 1.0;
    std::cout << "PHASES ms (avg per block, " << h_phase[3] << " blocks, block-serial, not wall-clock): init="
              << (double)h_phase[0] / ns_per_ms / nblocks
              << " traversal=" << (double)h_phase[1] / ns_per_ms / nblocks
              << " triangles=" << (double)h_phase[2] / ns_per_ms / nblocks << std::endl;

    // Copy result back to host (num_confs * sizeof(bool))
    //TODO deleteme
    for (int i = 0; i < 10000000; ++i) {
        i++;
    }
    cudaEventRecord(start, 0);
    checkCudaMem(cudaGetLastError());

    checkCudaMem(cudaMemcpy(disjoint.get(), pdisjoint, num_words * sizeof(uint32_t), cudaMemcpyDeviceToHost));
    cudaDeviceSynchronize();
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&duration, start, stop);
    std::cout << "Copying results from GPU took " << duration << " ms." << std::endl;

    for (int i = 0; i < num_confs; ++i) {
        valid[i] = (disjoint[i >> 5] >> (i & 31)) & 1u;
    }

    // Free device memory
    cudaFree(d_R_obs);
    cudaFree(d_T_obs);
    cudaFree(d_R_rob);
    cudaFree(d_T_rob);
    cudaFree(d_Rob_dim);
    cudaFree(d_Obs_dim);
    cudaFree(d_Rob_conf_rot);
    cudaFree(d_Rob_conf_trans);
    cudaFree(d_Obs_first_child);
    cudaFree(d_Rob_first_child);
    cudaFree(d_Obs_a);
    cudaFree(d_Rob_a);
    cudaFree(d_Rob_vertices);
    cudaFree(d_Obs_vertices);
    cudaFree(d_Rob_triangles);
    cudaFree(d_Obs_triangles);
    cudaFree(pdisjoint);
    cudaFree(d_next_conf);
    cudaFree(d_phase);

    return duration;
}
