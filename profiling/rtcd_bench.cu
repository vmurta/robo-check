// Apples-to-apples benchmark harness for robo-check's articulated (URDF)
// collision detection vs RTCollisionDetection's discrete benchmark:
//   - same robot: Franka Panda visual meshes (merged_meshes), links 1..7
//     (base skipped, mirroring RTCD's SKIP_BASE)
//   - same pose pool: data/motions/panda8192.bin (8192 random joint poses)
//   - same scenes: shelfSimple / shelf / denseShelf (curobo meshes, same
//     world poses as RTCD/CollisionScenes/Scene/*.h)
//   - same query semantics: robot links (with forward kinematics) vs the
//     whole scene; a pose is "in collision" if any link collides.
//
// GPU results are validated against an FCL ground truth on the same triangle
// sets (fcl::BVHModel<OBBRSSf>, fcl::collide) and the harness reports
// false positives / false negatives explicitly.

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cfloat>
#include <chrono>
#include <string>
#include <vector>
#include <array>
#include <memory>
#include <cstring>
#include <algorithm>
#include <unistd.h>

#include <Eigen/Dense>
#include <fcl/fcl.h>

#include "Utils.h"
#include "Triangle.hu"
#include "OBB-BVH-naive.hu"
#include "ArticulatedRobot.hu"
#include "Edge.hu"

// ---------------------------------------------------------------------------
// Panda FK ground truth (matches RTCD's mask-based FK and the URDF values).
// ---------------------------------------------------------------------------
struct PandaFK {
    const std::array<Eigen::Vector3f, 7> origin{
        Eigen::Vector3f(0.0f, 0.0f, 0.333f),
        Eigen::Vector3f(0.0f, 0.0f, 0.0f),
        Eigen::Vector3f(0.0f, -0.316f, 0.0f),
        Eigen::Vector3f(0.0825f, 0.0f, 0.0f),
        Eigen::Vector3f(-0.0825f, 0.384f, 0.0f),
        Eigen::Vector3f(0.0f, 0.0f, 0.0f),
        Eigen::Vector3f(0.088f, 0.0f, 0.0f),
    };
    const std::array<Eigen::Vector3f, 7> rpy{
        Eigen::Vector3f(0.0f, 0.0f, 0.0f),
        Eigen::Vector3f(-1.57079632679f, 0.0f, 0.0f),
        Eigen::Vector3f(1.57079632679f, 0.0f, 0.0f),
        Eigen::Vector3f(1.57079632679f, 0.0f, 0.0f),
        Eigen::Vector3f(-1.57079632679f, 0.0f, 0.0f),
        Eigen::Vector3f(1.57079632679f, 0.0f, 0.0f),
        Eigen::Vector3f(1.57079632679f, 0.0f, 0.0f),
    };

    void compute(const std::array<float, 7>& q, std::array<Eigen::Isometry3f, 7>& out) const {
        Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
        for (int i = 0; i < 7; ++i) {
            Eigen::Isometry3f Tjoint = Eigen::Isometry3f::Identity();
            Tjoint.translate(origin[i]);
            Tjoint.rotate(Eigen::AngleAxisf(rpy[i][2], Eigen::Vector3f::UnitZ()));
            Tjoint.rotate(Eigen::AngleAxisf(rpy[i][1], Eigen::Vector3f::UnitY()));
            Tjoint.rotate(Eigen::AngleAxisf(rpy[i][0], Eigen::Vector3f::UnitX()));
            T = T * Tjoint * Eigen::AngleAxisf(q[i], Eigen::Vector3f::UnitZ());
            out[i] = T;
        }
    }
};

// ---------------------------------------------------------------------------
// Scene definitions (from RTCD/CollisionScenes/Scene/{shelfSimple,shelf,denseShelf}.h)
// ---------------------------------------------------------------------------
struct ObstacleDef {
    std::string file;
    Eigen::Vector3f pos;
};

static std::vector<ObstacleDef> makeScene(const std::string& scene) {
    if (scene == "dense") {
        return {
            {"shelves.obj", {0.45f, 0.0f, 0.4f}},
            {"bin_dense.obj", {0.0f, -0.6f, 0.0f}},
            {"bin_dense.obj", {0.0f, 0.6f, 0.0f}},
            {"Bob.obj", {-0.4f, 0.0f, 0.0f}},
            {"Face1.obj", {-0.4f, 0.0f, 0.0f}},
            {"Face2.obj", {-0.4f, 0.0f, 0.0f}},
            {"Face3.obj", {-0.4f, 0.0f, 0.0f}},
            {"Cow.obj", {-0.4f, 0.0f, 0.0f}},
            {"Fish.obj", {-0.4f, 0.0f, 0.0f}},
            {"Sheep.obj", {-0.4f, 0.0f, 0.0f}},
            {"Snakeboard.obj", {-0.4f, 0.0f, 0.0f}},
        };
    } else if (scene == "shelf") {
        return {
            {"shelves.obj", {0.45f, 0.0f, 0.4f}},
            {"bin.obj", {0.0f, -0.6f, 0.0f}},
            {"bin.obj", {0.0f, 0.6f, 0.0f}},
            {"Bob.obj", {-0.4f, 0.0f, 0.0f}},
            {"Cow.obj", {-0.4f, 0.0f, 0.0f}},
            {"Fish.obj", {-0.4f, 0.0f, 0.0f}},
            {"Sheep.obj", {-0.4f, 0.0f, 0.0f}},
            {"Snakeboard.obj", {-0.4f, 0.0f, 0.0f}},
        };
    } else if (scene == "rtcc") {
        // MoveIt rtcc_benchmark scene (benchmarkFCLVizPanda's Viz overload of
        // setCollisionScene): shelves @ (0.45,0,0.4), bins @ (0,+-0.6,0), and
        // Bob/Face1/Face2/Face3/Cow/Fish/Sheep/Snakeboard @ (-0.4,0,0)
        // ("Board" in the benchmark is Snakeboard.obj).
        return {
            {"shelves.obj", {0.45f, 0.0f, 0.4f}},
            {"bin.obj", {0.0f, -0.6f, 0.0f}},
            {"bin.obj", {0.0f, 0.6f, 0.0f}},
            {"Bob.obj", {-0.4f, 0.0f, 0.0f}},
            {"Face1.obj", {-0.4f, 0.0f, 0.0f}},
            {"Face2.obj", {-0.4f, 0.0f, 0.0f}},
            {"Face3.obj", {-0.4f, 0.0f, 0.0f}},
            {"Cow.obj", {-0.4f, 0.0f, 0.0f}},
            {"Fish.obj", {-0.4f, 0.0f, 0.0f}},
            {"Sheep.obj", {-0.4f, 0.0f, 0.0f}},
            {"Snakeboard.obj", {-0.4f, 0.0f, 0.0f}},
        };
    } else { // "simple"
        return {
            {"shelves.obj", {0.45f, 0.0f, 0.4f}},
            {"bin.obj", {0.0f, -0.6f, 0.0f}},
            {"bin.obj", {0.0f, 0.6f, 0.0f}},
        };
    }
}

// ---------------------------------------------------------------------------
// Pose pool (same layout as RTCD's readBin/writeBin)
// ---------------------------------------------------------------------------
static void readBin(const std::string& filename, std::vector<std::array<float, 7>>& data) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open pose file: " << filename << std::endl;
        exit(1);
    }
    data.clear();
    while (true) {
        std::array<float, 7> t;
        file.read(reinterpret_cast<char*>(t.data()), sizeof(float) * 7);
        if (file.eof()) break;
        if (file.gcount() != sizeof(float) * 7) {
            std::cerr << "Truncated pose file: " << filename << std::endl;
            exit(1);
        }
        data.push_back(t);
    }
    std::cout << "Loaded " << data.size() << " poses from " << filename << std::endl;
}

// ---------------------------------------------------------------------------
// Scene loading: merge all obstacles (with world transforms) into one mesh,
// mirroring the geometry of RTCD's scene (one BVH over the whole scene).
// ---------------------------------------------------------------------------
struct SceneData {
    MeshData mesh;       // merged vertices + triangles (world frame)
    BVNode_soa<int32_t> bvh; // 4-ary BVH over the merged scene (int32: shelf/dense scenes > 32767 nodes)
    std::string mergedObjPath; // merged OBJ written to disk (for rebuilding an int16 BVH in debug mode)
    std::vector<std::pair<std::string, Eigen::Vector3f>> obstacles; // per-obstacle, for FCL ground truth
    SceneData() : bvh(0) {}
};

static SceneData buildScene(const std::string& sceneDir, const std::string& sceneName) {
    SceneData sd;
    sd.obstacles = std::vector<std::pair<std::string, Eigen::Vector3f>>();
    for (const auto& o : makeScene(sceneName)) {
        sd.obstacles.emplace_back(o.file, o.pos);
    }
    // Build the merged mesh for the GPU side and as FCL ground truth input.
    sd.mesh.vertices.clear();
    sd.mesh.triangles.clear();
    for (const auto& ob : sd.obstacles) {
        std::string path = sceneDir + "/" + ob.first;
        std::vector<Eigen::Vector3f> verts;
        std::vector<Triangle> tris;
        loadOBJFile(path, verts, tris);
        if (verts.empty() || tris.empty()) {
            std::cerr << "Failed to load obstacle " << path << std::endl;
            exit(1);
        }
        const size_t base = sd.mesh.vertices.size();
        for (const auto& v : verts) {
            sd.mesh.vertices.push_back(v + ob.second);
        }
        for (const auto& t : tris) {
            Triangle tt;
            tt.v1 = t.v1 + (int)base;
            tt.v2 = t.v2 + (int)base;
            tt.v3 = t.v3 + (int)base;
            sd.mesh.triangles.push_back(tt);
        }
        std::cout << "  obstacle " << ob.first << ": " << tris.size() << " tris" << std::endl;
    }
    std::cout << "Scene '" << sceneName << "': " << sd.mesh.triangles.size() << " triangles total" << std::endl;

    // Write merged OBJ so the existing BVH builder path (file-based) is used.
    // mkstemp: per-process unique path so concurrent benchmark/profiling runs
    // cannot collide on a shared temp file.
    char tmp_template[] = "/tmp/rtcd_scene_merged_XXXXXX";
    const int tmpfd = mkstemp(tmp_template);
    if (tmpfd < 0) {
        std::cerr << "Cannot create merged-scene temp file" << std::endl;
        exit(1);
    }
    close(tmpfd);
    const char* tmp = tmp_template;
    FILE* f = fopen(tmp, "wb");
    if (!f) {
        std::cerr << "Cannot write " << tmp << std::endl;
        exit(1);
    }
    for (const auto& v : sd.mesh.vertices) {
        fprintf(f, "v %g %g %g\n", v.x(), v.y(), v.z());
    }
    for (const auto& t : sd.mesh.triangles) {
        fprintf(f, "f %d %d %d\n", t.v1 + 1, t.v2 + 1, t.v3 + 1);
    }
    fclose(f);

    sd.bvh = BVH_n_ary_hierarchy_from_mesh<int32_t>(tmp, 2);
    sd.mergedObjPath = tmp;
    std::cout << "Scene BVH: " << sd.bvh.size << " nodes" << std::endl;
    return sd;
}

// ---------------------------------------------------------------------------
// Parallel-phase gate node lists (G0 root / Gk merged-BVH cut / Gobs roots)
// ---------------------------------------------------------------------------
// 4-ary levels are serialized contiguously in the SoA; level k+1 holds exactly
// 4 slots per internal slot of level k, in order. Cut(k) = non-dummy slots at
// level k + leaf slots at levels 1..k-1 (shallow leaves are NOT carried to
// deeper levels, so a naive "all level-k slots" gate would miss their tris).
static std::vector<int> cutIndices(const BVNode_soa<int32_t>& bvh, int k) {
    if (k <= 0) return {0};
    std::vector<int> lvl_start(1, 0), lvl_size(1, 1);
    while ((int)lvl_start.size() <= k) {
        int internal = 0;
        const int s = lvl_start.back();
        const int n = lvl_size.back();
        for (int i = s; i < s + n; ++i) {
            if (bvh.first_child[i] > 0) internal++;
        }
        if (internal == 0) break; // tree bottoms out above level k
        lvl_start.push_back(s + n);
        lvl_size.push_back(4 * internal);
    }
    std::vector<int> cut;
    for (int lv = 1; lv <= k && lv < (int)lvl_start.size(); ++lv) {
        const int s = lvl_start[lv];
        const int end = s + lvl_size[lv];
        for (int i = s; i < end; ++i) {
            const int fc = bvh.first_child[i];
            if (fc == 0) continue;            // dummy (padding)
            if (lv < k && fc > 0) continue;   // internal above the cut
            cut.push_back(i);                 // level-k slot or shallow leaf
        }
    }
    return cut;
}

// Gate node lists for the gate modes:
//   0 = empty (wrapper default: obs root only)
//   1 = merged-BVH cut at 4-ary level 1
//   2 = merged-BVH cut at 4-ary level 2
//   3 = per-obstacle root OBBs (built by the validated BVH builder, node 0,
//       T += scene offset). Nodes are sorted largest-volume first so the
//       gate loop exits early on proximal links.
static void buildGateLists(int gateMode, const SceneData& sd, const std::string& sceneDir,
                           std::vector<Eigen::Matrix3f>& gR,
                           std::vector<Eigen::Vector3f>& gT,
                           std::vector<Eigen::Vector3f>& gD) {
    gR.clear(); gT.clear(); gD.clear();
    if (gateMode == 0) return;
    if (gateMode == 1 || gateMode == 2) {
        const int k = gateMode;
        const std::vector<int> cut = cutIndices(sd.bvh, k);
        for (int idx : cut) {
            gR.push_back(sd.bvh.pR[idx]);
            gT.push_back(sd.bvh.pT[idx]);
            gD.push_back(sd.bvh.pDim[idx]);
        }
    } else if (gateMode == 3) {
        for (const auto& ob : sd.obstacles) {
            const std::string path = sceneDir + "/" + ob.first;
            BVNode_soa<int32_t> bvh = BVH_n_ary_hierarchy_from_mesh<int32_t>(path.c_str(), 2);
            gR.push_back(bvh.pR[0]);
            gT.push_back(bvh.pT[0] + ob.second);
            gD.push_back(bvh.pDim[0]);
        }
    } else {
        std::cerr << "buildGateLists: unknown gate mode " << gateMode << std::endl;
        exit(1);
    }
    std::vector<int> order(gR.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = (int)i;
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        const Eigen::Vector3f da = gD[a], db = gD[b];
        return (da.x() * da.y() * da.z()) > (db.x() * db.y() * db.z());
    });
    std::vector<Eigen::Matrix3f> sR(gR.size());
    std::vector<Eigen::Vector3f> sT(gT.size()), sD(gD.size());
    for (size_t i = 0; i < order.size(); ++i) {
        sR[i] = gR[order[i]];
        sT[i] = gT[order[i]];
        sD[i] = gD[order[i]];
    }
    gR.swap(sR);
    gT.swap(sT);
    gD.swap(sD);
}

// ---------------------------------------------------------------------------
// FCL ground truth: mirrors the RTCD FCLBenchmark harness
// (fcl::BVHModel<OBBRSSf> per link/per obstacle, fcl::collide).
// ---------------------------------------------------------------------------
using MeshPtr = std::shared_ptr<fcl::BVHModel<fcl::OBBRSSf>>;

static MeshPtr makeFCLMesh(const std::vector<Eigen::Vector3f>& verts, const std::vector<Triangle>& tris) {
    std::vector<fcl::Vector3f> fverts;
    std::vector<fcl::Triangle> ftris;
    fverts.reserve(verts.size());
    ftris.reserve(tris.size());
    for (const auto& v : verts) fverts.push_back(fcl::Vector3f(v.x(), v.y(), v.z()));
    for (const auto& t : tris) ftris.push_back(fcl::Triangle(t.v1, t.v2, t.v3));

    MeshPtr mesh = std::make_shared<fcl::BVHModel<fcl::OBBRSSf>>();
    mesh->beginModel((int)ftris.size(), (int)fverts.size());
    mesh->addSubModel(fverts, ftris);
    mesh->endModel();
    return mesh;
}

static bool poseInCollisionFCL(const std::array<Eigen::Isometry3f, 7>& tfms,
                               const std::vector<MeshPtr>& links,
                               const std::vector<MeshPtr>& obstacles,
                               bool earlyExit) {
    fcl::CollisionRequestf request;
    request.num_max_contacts = 1;
    request.enable_contact   = false;
    request.enable_cost      = false;
    fcl::CollisionResultf result;
    static const fcl::Transform3f identity = fcl::Transform3f::Identity();

    bool inCollision = false;
    for (size_t l = 0; l < links.size(); ++l) {
        for (const auto& obs : obstacles) {
            result.clear();
            fcl::Transform3f tf;
            tf.translation() = fcl::Vector3f(tfms[l].translation().x(), tfms[l].translation().y(), tfms[l].translation().z());
            tf.linear() = tfms[l].linear();
            fcl::collide(links[l].get(), tf, obs.get(), identity, request, result);
            if (result.isCollision()) {
                if (earlyExit) {
                    return true;
                }
                inCollision = true;
            }
        }
    }
    return inCollision;
}

// ---------------------------------------------------------------------------
// FK equivalence check: robo-check's parsed URDF FK vs the PandaFK reference.
// ---------------------------------------------------------------------------
static float fkMaxError(const ArticulatedRobot& robot, const std::vector<std::array<float, 7>>& poses) {
    PandaFK ref;
    float maxErr = 0.0f;
    for (const auto& q : poses) {
        std::array<Eigen::Isometry3f, 7> refTf;
        ref.compute(q, refTf);

        // robo-check FK: link_R[1..7], link_T[1..7]
        Eigen::Matrix3f link_R[8];
        Eigen::Vector3f link_T[8];
        articulated_conf<7> conf;
        for (int i = 0; i < 7; ++i) conf[i] = q[i];
        forwardKinematics(conf, robot.joints.data(), link_R, link_T);

        for (int l = 1; l <= 7; ++l) {
            Eigen::Matrix3f dR = link_R[l] - refTf[l - 1].linear();
            float err = dR.cwiseAbs().maxCoeff();
            Eigen::Vector3f dT = link_T[l] - refTf[l - 1].translation();
            err = std::max(err, dT.cwiseAbs().maxCoeff());
            maxErr = std::max(maxErr, err);
        }
    }
    return maxErr;
}

// ---------------------------------------------------------------------------
// Host replica of triangles_valid (double path), same helper calls as the
// device implementation.
// ---------------------------------------------------------------------------
static bool triValidHost(const Eigen::Vector3f& f_rob_v1, const Eigen::Vector3f& f_rob_v2, const Eigen::Vector3f& f_rob_v3,
                         const Eigen::Vector3f& f_obs_v1, const Eigen::Vector3f& f_obs_v2, const Eigen::Vector3f& f_obs_v3) {
    Eigen::Vector3d rob_v1 = f_rob_v1.cast<double>();
    Eigen::Vector3d rob_v2 = f_rob_v2.cast<double>();
    Eigen::Vector3d rob_v3 = f_rob_v3.cast<double>();
    Eigen::Vector3d obs_v1 = f_obs_v1.cast<double>();
    Eigen::Vector3d obs_v2 = f_obs_v2.cast<double>();
    Eigen::Vector3d obs_v3 = f_obs_v3.cast<double>();

    Eigen::Vector3d Nr;
    double dr;
    compute_plane(f_rob_v1, f_rob_v2, f_rob_v3, Nr, dr);

    Eigen::Vector3d distO = compute_signed_dists(Nr, dr, obs_v1, obs_v2, obs_v3);
    if (no_overlap(distO)) {
        return true;
    }

    Eigen::Vector3d No;
    double do_;
    compute_plane(obs_v1, obs_v2, obs_v3, No, do_);
    if (is_coplanar(Nr, dr, No, do_)) {
        return false;
    }

    Eigen::Vector3d distR = compute_signed_dists(No, do_, rob_v1, rob_v2, rob_v3);
    if (no_overlap(distR)) {
        return true;
    }

    Eigen::Vector3d D, O;
    compute_intersect_line(Nr, dr, No, do_, &D, &O);
    O = obs_v1;

    canonicalize_triangle(rob_v1, rob_v2, rob_v3, distR);
    canonicalize_triangle(obs_v1, obs_v2, obs_v3, distO);

    double t_r01 = compute_parametric_variable(rob_v1, rob_v2, distR[0], distR[1], D, O);
    double t_r12 = compute_parametric_variable(rob_v2, rob_v3, distR[1], distR[2], D, O);
    double t_o01 = compute_parametric_variable(obs_v1, obs_v2, distO[0], distO[1], D, O);
    double t_o12 = compute_parametric_variable(obs_v2, obs_v3, distO[1], distO[2], D, O);

    if (std::min(t_r01, t_r12) > std::max(t_o01, t_o12) + 1e-7) {
        return true;
    } else if (std::min(t_o01, t_o12) > std::max(t_r01, t_r12) + 1e-7) {
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
static void usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "  --scene simple|shelf|dense   scene (default: simple)\n"
              << "  --poses <file.bin>           pose pool (default: ./data/rtcd/panda8192.bin)\n"
              << "  --urdf <file>                robot URDF (default: ./data/rtcd/panda_nobase.urdf)\n"
              << "  --meshes <dir>               panda link mesh dir (default: ./data/rtcd/meshes)\n"
              << "  --scene-dir <dir>            scene model dir (default: ./data/rtcd/scene)\n"
              << "  --nposes N                   use first N poses (default: 8192)\n"
              << "  --sweep                      sweep batch sizes 1..4096 like RTCD\n"
              << "  --repeat R                   repeat timed runs R times (default: 1)\n"
              << "  --kernel-mode N              run only kernel variant N (default: all 4 AB cells)\n"
              << "  --no-fcl                     skip FCL ground-truth comparison\n"
              << "  --dump-labels <file>         write FCL per-pose collision labels (binary uint8)\n"
              << "  --verify <file>              verify only the pose indices listed in <file> (one\n"
              << "                               per line) with FCL and report the false-collision\n"
              << "                               count + timing (pipeline double-check metric)\n"
              << "  --edge-check <n>             articulated edge checking: n consecutive-pose edges,\n"
              << "                               GPU vs FCL, FP/FN + timing, then exit\n"
              << "  --edge-lvs <float>           longest valid joint delta per edge step (default 0.1 rad)\n"
              << "  --gate-sweep                 parallel-phase gate sweep: G0 root / G1 lv1 / G2 lv2 /\n"
              << "                               Gobs per-obstacle roots on the quatSAT layout (4 cells)\n"
              << "  --csv <file>                 append results as CSV\n";
}

int main(int argc, char** argv) {
    std::string scene     = "simple";
    std::string posesFile = "./data/rtcd/panda8192.bin";
    std::string urdfPath  = "./data/rtcd/panda_nobase.urdf";
    std::string meshDir   = "./data/rtcd/meshes";
    std::string sceneDir  = "./data/rtcd/scene";
    size_t nPoses         = 8192;
    bool sweep            = false;
    int repeat            = 1;
    int kernelModeOverride = -1;
    bool doFCL            = true;
    std::string csvFile;
    std::string labelsFile;
    std::string verifyFile;
    int debugPose         = -1;
    size_t edgeCheckN     = 0;
    float edgeLvs         = 0.1f;
    int edgeKmode         = 64;
    bool gateSweep        = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            return (i + 1 < argc) ? std::string(argv[++i]) : std::string();
        };
        if (a == "--scene") scene = next();
        else if (a == "--poses") posesFile = next();
        else if (a == "--urdf") urdfPath = next();
        else if (a == "--meshes") meshDir = next();
        else if (a == "--scene-dir") sceneDir = next();
        else if (a == "--nposes") nPoses = (size_t)atoi(next().c_str());
        else if (a == "--sweep") sweep = true;
        else if (a == "--repeat") repeat = atoi(next().c_str());
        else if (a == "--kernel-mode") kernelModeOverride = atoi(next().c_str());
        else if (a == "--no-fcl") doFCL = false;
        else if (a == "--csv") csvFile = next();
        else if (a == "--dump-labels") labelsFile = next();
        else if (a == "--verify") verifyFile = next();
        else if (a == "--debug-pose") debugPose = atoi(next().c_str());
        else if (a == "--edge-check") edgeCheckN = (size_t)atoi(next().c_str());
        else if (a == "--edge-lvs") edgeLvs = (float)atof(next().c_str());
        else if (a == "--edge-kmode") edgeKmode = atoi(next().c_str());
        else if (a == "--gate-sweep") gateSweep = true;
        else if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
        else { std::cerr << "Unknown argument: " << a << std::endl; usage(argv[0]); return 1; }
    }

    std::cout << "=== robo-check articulated URDF benchmark (vs RTCD scenes) ===\n";

    // --- poses ---
    std::vector<std::array<float, 7>> traj;
    readBin(posesFile, traj);
    if (nPoses > traj.size()) nPoses = traj.size();
    traj.resize(nPoses);

    // --- robot FK sanity ---
    ArticulatedRobot robot;
    if (!parseSerialChainURDF(urdfPath, robot)) {
        std::cerr << "URDF parse failed" << std::endl;
        return 1;
    }
    std::cout << "URDF: " << robot.num_joints << " joints, " << robot.link_names.size() << " links\n";
    for (size_t i = 0; i < robot.link_meshes.size(); ++i) {
        std::cout << "  link " << i << " " << robot.link_names[i] << " -> " << robot.link_meshes[i] << "\n";
    }
    const float fkErr = fkMaxError(robot, traj);
    std::cout << "FK max deviation (robo-check parser vs PandaFK reference): " << fkErr << std::endl;
    if (fkErr > 1e-4f) {
        std::cerr << "FK MISMATCH > 1e-4 -- aborting (URDF parse or FK convention is wrong)" << std::endl;
        return 1;
    }

    // --- scene ---
    SceneData sd = buildScene(sceneDir, scene);

    // --- parallel-phase gate node lists (modes 1..3; mode 0 = obs root) ---
    std::vector<Eigen::Matrix3f> gateR[4];
    std::vector<Eigen::Vector3f> gateT[4], gateD[4];
    for (int gm = 1; gm <= 3; ++gm) {
        buildGateLists(gm, sd, sceneDir, gateR[gm], gateT[gm], gateD[gm]);
        if (!gateR[gm].empty()) {
            std::cout << "gate mode " << gm << ": " << gateR[gm].size() << " nodes" << std::endl;
        }
    }

    // --- per-link bisection debug mode ---
    if (debugPose >= 0 && debugPose < (int)traj.size()) {
        const auto& q = traj[debugPose];
        std::cout << "DEBUG pose " << debugPose << ": q =";
        for (int j = 0; j < 7; ++j) std::cout << " " << q[j];
        std::cout << std::endl;

        PandaFK fk;
        std::array<Eigen::Isometry3f, 7> tfms;
        fk.compute(q, tfms);

        // FCL merged-scene model (single mesh) for per-link checks
        MeshPtr fclScene = makeFCLMesh(sd.mesh.vertices, sd.mesh.triangles);

        bool anyFCL = false, anyRigid = false, anyArtic = false;
        for (int l = 1; l <= 7; ++l) {
            std::string path = meshDir + "/panda_link" + std::to_string(l) + "_visual.obj";
            std::vector<Eigen::Vector3f> verts;
            std::vector<Triangle> tris;
            loadOBJFile(path, verts, tris);
            MeshPtr fclLink = makeFCLMesh(verts, tris);

            fcl::CollisionRequestf request;
            request.num_max_contacts = 1;
            request.enable_contact = false;
            request.enable_cost = false;
            fcl::CollisionResultf result;
            fcl::Transform3f tf;
            tf.translation() = fcl::Vector3f(tfms[l - 1].translation().x(), tfms[l - 1].translation().y(), tfms[l - 1].translation().z());
            tf.linear() = tfms[l - 1].linear();
            fcl::collide(fclLink.get(), tf, fclScene.get(), fcl::Transform3f::Identity(), request, result);
            const bool fclHit = result.isCollision();
            anyFCL |= fclHit;

            // distance query: reveals near-contact/coplanar cases where the
            // boolean collide() predicate is unreliable
            fcl::DistanceRequestf dreq;
            dreq.enable_signed_distance = false;
            fcl::DistanceResultf dres;
            fcl::distance(fclLink.get(), tf, fclScene.get(), fcl::Transform3f::Identity(), dreq, dres);
            const float minDist = dres.min_distance;

            // rigid kernel on the same link (FK transform -> Configuration)
            BVNode_soa<> linkBVH = BVH_n_ary_hierarchy_from_mesh<>(path.c_str(), 2);
            MeshData linkMesh;
            linkMesh.vertices = verts;
            linkMesh.triangles = tris;
            const Eigen::Matrix3f& R = tfms[l - 1].linear();
            const Eigen::Vector3f& T = tfms[l - 1].translation();
            Configuration c;
            c.x = T.x(); c.y = T.y(); c.z = T.z();
            c.pitch = atan2f(-R(2, 0), sqrtf(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
            c.yaw = atan2f(R(1, 0), R(0, 0));
            c.roll = atan2f(R(2, 1), R(2, 2));
            std::vector<Configuration> confs{c};
            std::vector<bool> valid;
            // bvh_naive (rigid) uses an int16 first_child BVH; rebuild the
            // scene BVH with the default child type for this cross-check.
            BVNode_soa<> sceneBvh16 = BVH_n_ary_hierarchy_from_mesh<>(sd.mergedObjPath.c_str(), 2);
            bvh_naive(linkBVH, sceneBvh16, linkMesh, sd.mesh, confs, valid, true);
            const bool rigidHit = !valid[0]; // rigid kernel: valid=true means disjoint
            anyRigid |= rigidHit;

            std::cout << "  link " << l << ": FCL=" << (fclHit ? "HIT" : "free")
                      << " rigidGPU=" << (rigidHit ? "HIT" : "free")
                      << " minDist=" << minDist
                      << (fclHit == rigidHit ? "" : "  <-- MISMATCH") << std::endl;
        }

        // articulated kernel on this single pose
        {
            std::vector<articulated_conf<7>> one(1);
            for (int j = 0; j < 7; ++j) one[0][j] = q[j];
            std::vector<bool> valid;
            bvh_articulated<7>(urdfPath, sd.bvh, sd.mesh, one, valid, true);
            anyArtic = !valid[0];
        }
        std::cout << "  FCL=" << (anyFCL ? "HIT" : "free")
                  << " rigidGPU=" << (anyRigid ? "HIT" : "free")
                  << " articulatedGPU=" << (anyArtic ? "HIT" : "free") << std::endl;

        // Host-side replica of the articulated traversal for the first FCL-HIT
        // link the articulated kernel missed: dumps every leaf pair whose OBBs
        // overlap (same obbOverlap math) and the host triangle-test verdict.
        {
            for (int l = 1; l <= 7; ++l) {
                std::string path = meshDir + "/panda_link" + std::to_string(l) + "_visual.obj";
                std::vector<Eigen::Vector3f> verts;
                std::vector<Triangle> tris;
                loadOBJFile(path, verts, tris);
                MeshPtr fclLink = makeFCLMesh(verts, tris);
                fcl::CollisionRequestf request;
                request.num_max_contacts = 1;
                request.enable_contact = false;
                request.enable_cost = false;
                fcl::CollisionResultf result;
                fcl::Transform3f tf;
                tf.translation() = fcl::Vector3f(tfms[l - 1].translation().x(), tfms[l - 1].translation().y(), tfms[l - 1].translation().z());
                tf.linear() = tfms[l - 1].linear();
                fcl::collide(fclLink.get(), tf, fclScene.get(), fcl::Transform3f::Identity(), request, result);
                if (!result.isCollision()) continue;

                BVNode_soa<> linkBVH = BVH_n_ary_hierarchy_from_mesh<>(path.c_str(), 2);
                const Eigen::Matrix3f& lR = tfms[l - 1].linear();
                const Eigen::Vector3f& lT = tfms[l - 1].translation();

                // FCL contacts for the HIT link: which triangle pair(s)?
                {
                    fcl::CollisionRequestf creq;
                    creq.num_max_contacts = 8;
                    creq.enable_contact = true;
                    creq.enable_cost = false;
                    fcl::CollisionResultf cres;
                    fcl::Transform3f ctf;
                    ctf.translation() = fcl::Vector3f(lT.x(), lT.y(), lT.z());
                    ctf.linear() = lR;
                    fcl::collide(fclLink.get(), ctf, fclScene.get(), fcl::Transform3f::Identity(), creq, cres);
                    std::cout << "  [fcl contacts] link " << l << ": " << cres.numContacts() << " contacts\n";
                    for (size_t ci = 0; ci < cres.numContacts(); ++ci) {
                        const auto& c = cres.getContact(ci);
                        std::cout << "    contact: pos=(" << c.pos[0] << "," << c.pos[1] << "," << c.pos[2]
                                  << ") pen=" << c.penetration_depth
                                  << " b1=" << c.b1 << " b2=" << c.b2 << "\n";
                        if (c.b1 >= 0 && c.b1 < (int)tris.size() && c.b2 >= 0 && c.b2 < (int)sd.mesh.triangles.size()) {
                            const Triangle& rt = tris[c.b1];
                            const Triangle& ot = sd.mesh.triangles[c.b2];
                            const Eigen::Vector3f r0 = lR * verts[rt.v1] + lT;
                            const Eigen::Vector3f r1 = lR * verts[rt.v2] + lT;
                            const Eigen::Vector3f r2 = lR * verts[rt.v3] + lT;
                            const Eigen::Vector3f o0 = sd.mesh.vertices[ot.v1];
                            const Eigen::Vector3f o1 = sd.mesh.vertices[ot.v2];
                            const Eigen::Vector3f o2 = sd.mesh.vertices[ot.v3];
                            std::cout << "      rob " << c.b1 << ": (" << r0[0] << "," << r0[1] << "," << r0[2]
                                      << ") (" << r1[0] << "," << r1[1] << "," << r1[2]
                                      << ") (" << r2[0] << "," << r2[1] << "," << r2[2] << ")\n";
                            std::cout << "      obs " << c.b2 << ": (" << o0[0] << "," << o0[1] << "," << o0[2]
                                      << ") (" << o1[0] << "," << o1[1] << "," << o1[2]
                                      << ") (" << o2[0] << "," << o2[1] << "," << o2[2] << ")\n";
                            std::cout << "      triValidHost=" << (triValidHost(r0, r1, r2, o0, o1, o2) ? "free" : "COLLIDE") << "\n";
                        }
                    }
                }

                // world-space AABB of this link (cheap prefilter)
                Eigen::Vector3f linkAabbMin(FLT_MAX, FLT_MAX, FLT_MAX);
                Eigen::Vector3f linkAabbMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);
                for (const auto& v : verts) {
                    const Eigen::Vector3f w = lR * v + lT;
                    linkAabbMin = linkAabbMin.cwiseMin(w);
                    linkAabbMax = linkAabbMax.cwiseMax(w);
                }

                int overlapCount = 0, collideCount = 0;
                std::cout << "  [host replica] link " << l << " (FCL HIT): leaf pairs reaching tri test:\n";
                for (int obsTri = 0; obsTri < (int)sd.mesh.triangles.size(); ++obsTri) {
                    const Triangle& ot = sd.mesh.triangles[obsTri];
                    const Eigen::Vector3f o0 = sd.mesh.vertices[ot.v1];
                    const Eigen::Vector3f o1 = sd.mesh.vertices[ot.v2];
                    const Eigen::Vector3f o2 = sd.mesh.vertices[ot.v3];
                    // cheap AABB prefilter against the link's world AABB
                    Eigen::Vector3f omin = o0.cwiseMin(o1).cwiseMin(o2);
                    Eigen::Vector3f omax = o0.cwiseMax(o1).cwiseMax(o2);
                    if (omax.x() < linkAabbMin.x() || omin.x() > linkAabbMax.x() ||
                        omax.y() < linkAabbMin.y() || omin.y() > linkAabbMax.y() ||
                        omax.z() < linkAabbMin.z() || omin.z() > linkAabbMax.z()) continue;
                    for (int robTri = 0; robTri < (int)tris.size(); ++robTri) {
                        const Triangle& rt = tris[robTri];
                        Eigen::Vector3f r0 = lR * verts[rt.v1] + lT;
                        Eigen::Vector3f r1 = lR * verts[rt.v2] + lT;
                        Eigen::Vector3f r2 = lR * verts[rt.v3] + lT;
                        const bool tv = triValidHost(r0, r1, r2, o0, o1, o2);
                        ++overlapCount;
                        if (!tv) {
                            ++collideCount;
                            if (collideCount <= 8) {
                                std::cout << "    rob_tri=" << robTri << " obs_tri=" << obsTri
                                          << " tri_valid=COLLIDE\n";
                            }
                        }
                    }
                }
                std::cout << "  [host replica] link " << l << ": " << overlapCount
                          << " candidate pairs, " << collideCount << " collide per triangles_valid\n";
                break;
            }
        }
        return 0;
    }


    // --- FCL ground truth ---
    std::vector<unsigned char> fclResult; // 1 = collision
    std::vector<MeshPtr> fclLinks;
    std::vector<MeshPtr> fclObstacles;
    std::vector<std::vector<Eigen::Vector3f>> linkVerts;
    std::vector<std::vector<Triangle>> linkTris;

    auto buildFCLModels = [&]() {
        if (!fclLinks.empty()) {
            return;
        }
        // link meshes (links 1..7; skip base like RTCD SKIP_BASE)
        for (int i = 1; i <= 7; ++i) {
            std::string path = meshDir + "/panda_link" + std::to_string(i) + "_visual.obj";
            std::vector<Eigen::Vector3f> verts;
            std::vector<Triangle> tris;
            loadOBJFile(path, verts, tris);
            if (verts.empty()) { std::cerr << "Failed to load " << path << std::endl; exit(1); }
            std::cout << "  link " << i << ": " << tris.size() << " tris (" << path << ")" << std::endl;
            linkVerts.push_back(verts);
            linkTris.push_back(tris);
            fclLinks.push_back(makeFCLMesh(verts, tris));
        }
        // obstacles (world-frame transforms applied at load)
        for (const auto& ob : sd.obstacles) {
            std::string path = sceneDir + "/" + ob.first;
            std::vector<Eigen::Vector3f> verts;
            std::vector<Triangle> tris;
            loadOBJFile(path, verts, tris);
            for (auto& v : verts) v += ob.second;
            fclObstacles.push_back(makeFCLMesh(verts, tris));
        }
    };

    if (doFCL && verifyFile.empty()) {
        buildFCLModels();

        PandaFK fk;
        fclResult.assign(nPoses, 0);
        size_t ncol = 0;
        const auto fclT0 = std::chrono::high_resolution_clock::now();
        for (size_t i = 0; i < nPoses; ++i) {
            std::array<Eigen::Isometry3f, 7> tfms;
            fk.compute(traj[i], tfms);
            bool c = poseInCollisionFCL(tfms, fclLinks, fclObstacles, true);
            fclResult[i] = c ? 1 : 0;
            ncol += c ? 1 : 0;
        }
        const auto fclT1 = std::chrono::high_resolution_clock::now();
        const double fclMs = std::chrono::duration<double, std::milli>(fclT1 - fclT0).count();
        std::cout << "FCL ground truth: " << ncol << "/" << nPoses << " poses in collision, "
                  << fclMs << " ms -> " << fclMs / nPoses * 1000.0 << " us/pose" << std::endl;

        // also compute full-pair FCL (no early exit) to double-check semantics
        size_t ncolFull = 0, mism = 0;
        for (size_t i = 0; i < nPoses; ++i) {
            std::array<Eigen::Isometry3f, 7> tfms;
            fk.compute(traj[i], tfms);
            bool c = poseInCollisionFCL(tfms, fclLinks, fclObstacles, false);
            ncolFull += c ? 1 : 0;
            if ((c ? 1 : 0) != fclResult[i]) ++mism;
        }
        std::cout << "FCL full-pair: " << ncolFull << "/" << nPoses << " in collision, "
                  << mism << " differ vs early-exit (should be 0)" << std::endl;
        if (mism != 0) {
            std::cerr << "FCL early-exit vs full-pair mismatch -- cannot trust ground truth" << std::endl;
            return 1;
        }

        if (!labelsFile.empty()) {
            std::ofstream lf(labelsFile, std::ios::binary);
            if (!lf.is_open()) {
                std::cerr << "Cannot open labels file: " << labelsFile << std::endl;
                return 1;
            }
            lf.write(reinterpret_cast<const char*>(fclResult.data()), (std::streamsize)fclResult.size());
            lf.close();
            std::cout << "Wrote " << nPoses << " FCL ground-truth labels to " << labelsFile << std::endl;
        }
    }

    // --- FCL verification-only mode: double-check the poses flagged by a GPU
    // checker (pipeline metric). Reads one pose index per line, runs FCL on
    // just those, and reports how many were false collisions + the time. ---
    if (!verifyFile.empty()) {
        std::ifstream vf(verifyFile);
        if (!vf.is_open()) {
            std::cerr << "Cannot open verify file: " << verifyFile << std::endl;
            return 1;
        }
        std::vector<size_t> vIdx;
        size_t x;
        while (vf >> x) {
            if (x < nPoses) vIdx.push_back(x);
        }
        vf.close();

        buildFCLModels();

        PandaFK fk;
        size_t nTrue = 0, nFalse = 0;
        const auto t0 = std::chrono::high_resolution_clock::now();
        for (size_t i : vIdx) {
            std::array<Eigen::Isometry3f, 7> tfms;
            fk.compute(traj[i], tfms);
            if (poseInCollisionFCL(tfms, fclLinks, fclObstacles, true)) {
                ++nTrue;
            } else {
                ++nFalse;
            }
        }
        const auto t1 = std::chrono::high_resolution_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        const double usPerCheck = vIdx.empty() ? 0.0 : ms * 1000.0 / (double)vIdx.size();
        std::cout << "FCL VERIFY: checked=" << vIdx.size() << " true=" << nTrue
                  << " false=" << nFalse << " time_ms=" << ms
                  << " us_per_check=" << usPerCheck << std::endl;
        return 0;
    }

    // --- articulated edge checking (GPU vs FCL), then exit ---------------
    if (edgeCheckN > 0) {
        if (edgeCheckN > traj.size() - 1) edgeCheckN = traj.size() - 1;
        buildFCLModels();

        std::vector<ArticulatedEdge<7>> edges(edgeCheckN);
        for (size_t i = 0; i < edgeCheckN; ++i) {
            articulated_conf<7> s7, e7;
            for (int j = 0; j < 7; ++j) {
                s7[j] = traj[i][j];
                e7[j] = traj[i + 1][j];
            }
            edges[i] = makeArticulatedEdge<7>(s7, e7, edgeLvs);
            edges[i].flags = EDGE_FLAG_REPORT_FIRST_INVALID_T;
        }
        ArticulatedEdgeBatch<7> batch(edges);
        size_t total_samples = 0;
        for (const auto& e : edges) total_samples += (size_t)e.nsteps + 1;
        std::cout << "Articulated edge validation: " << edgeCheckN << " edges, "
                  << total_samples << " samples (lvs=" << edgeLvs << " rad)." << std::endl;

        std::vector<bool> gpuValid;
        std::vector<float> gpuFirstT;
        const double kernelMs = bvh_edges_articulated<7>(urdfPath, sd.bvh, sd.mesh, batch,
                                                         gpuValid, gpuFirstT, true, edgeKmode);

        // FCL ground truth per sample (FK + per-link collide, early exit).
        PandaFK fk;
        std::vector<bool> cpuValid(edgeCheckN, true);
        std::vector<float> cpuFirstT(edgeCheckN, -1.0f);
        const auto cpuT0 = std::chrono::high_resolution_clock::now();
        size_t cpuSamples = 0;
        for (size_t i = 0; i < edgeCheckN; ++i) {
            const uint32_t n = edges[i].nsteps;
            for (uint32_t j = 0; j <= n; ++j) {
                const float t = (n > 0) ? (float)j / (float)n : 0.0f;
                std::array<float, 7> q;
                for (int k = 0; k < 7; ++k) q[k] = traj[i][k] + (traj[i + 1][k] - traj[i][k]) * t;
                std::array<Eigen::Isometry3f, 7> tfms;
                fk.compute(q, tfms);
                ++cpuSamples;
                if (poseInCollisionFCL(tfms, fclLinks, fclObstacles, true)) {
                    cpuValid[i] = false;
                    cpuFirstT[i] = t;
                    break;
                }
            }
        }
        const auto cpuT1 = std::chrono::high_resolution_clock::now();
        const double cpuMs = std::chrono::duration<double, std::milli>(cpuT1 - cpuT0).count();

        size_t tp = 0, tn = 0, fp = 0, fn = 0;
        for (size_t i = 0; i < edgeCheckN; ++i) {
            const bool g = gpuValid[i];
            const bool c = cpuValid[i];
            if (g && c) ++tp;
            else if (!g && !c) ++tn;
            else if (g && !c) ++fp;
            else ++fn;
            if ((g != c) && (fp + fn) <= 10) {
                std::cout << "  MISMATCH edge " << i << ": GPU=" << (g ? "valid" : "COLLIDE")
                          << " CPU=" << (c ? "valid" : "COLLIDE")
                          << " firstT gpu=" << gpuFirstT[i] << " cpu=" << cpuFirstT[i] << std::endl;
            }
        }
        std::cout << "EDGE CHECK (" << edgeCheckN << " edges): TP=" << tp << " TN=" << tn
                  << " FP=" << fp << " FN=" << fn << std::endl;
        std::cout << "EDGE TIMING: GPU kernel " << kernelMs << " ms ("
                  << kernelMs * 1000.0 / edgeCheckN << " us/edge), CPU " << cpuMs
                  << " ms (" << cpuMs * 1000.0 / edgeCheckN << " us/edge, "
                  << cpuSamples << " samples)" << std::endl;
        if (fp != 0 || fn != 0) {
            std::cerr << "MISMATCH in articulated edge checking vs FCL" << std::endl;
            return 1;
        }
        return 0;
    }

    // --- GPU articulated run ---
    std::vector<articulated_conf<7>> confs(nPoses);
    for (size_t i = 0; i < nPoses; ++i) {
        for (int j = 0; j < 7; ++j) confs[i][j] = traj[i][j];
    }

    // A/B/C/BC kernel variants (node-layout experiment):
    //   0 = base (matrix SoA)        4 = quat R
    //   8 = vecR (padded float4 R)  12 = quat R + packed T/dim/a
    // gate sweep: quatSAT (64) x gate mode in kernel_mode bits 7-8
    //   (0 = obs root, 1 = lv1 cut, 2 = lv2 cut, 3 = per-obstacle roots)
    static int kVariantMode[4];
    static const char* kVariantName[4];
    if (gateSweep) {
        kVariantMode[0] = 64;
        kVariantMode[1] = 64 + (1 << 7);
        kVariantMode[2] = 64 + (2 << 7);
        kVariantMode[3] = 64 + (3 << 7);
        kVariantName[0] = "quatSAT-G0";
        kVariantName[1] = "quatSAT-G1";
        kVariantName[2] = "quatSAT-G2";
        kVariantName[3] = "quatSAT-Gobs";
    } else {
        kVariantMode[0] = 0;
        kVariantMode[1] = 4;
        kVariantMode[2] = 8;
        kVariantMode[3] = 12;
        kVariantName[0] = "base";
        kVariantName[1] = "quat";
        kVariantName[2] = "vecR";
        kVariantName[3] = "quat+TD";
    }
    const int nCells = (kernelModeOverride >= 0) ? 1 : 4;

    auto runGpu = [&](size_t n, int kmode, std::vector<bool>* outValid) -> double {
        std::vector<articulated_conf<7>> sub(confs.begin(), confs.begin() + n);
        std::vector<bool> valid;
        const int gateMode = (kmode >> 7) & 3;
        const std::vector<Eigen::Matrix3f>* gR = (gateMode == 0) ? nullptr : &gateR[gateMode];
        const std::vector<Eigen::Vector3f>* gT = (gateMode == 0) ? nullptr : &gateT[gateMode];
        const std::vector<Eigen::Vector3f>* gD = (gateMode == 0) ? nullptr : &gateD[gateMode];
        const double kernelMs = bvh_articulated<7>(urdfPath, sd.bvh, sd.mesh, sub, valid, true, kmode, gR, gT, gD);
#ifdef RTCD_PROF
        articulatedProfDump();
        articulatedObbDump();
#endif
        if (doFCL && n == nPoses) {
            size_t fp = 0, fn = 0, tp = 0, tn = 0;
            for (size_t i = 0; i < n; ++i) {
                const bool gpuCollision = !valid[i];
                const bool fclCollision  = fclResult[i] != 0;
                if (gpuCollision && fclCollision) ++tp;
                else if (!gpuCollision && !fclCollision) ++tn;
                else if (gpuCollision && !fclCollision) {
                    ++fp;
                    if (fp <= 10) {
                        std::cout << "  FP at pose " << i << ": q =";
                        for (int j = 0; j < 7; ++j) std::cout << " " << traj[i][j];
                        std::cout << std::endl;
                    }
                } else {
                    ++fn;
                    if (fn <= 10) {
                        std::cout << "  FN at pose " << i << ": q =";
                        for (int j = 0; j < 7; ++j) std::cout << " " << traj[i][j];
                        std::cout << std::endl;
                    }
                }
            }
            const char* kName = (kmode == 0) ? "base" : (kmode == 4) ? "quat" : (kmode == 8) ? "vecR" : (kmode == 12) ? "quat+TD" : (kmode == 32) ? "1bsm" : (kmode == 16) ? "2bsm" : (kmode == 64) ? "quatSAT" : (kmode == 192) ? "quatSAT-G1" : (kmode == 320) ? "quatSAT-G2" : (kmode == 448) ? "quatSAT-Gobs" : "mode?";
        std::cout << "FP/FN check [" << kName << "] (" << n << " poses): TP=" << tp
                      << " TN=" << tn << " FP=" << fp << " FN=" << fn << std::endl;
            if (fp != 0 || fn != 0) {
                std::cerr << "MISMATCH vs FCL ground truth (FP=" << fp << " FN=" << fn << ")" << std::endl;
                exit(1);
            }
        }
        if (outValid) {
            *outValid = std::move(valid);
        }
        return kernelMs;
    };

    std::ofstream csv;
    if (!csvFile.empty()) {
        std::ifstream probe(csvFile);
        const bool empty = !probe.good() || probe.peek() == std::ifstream::traits_type::eof();
        probe.close();
        csv.open(csvFile, std::ios::app);
        if (empty) {
            csv << "scene,algorithm,batch,poses,kernel_ms,us_per_pose\n";
        }
    }

    double best = 1e30;
    std::vector<size_t> batchSizes;
    if (sweep) {
        // 1 pose doubling up to nPoses (capped: never time more poses than exist)
        for (size_t n = 1; n <= 4096 && n <= nPoses; n <<= 1) batchSizes.push_back(n);
        if (batchSizes.back() != nPoses) batchSizes.push_back(nPoses);
    } else {
        batchSizes.push_back(nPoses);
    }

    for (size_t batch : batchSizes) {
        double totalMs[4] = {0, 0, 0, 0};
        double bestMs[4] = {1e30, 1e30, 1e30, 1e30};
        bool agree = true;
        for (int r = 0; r < repeat; ++r) {
            std::vector<bool> validRef, vq;
            // Alternate the run order per repeat so clock/thermal drift does
            // not systematically favor one variant.
            const bool rev = (r & 1) != 0;
            for (int i = 0; i < nCells; ++i) {
                const int m = (kernelModeOverride >= 0) ? kernelModeOverride
                              : (rev ? kVariantMode[3 - i] : kVariantMode[i]);
                std::vector<bool> v;
                const double ms = runGpu(batch, m, &v);
                totalMs[i] += ms;
                bestMs[i] = std::min(bestMs[i], ms);
                best = std::min(best, ms);
                if (validRef.empty()) {
                    validRef = v;
                } else if (validRef != v) {
                    agree = false;
                }
            }
        }
        for (int i = 0; i < nCells; ++i) {
            const double avgMs = totalMs[i] / repeat;
            const double usPerPose = avgMs * 1000.0 / (double)batch;
            // Name the cell after the mode it actually ran (with --kernel-mode
            // the cell mode differs from the default variant list).
            const int cellMode = (kernelModeOverride >= 0) ? kernelModeOverride : kVariantMode[i];
            const char* kName = (cellMode == 0) ? "base" : (cellMode == 4) ? "quat"
                              : (cellMode == 8) ? "vecR" : (cellMode == 12) ? "quat+TD"
                              : (cellMode == 32) ? "1bsm" : (cellMode == 16) ? "2bsm"
                              : (cellMode == 64) ? "quatSAT" : (cellMode == 192) ? "quatSAT-G1"
                              : (cellMode == 320) ? "quatSAT-G2" : (cellMode == 448) ? "quatSAT-Gobs"
                              : kVariantName[i];
            std::cout << "BATCH " << batch << " [" << kName << "]: avg kernel " << avgMs
                      << " ms -> " << usPerPose << " us/pose (best " << bestMs[i] * 1000.0 / batch
                      << " us/pose)" << std::endl;
            if (csv.is_open()) {
                csv << scene << ",robo-check-bvh-" << kName << "," << batch << "," << nPoses
                    << "," << avgMs << "," << usPerPose << "\n";
            }
        }
        if (nCells == 4)
        std::cout << "BATCH " << batch << " [AB/BC]: best-kernel speedup vs " << kVariantName[0]
                  << ": " << kVariantName[1] << "=" << bestMs[0] / bestMs[1]
                  << "x " << kVariantName[2] << "=" << bestMs[0] / bestMs[2]
                  << "x " << kVariantName[3] << "=" << bestMs[0] / bestMs[3]
                  << "x, results " << (agree ? "identical" : "DIFFER!") << std::endl;
    }

    std::cout << "=== done ===" << std::endl;
    return 0;
}
