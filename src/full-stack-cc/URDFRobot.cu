#include "URDFRobot.hu"

#include <fstream>
#include <map>
#include <queue>
#include <regex>
#include <sstream>

namespace {

struct RawJoint {
    std::string name;
    std::string type;
    std::string parent;
    std::string child;
    Eigen::Vector3f origin_xyz = Eigen::Vector3f::Zero();
    Eigen::Vector3f origin_rpy = Eigen::Vector3f::Zero();
    Eigen::Vector3f axis = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
};

std::string attrOf(const std::string& tag, const std::string& attr) {
    std::regex re(attr + "\\s*=\\s*\"([^\"]*)\"");
    std::smatch m;
    if (std::regex_search(tag, m, re)) {
        return m[1].str();
    }
    return "";
}

std::string firstMatch(const std::string& s, const std::regex& re) {
    std::smatch m;
    if (std::regex_search(s, m, re)) {
        return m.str();
    }
    return "";
}

bool parseVec3(const std::string& s, Eigen::Vector3f& out) {
    if (s.empty()) {
        return false;
    }
    std::istringstream iss(s);
    float x, y, z;
    if (!(iss >> x >> y >> z)) {
        return false;
    }
    out = Eigen::Vector3f(x, y, z);
    return true;
}

Eigen::Matrix3f eulerRPYToRotation(float roll, float pitch, float yaw) {
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    Eigen::Matrix3f R;
    R(0, 0) = cy * cp;
    R(0, 1) = cy * sp * sr - sy * cr;
    R(0, 2) = cy * sp * cr + sy * sr;
    R(1, 0) = sy * cp;
    R(1, 1) = sy * sp * sr + cy * cr;
    R(1, 2) = sy * sp * cr - cy * sr;
    R(2, 0) = -sp;
    R(2, 1) = cp * sr;
    R(2, 2) = cp * cr;
    return R;
}

} // namespace

bool parseFullURDF(const std::string& urdf_path, URDFRobot& robot) {
    std::ifstream file(urdf_path);
    if (!file.is_open()) {
        std::cerr << "Error opening URDF file: " << urdf_path << std::endl;
        return false;
    }

    std::stringstream ss;
    ss << file.rdbuf();
    std::string xml = ss.str();

    std::vector<RawJoint> raw_joints;
    std::regex joint_re(R"(<joint\b[^>]*>[\s\S]*?</joint>)", std::regex::icase);
    for (std::sregex_iterator it(xml.begin(), xml.end(), joint_re), end; it != end; ++it) {
        std::string block = it->str();
        RawJoint j;

        std::string open = firstMatch(block, std::regex(R"(<joint\b[^>]*>)", std::regex::icase));
        j.name = attrOf(open, "name");
        j.type = attrOf(open, "type");

        std::string origin = firstMatch(block, std::regex(R"(<origin\b[^>]*>)", std::regex::icase));
        Eigen::Vector3f xyz;
        if (parseVec3(attrOf(origin, "xyz"), xyz)) {
            j.origin_xyz = xyz;
        }
        Eigen::Vector3f rpy;
        if (parseVec3(attrOf(origin, "rpy"), rpy)) {
            j.origin_rpy = rpy;
        }

        std::string parent = firstMatch(block, std::regex(R"(<parent\b[^>]*>)", std::regex::icase));
        j.parent = attrOf(parent, "link");

        std::string child = firstMatch(block, std::regex(R"(<child\b[^>]*>)", std::regex::icase));
        j.child = attrOf(child, "link");

        std::string axis = firstMatch(block, std::regex(R"(<axis\b[^>]*>)", std::regex::icase));
        Eigen::Vector3f ax;
        if (parseVec3(attrOf(axis, "xyz"), ax)) {
            j.axis = ax;
        }

        if (j.parent.empty() || j.child.empty()) {
            std::cerr << "Joint '" << j.name << "' is missing a parent or child link." << std::endl;
            return false;
        }
        raw_joints.push_back(j);
    }

    std::vector<std::string> link_names;
    std::map<std::string, std::string> link_meshes;
    std::regex link_re(R"(<link\b[^>]*>[\s\S]*?</link>)", std::regex::icase);
    for (std::sregex_iterator it(xml.begin(), xml.end(), link_re), end; it != end; ++it) {
        std::string block = it->str();
        std::string open = firstMatch(block, std::regex(R"(<link\b[^>]*>)", std::regex::icase));
        std::string name = attrOf(open, "name");

        std::string mesh;
        std::string mesh_tag = firstMatch(block, std::regex(R"(<mesh\b[^>]*>)", std::regex::icase));
        if (!mesh_tag.empty()) {
            mesh = attrOf(mesh_tag, "filename");
        }

        link_names.push_back(name);
        link_meshes[name] = mesh;
    }

    if (link_names.empty()) {
        std::cerr << "No links found in URDF: " << urdf_path << std::endl;
        return false;
    }

    std::map<std::string, int> link_index;
    for (size_t i = 0; i < link_names.size(); ++i) {
        link_index[link_names[i]] = static_cast<int>(i);
    }

    int num_links = static_cast<int>(link_names.size());

    std::vector<int> movable_angle_idx;
    for (const auto& j : raw_joints) {
        if (j.type == "revolute" || j.type == "continuous") {
            movable_angle_idx.push_back(static_cast<int>(movable_angle_idx.size()));
        } else {
            movable_angle_idx.push_back(-1);
        }
    }

    std::vector<int> link_parent(num_links, -1);
    std::vector<JointParams> joint_origin(num_links);
    std::vector<Eigen::Vector3f> joint_axis(num_links, Eigen::Vector3f(1.0f, 0.0f, 0.0f));
    std::vector<int> joint_angle_idx(num_links, -1);
    std::vector<std::string> joint_names;

    int assigned = 0;
    for (size_t k = 0; k < raw_joints.size(); ++k) {
        const RawJoint& j = raw_joints[k];

        auto pit = link_index.find(j.parent);
        auto cit = link_index.find(j.child);
        if (pit == link_index.end() || cit == link_index.end()) {
            std::cerr << "Joint '" << j.name << "' references an undefined link." << std::endl;
            return false;
        }

        int p = pit->second;
        int c = cit->second;
        if (link_parent[c] != -1) {
            std::cerr << "Link '" << j.child << "' has more than one parent joint." << std::endl;
            return false;
        }

        link_parent[c] = p;
        joint_origin[c].origin_R = eulerRPYToRotation(j.origin_rpy.x(), j.origin_rpy.y(), j.origin_rpy.z());
        joint_origin[c].origin_T = j.origin_xyz;
        joint_axis[c] = j.axis;

        if (movable_angle_idx[k] >= 0) {
            joint_angle_idx[c] = movable_angle_idx[k];
            joint_names.push_back(j.name);
        }
        assigned++;
    }

    int root = -1;
    for (int l = 0; l < num_links; ++l) {
        if (link_parent[l] == -1) {
            if (root != -1) {
                std::cerr << "URDF has multiple root links (disconnected tree)." << std::endl;
                return false;
            }
            root = l;
        }
    }
    if (root == -1) {
        std::cerr << "Could not find a root link." << std::endl;
        return false;
    }

    std::vector<std::vector<int>> children(num_links);
    for (int l = 0; l < num_links; ++l) {
        if (link_parent[l] != -1) {
            children[link_parent[l]].push_back(l);
        }
    }

    std::vector<int> link_order;
    std::queue<int> q;
    q.push(root);
    while (!q.empty()) {
        int l = q.front();
        q.pop();
        link_order.push_back(l);
        for (int c : children[l]) {
            q.push(c);
        }
    }

    if (static_cast<int>(link_order.size()) != num_links) {
        std::cerr << "URDF kinematic tree is not fully connected." << std::endl;
        return false;
    }

    robot.num_links = num_links;
    robot.num_joints = static_cast<int>(joint_names.size());
    robot.link_names = link_names;
    robot.link_meshes.resize(num_links);
    for (int l = 0; l < num_links; ++l) {
        auto it = link_meshes.find(link_names[l]);
        robot.link_meshes[l] = (it != link_meshes.end() ? it->second : std::string());
    }
    robot.link_parent = link_parent;
    robot.joint_origin = joint_origin;
    robot.joint_axis = joint_axis;
    robot.joint_angle_idx = joint_angle_idx;
    robot.link_order = link_order;
    robot.joint_names = joint_names;

    return true;
}
