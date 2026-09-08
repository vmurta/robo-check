#include "ArticulatedRobot.hu"

#include <fstream>
#include <map>
#include <regex>
#include <set>
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

bool parseSerialChainURDF(const std::string& urdf_path, ArticulatedRobot& robot) {
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

    if (raw_joints.empty()) {
        std::cerr << "No joints found in URDF: " << urdf_path << std::endl;
        return false;
    }

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
        link_meshes[name] = mesh;
    }

    std::set<std::string> parent_names;
    std::set<std::string> child_names;
    for (const auto& j : raw_joints) {
        parent_names.insert(j.parent);
        child_names.insert(j.child);
    }

    std::string root;
    for (const auto& p : parent_names) {
        if (child_names.find(p) == child_names.end()) {
            root = p;
            break;
        }
    }
    if (root.empty()) {
        std::cerr << "Could not find a root link (a link that is never a child)." << std::endl;
        return false;
    }

    std::vector<std::string> ordered_links;
    std::vector<JointParams> ordered_joints;

    ordered_links.push_back(root);
    std::string current = root;
    for (size_t i = 0; i < raw_joints.size(); ++i) {
        int found = -1;
        int matches = 0;
        for (size_t k = 0; k < raw_joints.size(); ++k) {
            if (raw_joints[k].parent == current) {
                found = static_cast<int>(k);
                matches++;
            }
        }
        if (found < 0) {
            std::cerr << "Joint chain is broken at link '" << current << "'." << std::endl;
            return false;
        }
        if (matches > 1) {
            std::cerr << "Link '" << current << "' has multiple child joints; expected a serial chain." << std::endl;
            return false;
        }

        JointParams jp;
        jp.origin_R = eulerRPYToRotation(raw_joints[found].origin_rpy.x(), raw_joints[found].origin_rpy.y(),
                                         raw_joints[found].origin_rpy.z());
        jp.origin_T = raw_joints[found].origin_xyz;
        jp.axis = raw_joints[found].axis;
        ordered_joints.push_back(jp);

        current = raw_joints[found].child;
        ordered_links.push_back(current);
    }

    robot.num_joints = raw_joints.size();
    robot.joints = ordered_joints;
    robot.link_names = ordered_links;
    robot.link_meshes.clear();
    robot.link_meshes.reserve(ordered_links.size());
    for (const auto& name : ordered_links) {
        auto it = link_meshes.find(name);
        robot.link_meshes.push_back(it != link_meshes.end() ? it->second : std::string());
    }

    return true;
}
