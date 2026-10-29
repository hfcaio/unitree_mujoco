#pragma once

// Model composition: attach component models (hands, arms, ...) to a robot body at load time.
//
// A robot YAML (simulate/robots/<robot>.yaml) may list `attachments`; each one names an MJCF
// file under unitree_robots/components/ and a mount point in the robot's scene:
//
//   attachments:
//     - type: brainco_revo2            # model = components/<type>/<side>.xml (or <type>/<type>.xml without side)
//       side: left                     # optional; default prefix is "<side>_"
//       mount: left_wrist_yaw_link     # body or site of the robot scene
//       pos: [0.0591, 0, 0]            # optional offset of the component's root body in the mount frame
//       rpy: [-1.5707963, 0, -1.5707963]  # optional, URDF convention (fixed axes X, then Y, then Z); or quat: [w, x, y, z]
//       remove_mesh_geoms: [left_rubber_hand]  # optional: delete the scene's geoms that use these meshes
//       mount_inertial: {mass: 0.08, pos: [...], fullinertia: [ixx, iyy, izz, ixy, ixz, iyz]}  # optional
//     - {model: brainco_revo2/g1_adapter_left.xml, mount: left_wrist_yaw_link, prefix: left_}
//
// The scene is parsed with mj_parseXML, each component file too, and the first body of the
// component's worldbody (with everything that refers to it: actuators, sensors, equalities,
// contact excludes, assets, defaults) is attached with mjs_attach under `prefix` + name + `suffix`,
// then the whole spec is compiled. Element names in the result are therefore predictable:
// actuator "index" of components/brainco_revo2/left.xml becomes "left_index".
// Robots without attachments never come here: they are still loaded with mj_loadXML.

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

struct AttachmentSpec
{
    std::filesystem::path model;           // component MJCF file (absolute after LoadRobotDef)
    std::string mount;                     // body or site name in the robot scene
    std::string prefix, suffix;            // added to every name of the attached elements
    std::array<double, 3> pos{0, 0, 0};    // root body offset in the mount frame
    std::array<double, 4> quat{1, 0, 0, 0};
    std::vector<std::string> remove_mesh_geoms;
    YAML::Node mount_inertial;             // optional replacement inertial of the mount body
};

// Parse one `attachments` entry. Relative model paths are resolved in `components_dir`.
inline AttachmentSpec ParseAttachment(const YAML::Node &n, const std::filesystem::path &components_dir)
{
    AttachmentSpec a;
    std::string side = n["side"] ? n["side"].as<std::string>() : "";
    std::filesystem::path model;
    if (n["model"]) {
        model = n["model"].as<std::string>();
    } else if (n["type"]) {
        auto type = n["type"].as<std::string>();
        model = std::filesystem::path(type) / ((side.empty() ? type : side) + ".xml");
    } else {
        throw std::invalid_argument("attachments: each entry needs `type` or `model`");
    }
    a.model = model.is_absolute() ? model : components_dir / model;
    if (!std::filesystem::exists(a.model)) {
        throw std::invalid_argument("attachments: model file " + a.model.string() + " not found");
    }
    if (!n["mount"]) {
        throw std::invalid_argument("attachments: " + a.model.string() + " needs a `mount` (body or site name)");
    }
    a.mount = n["mount"].as<std::string>();
    a.prefix = n["prefix"] ? n["prefix"].as<std::string>() : (side.empty() ? "" : side + "_");
    a.suffix = n["suffix"] ? n["suffix"].as<std::string>() : "";
    if (n["pos"]) {
        a.pos = n["pos"].as<std::array<double, 3>>();
    }
    if (n["quat"]) {
        a.quat = n["quat"].as<std::array<double, 4>>();
    } else if (n["rpy"]) {
        // URDF rpy: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        auto rpy = n["rpy"].as<std::array<double, 3>>();
        double qx[4] = {std::cos(rpy[0] / 2), std::sin(rpy[0] / 2), 0, 0};
        double qy[4] = {std::cos(rpy[1] / 2), 0, std::sin(rpy[1] / 2), 0};
        double qz[4] = {std::cos(rpy[2] / 2), 0, 0, std::sin(rpy[2] / 2)};
        double qzy[4];
        mju_mulQuat(qzy, qz, qy);
        mju_mulQuat(a.quat.data(), qzy, qx);
    }
    if (n["remove_mesh_geoms"]) {
        a.remove_mesh_geoms = n["remove_mesh_geoms"].as<std::vector<std::string>>();
    }
    if (n["mount_inertial"]) {
        a.mount_inertial.reset(n["mount_inertial"]);
    }
    return a;
}

namespace robot_compose_detail
{
// Owns a spec for the duration of ComposeModel.
struct SpecPtr
{
    mjSpec *s = nullptr;
    SpecPtr() = default;
    SpecPtr(const SpecPtr &) = delete;
    SpecPtr &operator=(const SpecPtr &) = delete;
    ~SpecPtr() { if (s) mj_deleteSpec(s); }
};

inline void Fail(char *error, int error_sz, const std::string &msg)
{
    if (error && error_sz > 0) {
        std::snprintf(error, error_sz, "%s", msg.c_str());
    }
}

} // namespace robot_compose_detail

// Parse `scene`, attach every component and compile. Returns nullptr and fills `error` on failure.
inline mjModel *ComposeModel(const char *scene, const std::vector<AttachmentSpec> &attachments,
                             char *error, int error_sz)
{
    using namespace robot_compose_detail;
    // Declared first so they are deleted last: the parent spec refers to its attached children.
    std::vector<SpecPtr> children(attachments.size());
    SpecPtr parent;
    parent.s = mj_parseXML(scene, nullptr, error, error_sz);
    if (!parent.s) {
        return nullptr;
    }

    for (size_t i = 0; i < attachments.size(); i++) {
        const auto &a = attachments[i];
        std::string what = "attachment " + a.model.string() + ": ";

        // Remove placeholder geometry of the robot body (e.g. the G1's rubber hands).
        for (const auto &mesh : a.remove_mesh_geoms) {
            bool found = false;
            for (mjsElement *el = mjs_firstElement(parent.s, mjOBJ_GEOM); el;) {
                mjsElement *next = mjs_nextElement(parent.s, el);
                mjsGeom *g = mjs_asGeom(el);
                const char *mn = mjs_getString(g->meshname);
                if (mn && mesh == mn) {
                    mjs_delete(parent.s, el);
                    found = true;
                }
                el = next;
            }
            if (!found) {
                Fail(error, error_sz, what + "no geom uses mesh '" + mesh + "' (remove_mesh_geoms)");
                return nullptr;
            }
        }

        // Mount point: a site or a body of the robot scene.
        mjsElement *mount = mjs_findElement(parent.s, mjOBJ_SITE, a.mount.c_str());
        mjsBody *mount_body = mount ? mjs_getParent(mount) : mjs_findBody(parent.s, a.mount.c_str());
        if (!mount && mount_body) {
            // mjs_attach puts a body under a frame or a site, not directly under a body
            mount = mjs_addFrame(mount_body, nullptr)->element;
        }
        if (!mount) {
            Fail(error, error_sz, what + "no body or site named '" + a.mount + "' in " + scene);
            return nullptr;
        }

        if (a.mount_inertial.IsMap()) {
            const auto &in = a.mount_inertial;
            mount_body->mass = in["mass"].as<double>();
            auto ipos = in["pos"] ? in["pos"].as<std::array<double, 3>>() : std::array<double, 3>{0, 0, 0};
            auto fi = in["fullinertia"].as<std::array<double, 6>>();
            for (int k = 0; k < 3; k++) mount_body->ipos[k] = ipos[k];
            // principal axes (mjsBody.fullinertia cannot replace an inertia already given in the XML)
            mjtNum mat[9] = {fi[0], fi[3], fi[4], fi[3], fi[1], fi[5], fi[4], fi[5], fi[2]}, vec[9];
            mju_eig3(mount_body->inertia, vec, mount_body->iquat, mat);
            mount_body->explicitinertial = 1;
        }

        children[i].s = mj_parseXML(a.model.c_str(), nullptr, error, error_sz);
        if (!children[i].s) {
            return nullptr;
        }
        mjsBody *root = mjs_asBody(mjs_firstChild(mjs_findBody(children[i].s, "world"), mjOBJ_BODY, 0));
        if (!root) {
            Fail(error, error_sz, what + "the model has no body to attach");
            return nullptr;
        }
        // Offset of the component in the mount frame, composed with the root body's own pose.
        double pos[3], quat[4];
        mju_rotVecQuat(pos, root->pos, a.quat.data());
        for (int k = 0; k < 3; k++) pos[k] += a.pos[k];
        mju_mulQuat(quat, a.quat.data(), root->quat);
        mju_copy3(root->pos, pos);
        mju_copy4(root->quat, quat);

        if (!mjs_attach(mount, root->element, a.prefix.c_str(), a.suffix.c_str())) {
            Fail(error, error_sz, what + "mjs_attach failed: " + mjs_getError(parent.s));
            return nullptr;
        }
    }

    mjModel *m = mj_compile(parent.s, nullptr);
    if (!m) {
        Fail(error, error_sz, std::string("compose: ") + mjs_getError(parent.s));
    } else if (mjs_isWarning(parent.s)) {
        Fail(error, error_sz, mjs_getError(parent.s));
    } else if (error && error_sz > 0) {
        error[0] = '\0';
    }
    return m;
}
