#pragma once

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "bridge_component.h"
#include "param.h"
#include "robot_compose.h"

// Per-robot description loaded from simulate/robots/<robot>.yaml:
//
//   scene_dir: g1            # optional: -s scenes are looked up in unitree_robots/<scene_dir>
//                            # (default: unitree_robots/<robot>)
//   idl: hg                  # go | hg (optional; omitted = decide from the actuator count)
//   motors:                  # optional: DDS motor index -> MJCF actuator name.
//     12: waist_yaw          # Omitted = actuator i is DDS motor i (legacy layout).
//   components:              # bridge components, see components.h
//     - secondary_imu
//     - {type: waist_pitch, actuator: waist_pitch}   # a component with parameters
//   attachments:             # optional: component models attached to the scene, see robot_compose.h
//     - {type: brainco_revo2, side: left, mount: left_wrist_yaw_link, pos: [0.0591, 0, 0]}
struct ComponentSpec
{
    std::string type;
    YAML::Node params;
};

struct RobotDef
{
    bool from_file = false;
    std::string scene_dir;
    int idl_type = param::IDL_AUTO;
    std::map<int, std::string> motors;
    std::vector<ComponentSpec> components;
    std::vector<AttachmentSpec> attachments;

    // Number of DDS motor slots the model uses: the actuator count, unless `motors` maps them.
    int motorCount(int actuator_count) const
    {
        return motors.empty() ? actuator_count : motors.rbegin()->first + 1;
    }
};

// `components_dir` (unitree_robots/components) resolves relative attachment model paths.
inline RobotDef LoadRobotDef(const std::filesystem::path &dir, const std::string &robot,
                             const std::filesystem::path &components_dir = {})
{
    RobotDef def;
    auto file = dir / (robot + ".yaml");
    if (!std::filesystem::exists(file)) {
        return def;
    }

    auto cfg = YAML::LoadFile(file.string());
    def.from_file = true;
    if (cfg["scene_dir"]) {
        def.scene_dir = cfg["scene_dir"].as<std::string>();
    }
    if (cfg["idl"]) {
        auto idl = cfg["idl"].as<std::string>();
        if (idl == "go") {
            def.idl_type = param::IDL_GO;
        } else if (idl == "hg") {
            def.idl_type = param::IDL_HG;
        } else {
            throw std::invalid_argument(file.string() + ": idl must be 'go' or 'hg', got '" + idl + "'");
        }
    }
    if (cfg["motors"]) {
        for (const auto &kv : cfg["motors"]) {
            def.motors[kv.first.as<int>()] = kv.second.as<std::string>();
        }
    }
    if (cfg["components"]) {
        for (const auto &c : cfg["components"]) {
            ComponentSpec spec;
            spec.type = c.IsScalar() ? c.as<std::string>() : c["type"].as<std::string>();
            if (c.IsMap()) { spec.params = YAML::Node(c); }
            if (!ComponentRegistry::instance().has(spec.type)) {
                throw std::invalid_argument(file.string() + ": unknown component '" + spec.type +
                                            "' (available: " + ComponentRegistry::instance().names() + ")");
            }
            def.components.push_back(spec);
        }
    }
    if (cfg["attachments"]) {
        for (const auto &a : cfg["attachments"]) {
            try {
                def.attachments.push_back(ParseAttachment(a, components_dir));
            } catch (const std::exception &e) {
                throw std::invalid_argument(file.string() + ": " + e.what());
            }
        }
    }
    return def;
}
