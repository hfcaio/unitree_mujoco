#pragma once

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "bridge_component.h"
#include "param.h"

// Per-robot description loaded from simulate/robots/<robot>.yaml:
//
//   idl: hg                  # go | hg (optional; omitted = decide from the actuator count)
//   components:              # bridge components, see components.h
//     - secondary_imu
//     - bms_state
struct RobotDef
{
    bool from_file = false;
    int idl_type = param::IDL_AUTO;
    std::vector<std::string> components;
};

inline RobotDef LoadRobotDef(const std::filesystem::path &dir, const std::string &robot)
{
    RobotDef def;
    auto file = dir / (robot + ".yaml");
    if (!std::filesystem::exists(file)) {
        return def;
    }

    auto cfg = YAML::LoadFile(file.string());
    def.from_file = true;
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
    if (cfg["components"]) {
        for (const auto &c : cfg["components"]) {
            auto name = c.as<std::string>();
            if (!ComponentRegistry::instance().has(name)) {
                throw std::invalid_argument(file.string() + ": unknown component '" + name +
                                            "' (available: " + ComponentRegistry::instance().names() + ")");
            }
            def.components.push_back(name);
        }
    }
    return def;
}
