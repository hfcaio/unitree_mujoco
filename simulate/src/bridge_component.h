#pragma once

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include <unitree/dds_wrapper/common/unitree_joystick.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// What the base bridge exposes to its components: read-only snapshots of the messages it
// published on the last tick (rt/lowstate, rt/sportmodestate), so a component can republish
// them (e.g. the low-rate lf/* topics) without duplicating the logic that fills them.
// Exactly one of go_lowstate / hg_lowstate is set, matching the robot's IDL.
struct BridgeContext
{
    const unitree_go::msg::dds_::LowState_ *go_lowstate = nullptr;
    const unitree_hg::msg::dds_::LowState_ *hg_lowstate = nullptr;
    const unitree_go::msg::dds_::SportModeState_ *sportmodestate = nullptr;
    std::shared_ptr<unitree::common::UnitreeJoystick> joystick;
};

// A BridgeComponent is one self-contained piece of the robot <-> DDS bridge
// (e.g. the torso IMU topic). Components are listed per robot in
// simulate/robots/<robot>.yaml and instantiated by name through the registry.
class BridgeComponent
{
public:
    virtual ~BridgeComponent() = default;

    // Called once per bridge tick, after the base bridge has run.
    virtual void step() = 0;

protected:
    BridgeComponent(mjModel *model, mjData *data) : mj_model_(model), mj_data_(data) {}

    // Start address of a named sensor in sensordata, or -1 when the model has none.
    int sensorAdr(const char *name) const
    {
        int id = mj_name2id(mj_model_, mjOBJ_SENSOR, name);
        return id >= 0 ? mj_model_->sensor_adr[id] : -1;
    }

    // Start address in sensordata of the first sensor of `type` attached to object `objid`, or -1.
    int sensorAdrByObj(int type, int objtype, int objid) const
    {
        for (int i = 0; i < mj_model_->nsensor; i++) {
            if (mj_model_->sensor_type[i] == type && mj_model_->sensor_objtype[i] == objtype &&
                mj_model_->sensor_objid[i] == objid) {
                return mj_model_->sensor_adr[i];
            }
        }
        return -1;
    }

    mjModel *mj_model_;
    mjData *mj_data_;
};

class ComponentRegistry
{
public:
    using Factory = std::function<std::unique_ptr<BridgeComponent>(mjModel *, mjData *, const YAML::Node &, const BridgeContext &)>;

    static ComponentRegistry &instance()
    {
        static ComponentRegistry registry;
        return registry;
    }

    void add(const std::string &name, Factory factory) { factories_[name] = std::move(factory); }

    bool has(const std::string &name) const { return factories_.count(name) > 0; }

    std::string names() const
    {
        std::string out;
        for (const auto &kv : factories_) {
            out += (out.empty() ? "" : ", ") + kv.first;
        }
        return out;
    }

    std::unique_ptr<BridgeComponent> create(const std::string &name, mjModel *model, mjData *data,
                                            const YAML::Node &params = YAML::Node(),
                                            const BridgeContext &ctx = BridgeContext()) const
    {
        auto it = factories_.find(name);
        if (it == factories_.end()) {
            throw std::invalid_argument("unknown bridge component '" + name + "' (available: " + names() + ")");
        }
        return it->second(model, data, params, ctx);
    }

private:
    std::map<std::string, Factory> factories_;
};
