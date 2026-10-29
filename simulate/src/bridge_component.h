#pragma once

#include <mujoco/mujoco.h>

#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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

    mjModel *mj_model_;
    mjData *mj_data_;
};

class ComponentRegistry
{
public:
    using Factory = std::function<std::unique_ptr<BridgeComponent>(mjModel *, mjData *)>;

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

    std::unique_ptr<BridgeComponent> create(const std::string &name, mjModel *model, mjData *data) const
    {
        auto it = factories_.find(name);
        if (it == factories_.end()) {
            throw std::invalid_argument("unknown bridge component '" + name + "' (available: " + names() + ")");
        }
        return it->second(model, data);
    }

private:
    std::map<std::string, Factory> factories_;
};
