#pragma once

#include "json.hpp"

struct ObjectDetected {
    int id;
    float position[2];
    float velocity[2];

    nlohmann::json to_json() {
        nlohmann::json obj;
        obj["id"] = id;
        obj["position"] = { position[0], position[1] };
        obj["velocity"] = { velocity[0], velocity[1] };
        return obj;
    }

    bool from_json(nlohmann::json obj) {
        if (obj.contains("position")) {
            auto pos = obj.at("position");
            if (pos.size() < 2) return false;
            position[0] = pos[0];
            position[1] = pos[1];
        }
        else return false;
        if (obj.contains("velocity")) {
            auto pos = obj.at("velocity");
            if (pos.size() < 2) return false;
            velocity[0] = pos[0];
            velocity[1] = pos[1];
        }
        if (obj.contains("id")) {
            id = obj.at("id");
        }
        return true;
    }
};
