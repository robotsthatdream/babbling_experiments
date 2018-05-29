//
// Created by phlf on 22/07/16.
//
#include "cafer_core/cafer_core.hpp"
#include <dream_babbling/pose_goalAction.h>
#include <yaml-cpp/yaml.h>

class DataController : public cafer_core::Data {
public:
    using cafer_core::Data::Data;

    virtual std::map<std::string, std::string> get_serialized_data() const override
    {
        std::string frame_id;
        std::map<std::string, std::string> serialized_data;
        YAML::Emitter joints_yml;

        cafer_core::shared_ptr<dream_babbling::pose_goalActionFeedback> msg;
        msg = _stored_msg.instantiate<dream_babbling::pose_goalActionFeedback>();

        frame_id = "frame_" + std::to_string(msg->header.stamp.sec + msg->header.stamp.nsec);

        if (!msg->feedback.joints_positions.empty()) {

            joints_yml << YAML::BeginMap
                           << YAML::Key << frame_id << YAML::Value
                           << YAML::BeginMap
                               << YAML::Key << "timestamp" << YAML::Value
                               << YAML::BeginMap
                                   << YAML::Key << "sec" << YAML::Value << msg->header.stamp.sec
                                   << YAML::Key << "nsec" << YAML::Value << msg->header.stamp.nsec
                               << YAML::EndMap
                               << YAML::Key << "joints_values" << YAML::Value
                               << YAML::BeginMap;

            for (unsigned int i = 0; i < msg->feedback.joints_positions.size(); ++i) {
                joints_yml << YAML::Key << "joint_" + std::to_string(i) << YAML::Value
                           << msg->feedback.joints_positions[i];
            }
            joints_yml << YAML::EndMap << YAML::EndMap << YAML::EndMap << YAML::Newline;
        }

        serialized_data["joints_values"] = joints_yml.c_str();

        return serialized_data;
    }
};
