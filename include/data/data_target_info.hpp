//
// Created by phlf on 22/07/16.
//

#ifndef DATA_TARGET_INFO_HPP
#define DATA_TARGET_INFO_HPP

#include "cafer_core/cafer_core.hpp"
#include <dream_babbling/target_info.h>
#include <yaml-cpp/yaml.h>

class DataTargetInfo : public cafer_core::Data {
public:
    using cafer_core::Data::Data;

    virtual std::map<std::string, std::string> get_serialized_data() const override
    {

        ROS_INFO_STREAM("DATA TAGRET INFO : data receive to serialized");

        std::map<std::string, std::string> serialized_data;
        YAML::Emitter yml_emitter;

        cafer_core::shared_ptr<dream_babbling::target_info> msg;
        msg = _stored_msg.instantiate<dream_babbling::target_info>();

        yml_emitter << YAML::BeginMap //BEGIN MAP_0
                        << YAML::Key << "reward" << YAML::Value << msg->reward //BEGIN MAP_3
                        << YAML::Key << "target_position" << YAML::Value
                        << YAML::BeginSeq
                            << msg->target_position.x
                            << msg->target_position.y
                            << msg->target_position.z
                        << YAML::EndSeq
                    << YAML::EndMap;



        serialized_data["target_info"] = yml_emitter.c_str();
        return serialized_data;
    }
};

#endif //DATA_TARGET_INFO_HPP
