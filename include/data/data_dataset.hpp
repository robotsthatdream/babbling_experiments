//
// Created by phlf on 22/07/16.
//

#ifndef DATA_DATASET_HPP
#define DATA_DATASET_HPP

#include "cafer_core/cafer_core.hpp"
#include <dream_babbling/dataset.h>
#include <yaml-cpp/yaml.h>

class DataDataset : public cafer_core::Data {
public:
    using cafer_core::Data::Data;

    virtual std::map<std::string, std::string> get_serialized_data() const override
    {
        ROS_INFO_STREAM("CLASSIFIER DATASET : data receive to serialized");

        std::map<std::string, std::string> serialized_data;
        std::string frame_id, feat_id;
        YAML::Emitter soi_yml;

        cafer_core::shared_ptr<dream_babbling::dataset> msg;
        msg = _stored_msg.instantiate<dream_babbling::dataset>();

        frame_id = "frame_" + std::to_string(msg->header.stamp.sec + msg->header.stamp.nsec);

        soi_yml << YAML::BeginMap //BEGIN MAP_0
                    << YAML::Key << frame_id << YAML::Value
                    << YAML::BeginMap //BEGIN MAP_1
                        << YAML::Key << "timestamp" << YAML::Value
                        << YAML::BeginMap //BEGIN MAP_2
                            << YAML::Key << "sec" << YAML::Value << msg->header.stamp.sec
                            << YAML::Key << "nsec" << YAML::Value << msg->header.stamp.nsec
                        << YAML::EndMap //END MAP_2
                        << YAML::Key << "features" << YAML::Value
                        << YAML::BeginMap; //BEGIN MAP_3

        for (unsigned int i = 0; i < msg->features.size(); ++i) {
            feat_id = "feature_" + std::to_string(i);

            soi_yml << YAML::Key << feat_id << YAML::Value
                    << YAML::BeginMap //BEGIN MAP_4
                        << YAML::Key << "label" << YAML::Value << (msg->features[i].label /*!= '1'*/)
                        << YAML::Key << "value" << YAML::Value
                        << YAML::BeginSeq;
            for (unsigned int j = 0; j < msg->features[i].feature.size(); ++j) {
                soi_yml  << msg->features[i].feature[j];
            }
            soi_yml << YAML::EndSeq
                    << YAML::EndMap; //END MAP_4
        }
        soi_yml << YAML::EndMap //END MAP_3
                << YAML::EndMap //END MAP_1
                << YAML::EndMap //END MAP_0
                << YAML::Newline;


        serialized_data["dataset_"+msg->type.data] = soi_yml.c_str();
        return serialized_data;
    }
};

#endif //DATA_DATASET_HPP
