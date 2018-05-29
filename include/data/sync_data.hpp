//
// Created by phlf on 22/07/16.
//

#ifndef SYNC_DATA_HPP
#define SYNC_DATA_HPP

#include "cafer_core/cafer_core.hpp"
#include <dream_babbling/sync_dataset.h>

#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

class SyncData : public cafer_core::Data {
public:
    using cafer_core::Data::Data;

    virtual std::map<std::string, std::string> get_serialized_data() const override
    {
        ROS_INFO_STREAM("SYNC DATA : data receive to serialized");

        std::map<std::string, std::string> serialized_data;
        std::string frame_id;
        YAML::Emitter yml_emitter;

        cv::Mat rgb, depth;
        std::vector<uchar> image_buffer;
        std::stringstream buffer_to_encode_0;
        std::stringstream buffer_to_encode_1;



        cafer_core::shared_ptr<dream_babbling::sync_dataset> msg;
        msg = _stored_msg.instantiate<dream_babbling::sync_dataset>();

        if(msg->depth.data.empty() || msg->rgb.data.empty())
            return serialized_data;


        depth = cv_bridge::toCvShare(msg->depth, nullptr)->image;
        depth = cv::Mat(depth.rows, depth.cols, CV_8UC4, depth.data);

        rgb = cv_bridge::toCvShare(msg->rgb, nullptr, "bgr8")->image;

        frame_id = "frame_" + std::to_string(msg->header.stamp.sec + msg->header.stamp.nsec);

        //Encode depth as PNG and encode it as BASE64 string:
        cv::imencode(".png", depth, image_buffer);
        buffer_to_encode_0  << std::string(image_buffer.begin(), image_buffer.end());

        image_buffer.clear();

        //Encode RGB as JPEG and encode it as BASE64 string:
        cv::imencode(".jpeg", rgb, image_buffer);

        buffer_to_encode_1 << std::string(image_buffer.begin(), image_buffer.end());

        yml_emitter << YAML::BeginMap
                       << YAML::Key << frame_id << YAML::Value
                       << YAML::BeginMap
                       << YAML::Key << "timestamp" << YAML::Value
                       << YAML::BeginMap
                           << YAML::Key << "sec" << YAML::Value << msg->header.stamp.sec
                           << YAML::Key << "nsec" << YAML::Value << msg->header.stamp.nsec
                       << YAML::EndMap
                       << YAML::Key << "position" << YAML::Value
                          << YAML::BeginSeq
                            << msg->pose.position.x
                            << msg->pose.position.y
                            << msg->pose.position.z
                          << YAML::EndSeq;
        yml_emitter << YAML::Key << "orientation" << YAML::Value
                       << YAML::BeginSeq
                       << msg->pose.orientation.x
                       << msg->pose.orientation.y
                       << msg->pose.orientation.z
                       << msg->pose.orientation.w
                       << YAML::EndSeq;

        yml_emitter << YAML::Key << "reward" << YAML::Value << msg->reward
                    << YAML::Key << "depth" << YAML::Value << YAML::Binary(
                  reinterpret_cast<const unsigned char*>(buffer_to_encode_0.str().c_str()),
                  buffer_to_encode_0.str().size())
                    << YAML::Key << "rgb" << YAML::Value << YAML::Binary(
                    reinterpret_cast<const unsigned char*>(buffer_to_encode_1.str().c_str()),
                    buffer_to_encode_1.str().size())
                    << YAML::EndMap << YAML::EndMap << YAML::Newline;

        serialized_data["sync_dataset"] = yml_emitter.c_str();

        return serialized_data;
    }
};

#endif //SYNC_DATA_HPP
