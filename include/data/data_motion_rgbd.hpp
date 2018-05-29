//
// Created by phlf on 22/07/16.
//
#include "cafer_core/cafer_core.hpp"
#include <dream_babbling/rgbd_motion_data.h>

#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

class DataMotionRGBD : public cafer_core::Data {
public:
    using cafer_core::Data::Data;

    virtual std::map<std::string, std::string> get_serialized_data() const override
    {
        std::map<std::string, std::string> serialized_data;
        std::string frame_id, rect_id;
        YAML::Emitter rgb_yml, depth_yml, rects_yml;

        cv::Mat rgb, depth;
        std::vector<uchar> image_buffer;
        std::stringstream buffer_to_encode_0;
        std::stringstream buffer_to_encode_1;

        cafer_core::shared_ptr<dream_babbling::rgbd_motion_data> msg;
        msg = _stored_msg.instantiate<dream_babbling::rgbd_motion_data>();

        depth = cv_bridge::toCvShare(msg->depth, nullptr)->image;
        depth = cv::Mat(depth.rows, depth.cols, CV_8UC4, depth.data);

        rgb = cv_bridge::toCvShare(msg->rgb, nullptr, "bgr8")->image;

        frame_id = "frame_" + std::to_string(msg->header.stamp.sec + msg->header.stamp.nsec);

        if (!msg->motion_rects.empty()) {

            rects_yml << YAML::BeginMap
                          << YAML::Key << frame_id << YAML::Value
                          << YAML::BeginMap
                              << YAML::Key << "timestamp" << YAML::Value
                              << YAML::BeginMap
                                  << YAML::Key << "sec" << YAML::Value << msg->header.stamp.sec
                                  << YAML::Key << "nsec" << YAML::Value << msg->header.stamp.nsec
                              << YAML::EndMap
                              << YAML::Key << "rects" << YAML::Value
                              << YAML::BeginMap;

            for (unsigned int i = 0; i < msg->motion_rects.size(); ++i) {
                rect_id = "rect_" + std::to_string(i);

                rects_yml << YAML::Key << rect_id << YAML::Value
                          << YAML::BeginMap
                              << YAML::Key << "x" << YAML::Value << msg->motion_rects[i].x
                              << YAML::Key << "y" << YAML::Value << msg->motion_rects[i].y
                              << YAML::Key << "width" << YAML::Value << msg->motion_rects[i].width
                              << YAML::Key << "height" << YAML::Value << msg->motion_rects[i].height
                          << YAML::EndMap;
            }
            rects_yml << YAML::EndMap << YAML::EndMap << YAML::EndMap << YAML::Newline;
            serialized_data["motion"] = rects_yml.c_str();

        }


        //Encode depth as PNG and encode it as BASE64 string:
        cv::imencode(".png", depth, image_buffer);
        buffer_to_encode_0  << std::string(image_buffer.begin(), image_buffer.end());

        depth_yml << YAML::BeginMap
                      << YAML::Key << frame_id << YAML::Value
                      << YAML::BeginMap
                          << YAML::Key << "timestamp" << YAML::Value
                          << YAML::BeginMap
                              << YAML::Key << "sec" << YAML::Value << msg->header.stamp.sec
                              << YAML::Key << "nsec" << YAML::Value << msg->header.stamp.nsec
                          << YAML::EndMap
                          << YAML::Key << "depth" << YAML::Value << YAML::Binary(
                        reinterpret_cast<const unsigned char*>(buffer_to_encode_0.str().c_str()),
                        buffer_to_encode_0.str().size())
                      << YAML::EndMap
                  << YAML::EndMap << YAML::Newline;

        serialized_data["depth"] = depth_yml.c_str();
        image_buffer.clear();

        //Encode RGB as JPEG and encode it as BASE64 string:
        cv::imencode(".jpeg", rgb, image_buffer);

        buffer_to_encode_1 << std::string(image_buffer.begin(), image_buffer.end());

        rgb_yml << YAML::BeginMap
                    << YAML::Key << frame_id << YAML::Value
                    << YAML::BeginMap
                        << YAML::Key << "timestamp" << YAML::Value
                        << YAML::BeginMap
                            << YAML::Key << "sec" << YAML::Value << msg->header.stamp.sec
                            << YAML::Key << "nsec" << YAML::Value << msg->header.stamp.nsec
                        << YAML::EndMap
                        << YAML::Key << "rgb" << YAML::Value << YAML::Binary(
                        reinterpret_cast<const unsigned char*>(buffer_to_encode_1.str().c_str()),
                        buffer_to_encode_1.str().size())
                    << YAML::EndMap
                << YAML::EndMap << YAML::Newline;

        serialized_data["rgb"] = rgb_yml.c_str();

        return serialized_data;
    }
};
