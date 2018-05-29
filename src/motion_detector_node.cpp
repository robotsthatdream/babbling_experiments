#include <cafer_core/cafer_core.hpp>
#include <image_processing/MotionDetection.h>
#include <image_processing/default_parameters.hpp>
#include <image_processing/pcl_types.h>
#include <image_processing/SupervoxelSet.h>
#include <dream_babbling/rgbd_motion_data.h>
#include <dream_babbling/is_moving.h>
#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>
#include "globals.h"
#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <std_msgs/Bool.h>

using namespace cafer_core;
namespace ip = image_processing;

class MotionSensor : public Component {
    using Component::Component;

public:


    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
            sensor_msgs::Image, sensor_msgs::CameraInfo>;

    ~MotionSensor()
    {
        client_disconnect_from_ros();
    }

    void init() override
    {

        _current_past_frame[0].reset(new ip::PointCloudT);
        _current_past_frame[1].reset(new ip::PointCloudT);

        client_connect_to_ros();
        _is_init = true;
    }


    void client_connect_to_ros() override
    {

        XmlRpc::XmlRpcValue params;

        cafer_core::ros_nh->getParam("/dream_babbling", params);

//        cafer_core::ros_nh->getParam("ite_end_topic_name",ite_end_topic_name);

        ros::NodeHandle mt_callback_nh;

        mt_callback_nh.setCallbackQueue(&_image_processing_cb_q);
        _motion_data_publisher.reset(new ros::Publisher(
                mt_callback_nh.advertise<dream_babbling::rgbd_motion_data>(ros::this_node::getName(), 5)));


        _rgbd_sub.reset(new rgbd_utils::RGBD_Subscriber(
                            params["rgb_info_topic"],
                            params["rgb_topic"],
                            params["depth_info_topic"],
                            params["depth_topic"],
                            *ros_nh));

//        _rgb_sub.reset(new message_filters::Subscriber<sensor_msgs::Image>(*ros_nh,static_cast<std::string>(params["rgb_topic"]),5));
//        _depth_sub.reset(new message_filters::Subscriber<sensor_msgs::Image>(*ros_nh,static_cast<std::string>(params["depth_topic"]),5));
//        _rgb_info_sub.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>(*ros_nh,static_cast<std::string>(params["rgb_info_topic"]),5));

//        _sync.reset(
//                new message_filters::Synchronizer<SyncPolicy>
//                    (SyncPolicy(5), *_depth_sub, *_rgb_sub, *_rgb_info_sub));
//        _sync->registerCallback(boost::bind(&MotionSensor::image_callback, this, _1, _2, _3));

        _is_motion_srv.reset(new ros::ServiceServer(cafer_core::ros_nh->advertiseService
                ("is_moving", &MotionSensor::is_moving_cb, this)));

        _iter_end.reset(new Subscriber(ros_nh->subscribe<std_msgs::Bool>(params["is_finish"],5,&MotionSensor::end_iteration_cb,this)));

        _spinner.reset(new ros::AsyncSpinner(4, &_image_processing_cb_q));
        _spinner->start();
        _babbling_terminated.reset(new ros::Subscriber(ros_nh->subscribe("/dream_babbling/babbling_node/babbling_terminated",5,&MotionSensor::babbling_term_cb,this)));

        _current_cloud_pub.reset(new ros::Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>("current_cloud",5)));
        _past_cloud_pub.reset(new ros::Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>("past_cloud",5)));
        _diff_pub.reset(new ros::Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>("diff_cloud",5)));


        _init_workspace();
    }

    void client_disconnect_from_ros() override
    {
        _spinner->stop();
        _motion_data_publisher.reset();
        _spinner.reset();
        _rgbd_sub.reset();
//        _rgb_sub.reset();
//        _depth_sub.reset();
//        _rgb_info_sub.reset();
        _is_motion_srv.reset();
        _iter_end.reset();
        _babbling_terminated.reset();

        _current_cloud_pub.reset();
        _past_cloud_pub.reset();
        _diff_pub.reset();

    }

    void babbling_term_cb(const std_msgs::BoolConstPtr msg){
        if(msg->data == 1){
            ROS_INFO_STREAM("BABBLING IS FINISH");
            is_finish = true;
        }
    }


    void update() override
    {
        if(is_finish)
            exit(0);

        dream_babbling::rgbd_motion_data msg;
        dream_babbling::motion_rect serialized_rect;
        std::vector<cv::Rect> motion_ROIs;
        cv_bridge::CvImagePtr cv_ptr;
        ip::PointCloudT::Ptr cloud(new ip::PointCloudT);
        try {
            //Detect motion
            rgbd_utils::RGBD_to_Pointcloud converter(_rgbd_sub->get_depthConstPtr(),
                                                     _rgbd_sub->get_rgbConstPtr(),
                                                     _rgbd_sub->get_rgb_infoConstPtr());
            converter.convert();

            if (_current_past_frame[0]->empty())
                pcl::fromROSMsg(converter.get_pointcloud(),*(_current_past_frame[0]));
            else
                pcl::fromROSMsg(converter.get_pointcloud(),*(_current_past_frame[1]));



            cv_ptr = cv_bridge::toCvCopy(_rgbd_sub->get_rgb(), sensor_msgs::image_encodings::RGB8);
            _image_worker.detect_MOG(cv_ptr->image);
            motion_ROIs = _image_worker.getResultsRects();


            //Fill and publish message
            if (!motion_ROIs.empty()) {
                for (auto rect:motion_ROIs) {
                    serialized_rect.x = rect.x;
                    serialized_rect.y = rect.y;
                    serialized_rect.width = rect.width;
                    serialized_rect.height = rect.height;

                    msg.motion_rects.push_back(serialized_rect);
                }
            }

            msg.header.stamp = ros::Time::now();
            msg.rgb_info = _rgbd_sub->get_rgb_info();
            msg.rgb = _rgbd_sub->get_rgb();
            msg.depth = _rgbd_sub->get_depth();

            _motion_data_publisher->publish(msg);


            _workspace->filter(_current_past_frame[0]);
            _workspace->filter(_current_past_frame[1]);
            sensor_msgs::PointCloud2 cc_msg,pc_msg;
            pcl::toROSMsg(*(_current_past_frame[1]),cc_msg);
            pcl::toROSMsg(*(_current_past_frame[0]),pc_msg);
            cc_msg.header = _rgbd_sub->get_depth().header;
            pc_msg.header = _rgbd_sub->get_depth().header;
            _current_cloud_pub->publish(cc_msg);
            _past_cloud_pub->publish(pc_msg);

        }
        catch (cv_bridge::Exception& e) { ROS_ERROR("MOTION_DETECTOR : cv_bridge exception: %s", e.what()); }


    }

    void end_iteration_cb(const std_msgs::BoolConstPtr& is_finish){
        if(is_finish->data){
            ROS_INFO_STREAM("MOTION_DETECTOR : iteration is finish");
//            _current_past_frame[1].copyTo(_current_past_frame[0]);
        }
    }

    bool is_moving_cb(dream_babbling::is_moving::Request& req, dream_babbling::is_moving::Response& rep)
    {
        ROS_INFO_STREAM("MOTION_DETECTOR : is_moving_cb");
        try {

            if(_current_past_frame[1]->empty() || _current_past_frame[0]->empty())
                return false;

            std::vector<double> center = {req.supervoxel_center.x,
                                          req.supervoxel_center.y,
                                          req.supervoxel_center.z};
            ip::PointCloudXYZ::Ptr sv(new ip::PointCloudXYZ);
            pcl::fromROSMsg(req.supervoxel_cloud,*sv);



            _image_worker.setInputClouds(_current_past_frame[0],_current_past_frame[1]);


            ip::PointCloudXYZ::Ptr diff(new ip::PointCloudXYZ);
            bool is_moving = _image_worker.detect_on_cloud(sv,center,diff,0.,0.005,0.20,0.01);

            sensor_msgs::PointCloud2 diff_msg;
            pcl::toROSMsg(*diff,diff_msg);
            diff_msg.header = _rgbd_sub->get_depth().header;
            _diff_pub->publish(diff_msg);

            if (is_moving) {
                rep.has_moved.data = 1;
                ROS_INFO_STREAM("MOTION_DETECTOR : positive sample");
            }
            else {
                rep.has_moved.data = 0;
                ROS_INFO_STREAM("MOTION_DETECTOR : negative sample");
            }

            _current_past_frame[0].reset(new ip::PointCloudT);
            _current_past_frame[1].reset(new ip::PointCloudT);

            ROS_INFO_STREAM("MOTION_DETECTOR : done ");
        }
        catch (cv::Exception& e) {
            ROS_ERROR_STREAM(e.what());
            exit(1);
        }

        return true;
    }

private:

    void _init_workspace()
    {
        XmlRpc::XmlRpcValue wks;
        cafer_core::ros_nh->getParam("experiment/workspace", wks);

        _workspace.reset(
                new ip::workspace_t(true,
                                static_cast<double>(wks["sphere"]["x"]),
                               static_cast<double> (wks["sphere"]["y"]),
                                static_cast<double>(wks["sphere"]["z"]),
                               static_cast<double> (wks["sphere"]["radius"]),
                               static_cast<double> (wks["sphere"]["threshold"]),
                                {static_cast<double>(wks["csg_intersect_cuboid"]["x_min"]),
                static_cast<double>(wks["csg_intersect_cuboid"]["x_max"]),
                                static_cast<double> (wks["csg_intersect_cuboid"]["y_min"]),
                static_cast<double>(wks["csg_intersect_cuboid"]["y_max"]),
                                static_cast<double> (wks["csg_intersect_cuboid"]["z_min"]),
                static_cast<double>(wks["csg_intersect_cuboid"]["z_max"])}));

    }

    void _3d_coord_to_pixel_coord(std::array<double, 2>& pix, const std::array<double, 3>& coord)
    {
        pix[0] = babbling::sv_param::focal_length_x * coord[0] / coord[2]
                 + babbling::sv_param::rgb_princ_pt_x;
        pix[1] = babbling::sv_param::focal_length_y * coord[1] / coord[2]
                 + babbling::sv_param::rgb_princ_pt_y;
    }

    ip::MotionDetection _image_worker;

    std::unique_ptr<rgbd_utils::RGBD_Subscriber> _rgbd_sub;
    std::unique_ptr<ip::workspace_t> _workspace;

//    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy>> _sync;
//    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> _rgb_sub;
//    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> _depth_sub;
//    std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> _rgb_info_sub;

    std::unique_ptr<ros::Publisher> _motion_data_publisher;
    std::unique_ptr<ros::ServiceServer> _is_motion_srv;
    std::unique_ptr<Subscriber> _iter_end;
    std::unique_ptr<Publisher> _current_cloud_pub;
    std::unique_ptr<Publisher> _past_cloud_pub;
    std::unique_ptr<Publisher> _diff_pub;

    ros::CallbackQueue _image_processing_cb_q;
    std::unique_ptr<ros::AsyncSpinner> _spinner;

    std::array<ip::PointCloudT::Ptr, 2> _current_past_frame;

    std::unique_ptr<ros::Subscriber> _babbling_terminated;
    bool is_finish = false;
};


int main(int argc, char** argv)
{
    std::string node_name;
    XmlRpc::XmlRpcValue cafer;
    node_name = global::parse_arg(argc, argv, "motion_detector_node");

    cafer_core::init(argc, argv, node_name);
    cafer_core::ros_nh->getParam("cafer", cafer);


    cafer_core::init(argc, argv, node_name);

    MotionSensor motion_detect(cafer["mgmt"], cafer["type"], cafer["freq"], cafer["uuid"]);

    motion_detect.wait_for_init();
//    std::this_thread::sleep_for(std::chrono::seconds(3));
    motion_detect.spin();

    while (ros::ok() && (!motion_detect.get_terminate())) {
        motion_detect.spin();
        motion_detect.update();
        motion_detect.sleep();
    }

    return 0;
}
