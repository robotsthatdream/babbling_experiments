#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

#include <image_processing/SupervoxelSet.h>
#include <image_processing/SurfaceOfInterest.h>
#include <image_processing/features.hpp>

#include <iagmm/nnmap.hpp>
#include <iagmm/gmm.hpp>
#include <iagmm/mcs.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/random.hpp>

#include <dream_babbling/is_moving.h>

#include <globals.h>

class motion_detection_test{

public:
    motion_detection_test(){
        XmlRpc::XmlRpcValue glob_params;

        _nh.reset(new ros::NodeHandle("motion_detection_test"));

        _nh->getParam("/dream_babbling", glob_params);

        /* Output the values of all parameters
        std::stringstream display_params;
        for (auto& param:glob_params) {
            display_params << "\t" << param.first << ":\t" << param.second << std::endl;
        }
        ROS_INFO_STREAM("BABBLING_NODE : Parameters retrieved:" << std::endl << display_params.str());
        //*/

        _rgbd_sub.reset(new rgbd_utils::RGBD_Subscriber(
                            glob_params["rgb_info_topic"],
                            glob_params["rgb_topic"],
                            glob_params["depth_info_topic"],
                            glob_params["depth_topic"],
                            *_nh));


        _sv_pub.reset(new ros::Publisher(_nh->advertise<sensor_msgs::PointCloud2>("supervoxels",5)));

        _client_motion.reset(
                    new ros::ServiceClient(
                        _nh->serviceClient<dream_babbling::is_moving>("/motion_detector_node/is_moving")));

        _goal_pub.reset(new ros::Publisher(_nh->advertise<sensor_msgs::PointCloud2>("goal_point",5)));

        init_workspace();

    }

    ~motion_detection_test(){
        _nh.reset();
        _rgbd_sub.reset();
        _sv_pub.reset();
        _client_motion.reset();
        _goal_pub.reset();
    }

    bool wait_for_images(){
        return _rgbd_sub->get_depth().data.empty() || _rgbd_sub->get_rgb().data.empty();
    }




    bool extract_sv(){

        _sv.clear<babbling::sv_param>();

        sensor_msgs::ImageConstPtr depth_msg(
                    new sensor_msgs::Image(_rgbd_sub->get_depth()));
        sensor_msgs::ImageConstPtr rgb_msg(
                    new sensor_msgs::Image(_rgbd_sub->get_rgb()));
        sensor_msgs::CameraInfoConstPtr info_msg(
                    new sensor_msgs::CameraInfo(_rgbd_sub->get_rgb_info()));

        rgbd_utils::RGBD_to_Pointcloud converter(depth_msg,rgb_msg,info_msg);
        sensor_msgs::PointCloud2 cloud_msg = converter.get_pointcloud();
        image_processing::PointCloudT::Ptr cloud(new image_processing::PointCloudT);
        pcl::fromROSMsg(cloud_msg,*cloud);

        _sv.setInputCloud(cloud);
        return _sv.computeSupervoxel(*_workspace);
    }



    void execute(){

        sensor_msgs::PointCloud2 sv_msg;
        sensor_msgs::PointCloud2 goal_msg;
        image_processing::PointCloudT colored_cloud;
        pcl::PointCloud<pcl::PointXYZ> goal_cloud;
        boost::random::uniform_int_distribution<> dist(0,_sv.getSupervoxels().size());
        dream_babbling::is_moving srv;

        int index = dist(_gen);

        image_processing::SupervoxelArray::iterator it = _sv.getSupervoxels().begin();
        std::advance(it,index);
        if(it == _sv.getSupervoxels().end())
            return;

        pcl::toROSMsg(*(it->second->voxels_),sv_msg);
        srv.request.supervoxel_cloud = sv_msg;
//        srv.request.supervoxel_center.x = it->second->centroid_.x;
//        srv.request.supervoxel_center.y = it->second->centroid_.y;
//        srv.request.supervoxel_center.z = it->second->centroid_.z;

        goal_cloud.push_back(pcl::PointXYZ(it->second->centroid_.x,
                                           it->second->centroid_.y,
                                           it->second->centroid_.z));
        pcl::toROSMsg(goal_cloud,goal_msg);
        goal_msg.header = _rgbd_sub->get_depth().header;
        _goal_pub->publish(goal_msg);

        _sv.getColoredCloud(colored_cloud);
        pcl::toROSMsg(colored_cloud,sv_msg);
        sv_msg.header = _rgbd_sub->get_depth().header;
        _sv_pub->publish(sv_msg);

        std::cout << "Do what ever you want and press Enter" << std::endl;
        std::cin.ignore();

        if(_client_motion->call(srv)){
            if (srv.response.has_moved.data == 0) {
                ROS_INFO_STREAM("---- No Motion Detected ! ----");
            }
            else {
                ROS_INFO_STREAM("++++ Motion Detected !!!! ++++");
            }
        }else{
            ROS_ERROR_STREAM("Unable to call motion detector service");
        }
    }



    void init_workspace()
    {
        XmlRpc::XmlRpcValue wks;

        _nh->getParam("experiment/workspace", wks);

        _workspace.reset(
                new image_processing::workspace_t(true,
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

private:
    std::unique_ptr<ros::NodeHandle> _nh;
    rgbd_utils::RGBD_Subscriber::Ptr _rgbd_sub;
    std::unique_ptr<ros::ServiceClient> _client_motion;
    std::unique_ptr<ros::Publisher> _sv_pub;
    std::unique_ptr<ros::Publisher> _goal_pub;

    std::unique_ptr<image_processing::workspace_t> _workspace;

    image_processing::SupervoxelSet _sv;

    boost::random::mt19937 _gen;

};

int main(int argc, char** argv){

    ros::init(argc, argv, "motion_detection_test");

    motion_detection_test md_test;



    while(md_test.wait_for_images()){
        ROS_INFO_STREAM("Wait for images");
        ros::spinOnce();
    }


    while(ros::ok()){
        if(md_test.extract_sv())
            md_test.execute();
        ros::spinOnce();
    }

    return 0;
}
