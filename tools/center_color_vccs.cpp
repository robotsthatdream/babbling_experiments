#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

#include <cafer_core/cafer_core.hpp>

#include <image_processing/SupervoxelSet.h>



class center_color_vccs{

public:
    center_color_vccs(){
        XmlRpc::XmlRpcValue glob_params;
        cafer_core::ros_nh->getParam("/global", glob_params);
        _rgbd_sub.reset(new rgbd_utils::RGBD_Subscriber(
                            glob_params["rgb_info_topic"],
                            glob_params["rgb_topic"],
                            glob_params["depth_info_topic"],
                            glob_params["depth_topic"],
                            *cafer_core::ros_nh));

        _sv_pub.reset(new ros::Publisher(cafer_core::ros_nh->advertise<sensor_msgs::PointCloud2>("supervoxels",5)));

        init_workspace();

    }

    ~center_color_vccs(){
        _rgbd_sub.reset();
        _sv_pub.reset();
        _sv.reset();
    }

    bool wait_for_images(){
        return !_rgbd_sub->get_depth().data.empty() && !_rgbd_sub->get_depth().data.empty();
    }

    void center_color_cloud(image_processing::PointCloudT& cloud){
        for(const auto& sv : _sv->getSupervoxels()){
            for(const auto& v : sv.second->voxels_->points){
                image_processing::PointT pt;
                pt.x = v.x;
                pt.y = v.y;
                pt.z = v.z;
                pt.r = sv.second->centroid_.r;
                pt.g = sv.second->centroid_.g;
                pt.b = sv.second->centroid_.b;
                cloud.push_back(pt);
            }
        }
    }

    void execute(){

        _sv.reset(new image_processing::SupervoxelSet);

        sensor_msgs::ImageConstPtr depth_msg(
                    new sensor_msgs::Image(_rgbd_sub->get_depth()));
        sensor_msgs::ImageConstPtr rgb_msg(
                    new sensor_msgs::Image(_rgbd_sub->get_rgb()));
        sensor_msgs::CameraInfoConstPtr info_msg(
                    new sensor_msgs::CameraInfo(_rgbd_sub->get_rgb_info()));


        sensor_msgs::PointCloud2 sv_msg;

        rgbd_utils::RGBD_to_Pointcloud converter(depth_msg,rgb_msg,info_msg);
        sensor_msgs::PointCloud2 cloud_msg = converter.get_pointcloud();
        image_processing::PointCloudT::Ptr cloud(new image_processing::PointCloudT);
        pcl::fromROSMsg(cloud_msg,*cloud);

        _sv->setInputCloud(cloud);
        _sv->computeSupervoxel(*_workspace);

        image_processing::PointCloudT output_cloud;
        center_color_cloud(output_cloud);
        pcl::toROSMsg(output_cloud,sv_msg);
        sv_msg.header = depth_msg->header;

        _sv_pub->publish(sv_msg);

    }

    void init_workspace()
    {
        XmlRpc::XmlRpcValue wks;

        cafer_core::ros_nh->getParamCached("experiment/workspace", wks);

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
    rgbd_utils::RGBD_Subscriber::Ptr _rgbd_sub;
    std::unique_ptr<image_processing::workspace_t> _workspace;
    cafer_core::PublisherPtr _sv_pub;
    image_processing::SupervoxelSet::Ptr _sv;

};

int main(int argc, char** argv){

    cafer_core::init(argc, argv, "display_workspace");

    center_color_vccs cc_vccs;

    while(cc_vccs.wait_for_images()){
        ros::spinOnce();
    }


    while(ros::ok()){
        cc_vccs.execute();
        ros::spinOnce();
    }

    return 0;
}
