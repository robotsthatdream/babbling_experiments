#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

#include <cafer_core/cafer_core.hpp>

#include <image_processing/SupervoxelSet.h>
#include <globals.h>


class display_workspace{

public:
    display_workspace(){
        XmlRpc::XmlRpcValue glob_params;
        XmlRpc::XmlRpcValue exp_params;
        cafer_core::ros_nh->getParam("/global", glob_params);
        cafer_core::ros_nh->getParam("experiment", exp_params);

        _workspace_pub.reset(new cafer_core::Publisher(
                                 cafer_core::ros_nh->advertise<sensor_msgs::PointCloud2>(
                                     "worspace_cloud",5)));
        XmlRpc::XmlRpcValue wks;
        cafer_core::ros_nh->getParam("experiment/workspace", wks);
        babbling::init_workspace(wks,_workspace);


        _pointcloud_sub.reset(new ros::Subscriber(cafer_core::ros_nh->subscribe<sensor_msgs::PointCloud2>
                                             (glob_params["cloud_topic"],5, boost::bind(&display_workspace::cloud_callback, this, _1))));
    }

    ~display_workspace(){
        _pointcloud_sub.reset();
        _workspace_pub.reset();
    }


    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
        if(!cloud_msg)
            return;
        _inputcloud_msg = *cloud_msg;
    }

    void execute(){
        sensor_msgs::PointCloud2 cloud_msg;
        image_processing::PointCloudT::Ptr cloud(new image_processing::PointCloudT);
        pcl::fromROSMsg(_inputcloud_msg, *cloud);


        update_workspace();

        _workspace->filter(cloud);

//        pcl::StatisticalOutlierRemoval<image_processing::PointT> sor;
//        sor.setInputCloud(cloud);
//        sor.setMeanK (20);
//        sor.setStddevMulThresh (5.0);
//        sor.filter(*cloud);

        pcl::toROSMsg(*cloud,cloud_msg);
        cloud_msg.header = _inputcloud_msg.header;
        _workspace_pub->publish(cloud_msg);
    }


    void update_workspace()
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
    std::unique_ptr<ros::Subscriber> _pointcloud_sub;
    sensor_msgs::PointCloud2 _inputcloud_msg;
    cafer_core::PublisherPtr _workspace_pub;
    std::unique_ptr<image_processing::workspace_t> _workspace;
};

int main(int argc, char** argv){

    cafer_core::init(argc, argv, "display_workspace");

    display_workspace dp;


    while(ros::ok()){
        dp.execute();
        ros::spinOnce();
    }

    return 0;
}
