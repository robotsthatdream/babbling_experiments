#include <iostream>

#include <pcl/io/pcd_io.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

#include <cafer_core/cafer_core.hpp>

#include <image_processing/SupervoxelSet.h>
#include <image_processing/SurfaceOfInterest.h>
#include <image_processing/features.hpp>

#include <iagmm/nnmap.hpp>
#include <iagmm/gmm.hpp>
#include <iagmm/mcs.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <relevance_map/relevance_map_node.hpp>
#include <relevance_map/utilities.hpp>

namespace rm = relevance_map;
namespace ip = image_processing;

class choice_heat_map : public rm::relevance_map_node{

public:
    choice_heat_map(){

        initialize(cafer_core::ros_nh);

        _heat_map_pub.reset(new ros::Publisher(cafer_core::ros_nh->advertise<sensor_msgs::PointCloud2>("choice_heat_map",5)));
        rm::load_archive(_load_exp,_ite_folders);

        XmlRpc::XmlRpcValue exp;
        cafer_core::ros_nh->getParam("experiment/soi",exp);
        _output_file = static_cast<std::string>(exp["output_file"]);
        _output_file = _load_exp + std::string("/") + _output_file;
        _counter = 0;
    }

    ~choice_heat_map(){
        release();
    }

    bool generate_supervoxels(){
        ip::PointCloudT::Ptr cloud(new ip::PointCloudT);
        if(!retrieve_input_cloud(cloud))
            return false;
        if(!_compute_supervoxels(cloud))
            return false;

        for(const auto& val: _soi.getSupervoxels())
            _heat_map[val.first] = 0;

        return true;
    }

    void execute(){
        ROS_INFO_STREAM("Number of files last to analyse : " << _ite_folders.size());

        init_classifiers(_ite_folders.front());
        _ite_folders.pop();

        _compute_relevance_map();
        pcl::Supervoxel<ip::PointT> sv;
        uint32_t lbl;
        _compute_choice_map(sv,lbl);

        for(const auto& val: _choice_map)
            _heat_map[val.first] += val.second;

        _counter += 1;
    }

    void create_msg(){
        pcl::PointCloud<pcl::PointXYZI> heat_map_cloud;
        pcl::PointXYZI point;
        for(const auto& val: _heat_map){
            pcl::Supervoxel<ip::PointT>::Ptr current_sv = _soi.getSupervoxels()[val.first];
            for(auto v : *(current_sv->voxels_)){
                point.x = v.x;
                point.y = v.y;
                point.z = v.z;
                point.intensity = val.second/_counter;
                heat_map_cloud.push_back(point);
            }
        }
        pcl::io::savePCDFileASCII(_load_exp + std::string("/cloud.pcd"),*(_soi.getInputCloud()));
        pcl::io::savePCDFileASCII(_output_file,heat_map_cloud);

        pcl::toROSMsg(heat_map_cloud,_heat_map_msg);
        _heat_map_msg.header = _images_sub->get_depth().header;
    }

    void publish_heat_map(){
        _heat_map_pub->publish(_heat_map_msg);
    }

    bool is_finish(){return _ite_folders.empty();}

private:
    rm::relevance_map_node::map_t _heat_map;
    std::queue<std::string> _ite_folders;
    std::unique_ptr<ros::Publisher> _heat_map_pub;
    sensor_msgs::PointCloud2 _heat_map_msg;
    std::string _output_file;
    double _counter;
};

int main(int argc, char** argv){

    cafer_core::init(argc, argv, "choice_heat_map");

    choice_heat_map chm;

    while(ros::ok() && !chm.generate_supervoxels()){
        ros::spinOnce();
    }

    while(ros::ok() && !chm.is_finish()){
        chm.execute();
        ros::spinOnce();
    }

    chm.create_msg();

//    while(ros::ok()){
//        chm.publish_heat_map();
//        ros::spinOnce();
//    }

    return 0;
}
