#include <iostream>

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

#include <yaml-cpp/yaml.h>

#include <boost/shared_ptr.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <globals.h>
#include <utilities.hpp>

#include <pcl/filters/statistical_outlier_removal.h>

#include <relevance_map/relevance_map_node.hpp>

namespace rm = relevance_map;

class saliency_map_viz : public rm::relevance_map_node{

public:
    saliency_map_viz(){

        initialize(cafer_core::ros_nh);

        init_classifiers(_load_exp);

//        for(const auto& mod : _modalities){
//            _soi_pub.emplace(mod,cafer_core::PublisherPtr(new ros::Publisher(cafer_core::ros_nh->advertise<sensor_msgs::PointCloud2>("saliency_map_" + mod,5))));
//        }

        _sv_pub.reset(new ros::Publisher(cafer_core::ros_nh->advertise<sensor_msgs::PointCloud2>("supervoxels",5)));
//        _cumul_rm_pub.reset(new ros::Publisher(cafer_core::ros_nh->advertise<sensor_msgs::PointCloud2>("cumul_relevance_map",5)));
//        _soi_pub.reset(new ros::Publisher(cafer_core::ros_nh->advertise<sensor_msgs::PointCloud2>("saliency_map",5)));
        //        if(_soi_method="mcs")
//            _soi_pub.emplace("merge",cafer_core::PublisherPtr(new ros::Publisher(cafer_core::ros_nh->advertise<sensor_msgs::PointCloud2>("saliency_map_" + mod,5))));

        XmlRpc::XmlRpcValue exp_params; // parameters stored in experiment_params.yml
        cafer_core::ros_nh->getParam("experiment", exp_params);
        _with_bluring = std::stoi(exp_params["soi"]["with_bluring"]);
        _increment_bluring = std::stod(exp_params["soi"]["increment"]);
        _with_adaptive_thres = std::stoi(exp_params["soi"]["with_adpative_thres"]);
//        int cumul_size = std::stoi(exp_params["soi"]["cumul_size"]);
//        _maps.resize(cumul_size);
//        _counter = 0;
    }

    ~saliency_map_viz(){
        release();
        _sv_pub.reset();
//        _cumul_rm_pub.reset();
//        _soi_pub.reset();
    }




    void execute(){

        sensor_msgs::PointCloud2 sv_msg;
//        sensor_msgs::PointCloud2 soi_msg;

        ip::PointCloudT::Ptr cloud(new ip::PointCloudT);
        if(!retrieve_input_cloud(cloud))
            return;

        _compute_supervoxels(cloud);
        _compute_relevance_map();
        ROS_INFO_STREAM(_gmm_class[_modality].print_info());
        _gmm_class[_modality].model()[0][0]->update_parameters();
        ROS_INFO_STREAM("apres update : " << _gmm_class[_modality].print_info());

        //        _maps[_counter] = _soi.getColoredWeightedCloud(_modality);
//        _counter++;
//        if(_counter >= _maps.size()){
//            _counter = 0;
//            _cumul_rm = _soi.cumulative_relevance_map(_maps);
//            sensor_msgs::PointCloud2 cumul_rm_msg;
//            pcl::toROSMsg(_cumul_rm, cumul_rm_msg);
//            cumul_rm_msg.header = _images_sub->get_depth().header;
//            _cumul_rm_pub->publish(cumul_rm_msg);
//        }
        if(_with_bluring)
            _soi.neighbor_bluring(_modality,_increment_bluring,1);
        if(_with_adaptive_thres)
            _soi.adaptive_threshold(_modality,1);

        image_processing::PointCloudT colored_cloud;
        _soi.getColoredCloud(colored_cloud);
        pcl::toROSMsg(colored_cloud,sv_msg);
        sv_msg.header = _images_sub->get_depth().header;
        _sv_pub->publish(sv_msg);

        publish_feedback();
    }



private:
    cafer_core::PublisherPtr _sv_pub;
//    cafer_core::PublisherPtr _cumul_rm_pub;
//    std::map<std::string,cafer_core::PublisherPtr> _soi_pub;
    int _with_bluring;
    int _with_adaptive_thres;
    double _increment_bluring;
//    std::vector<pcl::PointCloud<pcl::PointXYZI>> _maps;
//    pcl::PointCloud<pcl::PointXYZI> _cumul_rm;
//    int _counter;
};

int main(int argc, char** argv){

    cafer_core::init(argc, argv, "saliency_map_viz");

    saliency_map_viz sm_viz;

    while(ros::ok()){
        sm_viz.execute();
        ros::spinOnce();
    }

    return 0;
}
