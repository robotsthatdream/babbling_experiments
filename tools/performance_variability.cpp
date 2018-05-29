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

class saliency_map_viz{

public:
    saliency_map_viz(){
        XmlRpc::XmlRpcValue glob_params;
        XmlRpc::XmlRpcValue exp_params;
        XmlRpc::XmlRpcValue modalities, moda;

        cafer_core::ros_nh->getParam("/global", glob_params);
        cafer_core::ros_nh->getParam("experiment", exp_params);
        _rgbd_sub.reset(new rgbd_utils::RGBD_Subscriber(
                            glob_params["rgb_info_topic"],
                            glob_params["rgb_topic"],
                            glob_params["depth_info_topic"],
                            glob_params["depth_topic"],
                            *cafer_core::ros_nh));

        cafer_core::ros_nh->getParam("modalities",modalities);

        for(const auto& mod: modalities){
            moda = mod.second;
            _modalities.emplace(static_cast<std::string>(moda["name"]),moda["dimension"]);
        }
        _soi_method = static_cast<std::string>(exp_params["soi"]["method"]);
        _modality = static_cast<std::string>(exp_params["soi"]["modality"]);
        _dimension = std::stoi(exp_params["soi"]["dimension"]);
        _load_exp = static_cast<std::string>(exp_params["soi"]["load_exp"]);


//        for(const auto& mod : _modalities){
//            _soi_pub.emplace(mod,cafer_core::PublisherPtr(new ros::Publisher(cafer_core::ros_nh->advertise<sensor_msgs::PointCloud2>("saliency_map_" + mod,5))));
//        }

        _sv_pub.reset(new ros::Publisher(cafer_core::ros_nh->advertise<sensor_msgs::PointCloud2>("supervoxels",5)));
        _soi_pub.reset(new ros::Publisher(cafer_core::ros_nh->advertise<sensor_msgs::PointCloud2>("saliency_map",5)));
        //        if(_soi_method="mcs")
//            _soi_pub.emplace("merge",cafer_core::PublisherPtr(new ros::Publisher(cafer_core::ros_nh->advertise<sensor_msgs::PointCloud2>("saliency_map_" + mod,5))));

        init_workspace();

        if(_soi_method == "nnmap")
            _nnmap.emplace(_modality,iagmm::NNMap(_dimension,2,.3,.05));

        utilities::load_experiment(_soi_method,_load_exp,_modalities,_gmm,_nnmap,_mcs);


    }

    ~saliency_map_viz(){
        _rgbd_sub.reset();
        _sv_pub.reset();
        _soi_pub.reset();
    }

    bool wait_for_images(){
        return !_rgbd_sub->get_depth().data.empty() && !_rgbd_sub->get_rgb().data.empty();
    }





    void extract_sv(){

        _soi.clear<babbling::sv_param>();

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

        _soi.setInputCloud(cloud);
        _soi.computeSupervoxel(*_workspace);
    }



    void execute(){

        sensor_msgs::PointCloud2 sv_msg;
        sensor_msgs::PointCloud2 soi_msg;


        image_processing::PointCloudT colored_cloud;
        _soi.getColoredCloud(colored_cloud);
        pcl::toROSMsg(colored_cloud,sv_msg);
        sv_msg.header = _rgbd_sub->get_depth().header;
        _sv_pub->publish(sv_msg);

        if(_soi.getSupervoxels().empty())
            return;

        if(_soi_method == "nnmap"){
            _soi.init_weights(_modality,.5);
            _soi.compute_feature(_modality);
            _soi.compute_weights<iagmm::NNMap>(_modality,_nnmap[_modality]);
        }
        else if(_soi_method == "gmm"){
            _soi.compute_feature(_modality);
            _soi.compute_weights<iagmm::GMM>(_modality,_gmm[_modality]);

        }
        else if(_soi_method == "mcs"){
            for(const auto& classifier: _mcs.access_classifiers()){
               _soi.compute_feature(classifier.first);
//               _soi.compute_weights<iagmm::GMM>(classifier.first,classifier.second);
            }
            _soi.compute_weights<iagmm::MCS>(_mcs);
            image_processing::PointCloudT soi_cloud = _soi.getColoredWeightedCloud("merge");
            pcl::toROSMsg(soi_cloud,soi_msg);
            soi_msg.header = _rgbd_sub->get_depth().header;
    //        for(const auto& pub: _soi_pub)
            _soi_pub->publish(soi_msg);
            return;
        }

        image_processing::PointCloudT soi_cloud = _soi.getColoredWeightedCloud(_modality);
        pcl::toROSMsg(soi_cloud,soi_msg);
        soi_msg.header = _rgbd_sub->get_depth().header;
//        for(const auto& pub: _soi_pub)
        _soi_pub->publish(soi_msg);

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
//    std::map<std::string,cafer_core::PublisherPtr> _soi_pub;
    cafer_core::PublisherPtr _soi_pub;
    std::map<std::string,iagmm::NNMap> _nnmap;
    std::map<std::string,iagmm::GMM> _gmm;
    iagmm::MCS _mcs;


    image_processing::SurfaceOfInterest _soi;

    std::string _soi_method;
    std::string _modality;
    int _dimension;
    std::string _load_exp;
    std::map<std::string,int> _modalities;


};

int main(int argc, char** argv){

    cafer_core::init(argc, argv, "saliency_map_viz");

    saliency_map_viz sm_viz;


    while(sm_viz.wait_for_images()){
        ros::spinOnce();
    }


    while(ros::ok()){
        sm_viz.extract_sv();

        sm_viz.execute();
        ros::spinOnce();
    }

    return 0;
}
    