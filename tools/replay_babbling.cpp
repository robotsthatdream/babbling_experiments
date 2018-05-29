#include <iostream>
#include <array>
#include <string>

#include <cafer_core/cafer_core.hpp>

#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <image_processing/SurfaceOfInterest.h>
#include "globals.h"
#include <iagmm/gmm.hpp>
#include <iagmm/nnmap.hpp>

#include <boost/filesystem.hpp>

#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SpawnModel.h>

#include <yaml-cpp/yaml.h>

#include <relevance_map/relevance_map_node.hpp>
#include <relevance_map/utilities.hpp>

#include <boost/archive/text_iarchive.hpp>

using namespace cafer_core;
using namespace image_processing;
using namespace rgbd_utils;
namespace rm = relevance_map;

struct sv_param {
    static constexpr bool use_transform = false;
    static constexpr double voxel_resolution = 0.008f;
    static constexpr double color_importance = 0.2f;
    static constexpr double spatial_importance = 0.4f;
    static constexpr double normal_importance = 0.4f;
    static constexpr double seed_resolution = 0.05f;
};

struct soi_param {
    static constexpr double interest_increment = .05f;
    static constexpr double non_interest_val = 0.f;
    static constexpr double color_normal_ratio = .5f;
    static constexpr double distance_threshold = .5f;
};


class ReplayBabbling : public Component, public rm::relevance_map_node{
    using Component::Component;

public:
    void init(){

        initialize(ros_nh);

        client_connect_to_ros();


        if(!load_experiment()){
            ROS_INFO_STREAM(_folder_name << " does not exist");
            return;
        }

        _counter = 0;

        while(!init_classifier())
            _counter++;

        update_workspace();
    }

    void client_connect_to_ros(){
        //* Retrieve parameters
        XmlRpc::XmlRpcValue glob_params; // parameters stored in global_params.yml
        XmlRpc::XmlRpcValue exp_params; // parameters stored in experiment_params.yml
        cafer_core::ros_nh->getParam("/global", glob_params);
        cafer_core::ros_nh->getParam("experiment", exp_params);


        _soi_method = static_cast<std::string>(exp_params["soi"]["method"]);
        _modality = static_cast<std::string>(exp_params["soi"]["modality"]);
        _folder_name = static_cast<std::string>(exp_params["soi"]["archive_folder"]);
        _dimension = std::stoi(exp_params["soi"]["dimension"]);

        if(_folder_name.empty()){
            ROS_ERROR_STREAM("archive folder name is empty");
            exit(1);
        }


        std::stringstream display_params;
        for (auto& param:glob_params) {
            display_params << "\t" << param.first << ":\t" << param.second << std::endl;
        }
        for (auto& param:exp_params) {
            display_params << "\t" << param.first << ":\t" << param.second << std::endl;
        }

        ROS_INFO_STREAM("CLASS_EVAL_NODE : Parameters retrieved:" << std::endl << display_params.str());


        _pointcloud_sub.reset(new Subscriber(ros_nh->subscribe<sensor_msgs::PointCloud2>
                                             (glob_params["cloud_topic"],5, boost::bind(&ReplayBabbling::cloud_callback, this, _1))));


        //*Publisher for visual feedback
        _weighted_cloud_pub.reset(new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>("weighted_cloud", 5)));
        _goal_point_pub.reset(new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>("goal_point", 5)));
        //*/
    }

    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
        if(!cloud_msg)
            return;
        _inputcloud_msg = *cloud_msg;
        _waiting_for_input_cloud = false;
    }

    void client_disconnect_from_ros(){
        _pointcloud_sub.reset();
        _weighted_cloud_pub.reset();
        _goal_point_pub.reset();
    }



    void update(){

        PointCloudT::Ptr input_cloud(new PointCloudT);

        //        if(_waiting_for_input_cloud){
        //            ROS_INFO_STREAM("Waiting for input point_cloud");
        //            return;
        //        }

        pcl::fromROSMsg(_inputcloud_msg, *input_cloud);

        if (input_cloud->empty()) {
            ROS_INFO_STREAM("Waiting for input point_cloud");
            return;
        }

        ROS_INFO_STREAM("iteration " << _counter);


        if(_dataset_file_list.empty())
            return;

        if(!init_classifier()){
            _counter++;
            return;
        }

        _extract_current_target(_target_file_list[_counter]);

        _soi.clear<babbling::sv_param>();
        _soi.setInputCloud(input_cloud);


        if(!_soi.computeSupervoxel(*_workspace)){
            ROS_INFO_STREAM("unable to generate supervoxels");
            return;
        }
        _soi.init_weights(_modality,.5);
        _soi.compute_feature(_modality);
        if(_soi_method == "gmm")
            _soi.compute_weights<iagmm::GMM>(_modality,_gmm_class);
        else if(_soi_method == "nnmap"){
            _nnmap_class.default_estimation = .5;
            _soi.compute_weights<iagmm::NNMap>(_modality,_nnmap_class);
        }

        publish_rviz_feedback();

       ROS_INFO_STREAM(_gmm_class.print_info());

        _counter++;

    }

    void _extract_current_target(const std::string& target_file){
        YAML::Node fileNode = YAML::LoadFile(target_file);
        if(fileNode.IsNull()){
            ROS_ERROR_STREAM("file " << target_file << " not found");
            return;
        }
        ROS_INFO_STREAM("sample retrieve label : " << fileNode["reward"]);


        YAML::Node target_yml = fileNode["target_position"];
        Eigen::Vector3d robot_target;
        robot_target << target_yml[0].as<double>(), target_yml[1].as<double>(), target_yml[2].as<double>();
        babbling::tf_base_conversion(_current_target, robot_target, "/base","/camera_rgb_optical_frame");

    }


    /**
    * @brief load an archive of a previous experiment
    * @return true if something was loaded false otherwise
    */
    bool load_experiment(){
        std::cout << "load : " << _folder_name << std::endl;
        if(_folder_name.empty())
            return false;

        if(!boost::filesystem::exists(_folder_name))
            return false;

        boost::filesystem::directory_iterator dir_it(_folder_name);
        boost::filesystem::directory_iterator end_it;
        std::vector<std::string> split_str;
        std::string type, modality;

        boost::split(split_str,_folder_name,boost::is_any_of("/"));
        boost::split(split_str,split_str.back(),boost::is_any_of("_"));

        modality = split_str[0];

        for(;dir_it != end_it; ++dir_it){
            if(!boost::filesystem::is_directory(dir_it->path().string()))
                continue;
            boost::filesystem::directory_iterator dir_it2(dir_it->path().string());
            boost::split(split_str,dir_it->path().string(),boost::is_any_of("/"));
            boost::split(split_str,split_str.back(),boost::is_any_of("_"));
            int iter = std::stoi(split_str.back());
            for(;dir_it2 != end_it; ++dir_it2){
                boost::split(split_str,dir_it2->path().string(),boost::is_any_of("/"));
                boost::split(split_str,split_str.back(),boost::is_any_of("_"));

                type = split_str[0];
                boost::split(split_str,split_str.back(),boost::is_any_of("."));
                if(type == "gmm" && _soi_method == "gmm")
                    if(split_str[0] == _modality)
                        _gmm_file_list.emplace(iter,dir_it2->path().string());
                if(type == "dataset")
                    if(split_str[0] == _modality)
                        _dataset_file_list.emplace(iter,dir_it2->path().string());
                if(type == "target")
                    _target_file_list.emplace(iter,dir_it2->path().string());
            }
        }

        return true;
    }

    bool init_classifier(){

        if(_soi_method == "gmm"){
            iagmm::GMM gmm;
            if(boost::filesystem::is_empty(_gmm_file_list[_counter])){
                ROS_INFO_STREAM(_gmm_file_list[_counter] << " is empty");
                return false;
            }
            std::ifstream ifs(_gmm_file_list[_counter]);

            boost::archive::text_iarchive iarch(ifs);
            iarch >> gmm;
            _gmm_class = gmm;
        }

        if(boost::filesystem::is_empty(_dataset_file_list[_counter])){
            ROS_INFO_STREAM(_dataset_file_list[_counter] << " is empty");
            return false;
        }

        iagmm::TrainingData data = load_dataset(_dataset_file_list[_counter]);
        _nnmap_class = iagmm::NNMap(_dimension,2,0.3,0.05);
        _nnmap_class.set_samples(data);
        if(_soi_method == "gmm")
            _gmm_class.set_samples(data);

        return true;
    }

    iagmm::TrainingData load_dataset(const std::string& filename)
    {
        iagmm::TrainingData dataset;

        YAML::Node fileNode = YAML::LoadFile(filename);
        if (fileNode.IsNull()) {
            ROS_ERROR("File not found.");
            return dataset;
        }

        YAML::Node features = fileNode["frame_0"]["features"];


        for (unsigned int i = 0; i < features.size(); ++i) {
            std::stringstream stream;
            stream << "feature_" << i;
            YAML::Node tmp_node = features[stream.str()];

            Eigen::VectorXd feature(tmp_node["value"].size());
            for(size_t i = 0; i < tmp_node["value"].size(); ++i)
                feature(i) = tmp_node["value"][i].as<double>();


            dataset.add(tmp_node["label"].as<int>(),feature);
        }
        return dataset;
    }




    bool is_finish(){return _dataset_file_list.empty();}


    void update_workspace()
    {
        XmlRpc::XmlRpcValue wks;

        cafer_core::ros_nh->getParamCached("experiment/workspace", wks);

        _workspace.reset(
                    new workspace_t(true,
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

    /**
     * Visual feedback for RVIZ
     */
    void publish_rviz_feedback()
    {
        sensor_msgs::PointCloud2 weighted_cloud;
        sensor_msgs::PointCloud2 point_goal_msg;
        PointCloudXYZ point_goal;
        PointCloudT w_cl;

        //visualisation of weights distribution
        w_cl =_soi.getColoredWeightedCloud(_modality); //*_background;
        pcl::toROSMsg(w_cl, weighted_cloud);
        weighted_cloud.header = _inputcloud_msg.header;

        //visualisation of goal point
        point_goal.push_back(pcl::PointXYZ(_current_target(0), _current_target(1), _current_target(2)));
        pcl::toROSMsg(point_goal, point_goal_msg);
        point_goal_msg.header = _inputcloud_msg.header;

        _weighted_cloud_pub->publish(weighted_cloud);
        _goal_point_pub->publish(point_goal_msg);
    }

private:

    SurfaceOfInterest _soi;

    std::unique_ptr<Subscriber> _pointcloud_sub;
    sensor_msgs::PointCloud2 _inputcloud_msg;
    bool _waiting_for_input_cloud = true;

    iagmm::NNMap _nnmap_class; /**< nnmap classifiers */
    iagmm::GMM _gmm_class; /**< gmm classifiers */


    bool new_iter = true;
    std::string _folder_name;
    int _counter = 0;
    std::map<std::string,std::array<double,6>> _results;

    std::string _soi_method; /**< which classifier will be use to compute the saliency map : expert, nnmap, gmm, random or sift */
    std::string _modality;
    int _dimension;

    std::map<int,std::string> _gmm_file_list;
    std::map<int,std::string> _dataset_file_list;
    std::map<int,std::string> _target_file_list;
    Eigen::Vector3d _current_target;

    std::unique_ptr<workspace_t> _workspace;


    //*Publisher for visual feedback (to be commented if don't needed)
    std::unique_ptr<ros::Publisher> _weighted_cloud_pub;
    std::unique_ptr<ros::Publisher> _goal_point_pub;
    std::unique_ptr<Publisher> _soi_classifier_pub;
    //*/



};

std::string parse_arg(int& argc, char **& argv, const std::string& default_val)
{
    std::string key;
    std::string value;
    std::string temp_str;
    std::string::size_type res;

    key = "__name:=";

    for (unsigned short i = 0; i < argc; ++i) {
        temp_str = argv[i];
        res = temp_str.find(key);

        if (res != std::string::npos) {
            value = temp_str.erase(res, key.length());
            break;
        }
        else if (i == argc - 1) {
            value = default_val;
        }
    }
    return value;
}

int main(int argc, char** argv){

    std::string management_topic;
    std::string cafer_type;
    std::string node_name;
    std::string freq;
    std::string output_file;

    node_name = parse_arg(argc, argv, "replay_babbling");

    cafer_core::init(argc, argv, node_name);

    cafer_core::ros_nh->getParam("/dream_babbling/management_topic", management_topic);

    cafer_core::ros_nh->getParam("/cafer/cafer_type", cafer_type);
    cafer_core::ros_nh->getParam("/cafer/mgmt_topic_pub_freq", freq);

    cafer_core::ros_nh->getParam("experiment/soi/output_file",output_file);

    ReplayBabbling c_eval(management_topic, cafer_type, 10.0);

    c_eval.wait_for_init();


    while (ros::ok() && (!c_eval.get_terminate()) && !c_eval.is_finish()) {
        c_eval.spin();
        c_eval.update();
        c_eval.sleep();
    }

    return 0;

}
