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

#include <relevance_map/utilities.hpp>
#include <relevance_map/relevance_map_node.hpp>
#include <relevance_map/score_computation.hpp>

#include <boost/archive/text_iarchive.hpp>

using namespace cafer_core;
using namespace image_processing;
using namespace rgbd_utils;
namespace rm = relevance_map;


class ClassifierEval : public rm::relevance_map_node{

public:
    ClassifierEval(){
        initialize(ros_nh);

        std::string folder_name;

        XmlRpc::XmlRpcValue exp_params; // parameters stored in experiment_params.yml
        cafer_core::ros_nh->getParam("experiment", exp_params);

        folder_name = static_cast<std::string>(exp_params["soi"]["archive_folder"]);
        _output_file = static_cast<std::string>(exp_params["soi"]["output_file"]);

        _with_bluring = std::stoi(exp_params["soi"]["with_bluring"]);
        _increment_bluring = std::stod(exp_params["soi"]["increment"]);
        _with_adaptive_thres = std::stoi(exp_params["soi"]["with_adpative_thres"]);

        if(folder_name.empty()){
            ROS_ERROR_STREAM("archive folder name is empty");
            exit(1);
        }

        _counter = 0;

        if(!load_folder(folder_name,_archive_folders)){
            ROS_ERROR_STREAM("unable to load " << folder_name);
        }
    }

    ~ClassifierEval(){
        release();
    }

    bool load_folder(const std::string& folder_name, std::queue<std::string>& folder_list){
        if(folder_name.empty())
            return false;

        if(!boost::filesystem::exists(folder_name)){
            ROS_ERROR_STREAM(folder_name << " does not exist");
            return false;
        }

        boost::filesystem::directory_iterator dir_it(folder_name);
        boost::filesystem::directory_iterator end_it;

        for(;dir_it != end_it; ++dir_it){
            if(!boost::filesystem::is_directory(dir_it->path().string()))
                continue;

            folder_list.push(dir_it->path().string());
        }

        if(folder_list.empty()){
            ROS_ERROR_STREAM(folder_name << " is empty");
            return false;
        }


        return true;
    }

    void init_next_archive(){
        rm::load_archive(_archive_folders.front(),_ite_folders);

        _output_path = _archive_folders.front() + "/" + _output_file;

        _archive_folders.pop();
        _counter = 0;
    }

    void update(){

        PointCloudT::Ptr input_cloud(new PointCloudT);
        if(!retrieve_input_cloud(input_cloud))
            return;


        if (!_background_saved) {

            std::cout << "take the background" << std::endl;
            _background.reset(new ip::PointCloudT);
            *_background = *input_cloud;
            std::cout << "done" << std::endl;

            std::cout << "Press enter to start.";
            std::cin.ignore();
            _background_saved = true;

            return;
        }

        if(_ite_folders.empty())
            init_next_archive();

        init_classifiers(_ite_folders.front());
        _ite_folders.pop();
        _compute_supervoxels(input_cloud);
        _compute_relevance_map();
        if(_with_bluring)
            _soi.neighbor_bluring(_modality,_increment_bluring,1);
        if(_with_adaptive_thres)
            _soi.adaptive_threshold(_modality,1);
        rm::score_computation<ClassifierEval> sc(this, _output_path);

        sc.compute_scores_results(_counter);
        _counter++;
        publish_feedback();
    }

    bool is_finish(){return _archive_folders.empty() && _ite_folders.empty();}

private:

    pcl::Supervoxel<PointT> _sv;
    uint32_t _lbl;

    int _counter = 0;
    int _with_bluring;
    int _with_adaptive_thres;
    double _increment_bluring;

    std::string _output_path;
    std::string _output_file;
    std::queue<std::string> _archive_folders;
    std::queue<std::string> _ite_folders;

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


    std::string node_name;

    node_name = parse_arg(argc, argv, "classifier_eval");

    cafer_core::init(argc, argv, node_name);

    ClassifierEval c_eval;

    while (ros::ok() && !c_eval.is_finish()) {
        c_eval.update();
        ros::spinOnce();
        usleep(500);
    }


    return 0;

}
