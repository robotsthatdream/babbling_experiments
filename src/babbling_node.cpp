//
// Created by phlf on 24/03/16.
//

#include <iostream>
#include <math.h>
#include <thread>
#include <chrono>
#include <string>
#include <unistd.h>
#include <time.h>

#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

#include <cafer_core/cafer_core.hpp>

#include <image_processing/SurfaceOfInterest.h>
#include <image_processing/DescriptorExtraction.h>
#include <image_processing/HistogramFactory.hpp>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <iagmm/gmm.hpp>
#include <iagmm/nnmap.hpp>
#include <iagmm/mcs.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Bool.h>
#include <actionlib/client/simple_action_client.h>

#include <yaml-cpp/yaml.h>

#include <dream_babbling/pose_goalAction.h>

#include <globals.h>

#include <relevance_map/utilities.hpp>
#include <relevance_map/relevance_map_node.hpp>
#include <relevance_map/score_computation.hpp>

#include <dream_babbling/is_moving.h>
#include <dream_babbling/dataset.h>
#include <dream_babbling/gmm_archive.h>
#include <geometry_msgs/Point.h>
#include <dream_babbling/target_info.h>

#include <Eigen/Core>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/algorithm/string.hpp>

#include <std_srvs/Empty.h>

using namespace cafer_core;
using namespace rgbd_utils;
namespace ip = image_processing;
using namespace dream_babbling;
using namespace Eigen;
using namespace babbling;
namespace rm = relevance_map;

/**
 * @brief The Babbling class
 * Main node of the babbling experiment. This node handle video inputs, update of classifiers,
 */
class Babbling : public Component, public rm::relevance_map_node {
    using Component::Component;

public :

    ~Babbling()
    {
        ROS_ERROR("BABBLING_NODE : Destroy?");
        client_disconnect_from_ros();
    }

    /**
     * @brief client_connect_to_ros
     * Initialisation of all subscribers, publishers, servers and clients.
     */
    void client_connect_to_ros() override
    {
        initialize(ros_nh);


        //* Retrieve parameters
        XmlRpc::XmlRpcValue glob_params; // parameters stored in global_params.yml
        XmlRpc::XmlRpcValue exp_params; // parameters stored in experiment_params.yml
        XmlRpc::XmlRpcValue datamanager_topics;
        XmlRpc::XmlRpcValue no_sim;


        cafer_core::ros_nh->getParam("/dream_babbling/params", glob_params);
        cafer_core::ros_nh->getParam("experiment", exp_params);
        ros_nh->getParam("/dream_babbling/babbling/topics/data",datamanager_topics);
        ros_nh->getParam("no_sim",no_sim);


        _nb_iter = std::stoi(exp_params["soi"]["number_of_iteration"]);
        _max_trials = std::stoi(exp_params["soi"]["max_trials"]);
        _no_sim = no_sim;
        _robot = static_cast<std::string>(glob_params["robot"]);

        //*/

        //* Output the values of all parameters
        std::stringstream display_params;
        for (auto& param:glob_params) {
            display_params << "\t" << param.first << ":\t" << param.second << std::endl;
        }
        for (auto& param:exp_params) {
            display_params << "\t" << param.first << ":\t" << param.second << std::endl;
        }
        for (auto& param:datamanager_topics) {
            display_params << "\t" << param.first << ":\t" << param.second << std::endl;
        }
        ROS_INFO_STREAM("BABBLING_NODE : Parameters retrieved:" << std::endl << display_params.str());
        //*/



        // Initialisation of action client for synchronisation with the controler node
        _client_controller.reset(
                new actionlib::SimpleActionClient<pose_goalAction>("controller_node/"+static_cast<std::string>(glob_params["controller_server"]), false));
        _controller_status_sub.reset(
                    new Subscriber(ros_nh->subscribe<actionlib_msgs::GoalStatusArray>(
                                       "/dream_babbling/controller_node/" + static_cast<std::string>(glob_params["controller_server"]) + "/status",5,&Babbling::_controller_status_cb,this)));


        // Initialisation of client for commucating with the motion detector node
        _client_motion.reset(
                new ros::ServiceClient(ros_nh->serviceClient<is_moving>(glob_params["motion_detector_service"])));


        //*Publisher for visual feedback in rviz
        _terminated.reset(new Publisher(ros_nh->advertise<std_msgs::Bool>("babbling_terminated",5)));

        _goal_point_pub.reset(new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>("goal_point", 5)));
        _input_cloud_pub.reset(new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>("input_cloud",5)));
        //*/

        //Publishers to send the training dataset to the data manager node
        for(const auto& mod : _modalities)
            _dataset_pub.emplace(mod.first,std::unique_ptr<Publisher>(
                                     new Publisher(ros_nh->advertise<dataset>(glob_params["dataset_topic_"+mod.first], 5))));

        //Publishers to send the GMM Classifier archive to the data manager node
        for(const auto& mod : _modalities)
            _gmm_arch_pub.emplace(mod.first,std::unique_ptr<Publisher>(
                                      new Publisher(ros_nh->advertise<gmm_archive>(glob_params["gmm_topic_"+mod.first],5))));


        //publisher to publish the information if the iteration is finish or not
        _is_finish_pub.reset(new Publisher(ros_nh->advertise<std_msgs::Bool>("is_finish",5)));

        //DB Manager clients for synchronisation
        _db_request_publisher.reset(new Publisher(ros_nh->advertise<cafer_core::DBManager>(
                static_cast<std::string>(glob_params["database_server"]) + "/request", 10)));
        _db_status_subscriber.reset(new Subscriber(ros_nh->subscribe<cafer_core::DBManager>(
                static_cast<std::string>(glob_params["database_server"]) + "/status", 10,
               boost::bind(&Babbling::_done_callback, this, _1))));


        _target_point_pub.reset(
                            new Publisher(ros_nh->advertise<geometry_msgs::PointStamped>(glob_params["target_topic"],5)));
        _target_info_pub.reset(
                    new Publisher(ros_nh->advertise<dream_babbling::target_info>(datamanager_topics["target_info"],5)));

    }

    /**
     * @brief initialisation method
     */
    void init() override
    {

        //* init the attributes
        _is_init = true;
        _robot_controller_ready = false;
        _db_ready = true;
        _background_saved = false;

//        if(!_load_exp.empty()){
//            std::vector<std::string> strs;
//            boost::split(strs,_load_exp,boost::is_any_of("/"));
//            boost::split(strs,strs[strs.size()-2],boost::is_any_of("_"));

//            try{
//                _counter_iter = std::stoi(strs.back());
//            }catch(...){
//                _counter_iter = 0;
//            }
//        }else _counter_iter = 0;

        _target_point.resize(3);
        _target_point[0] = 0; _target_point[1] = 0; _target_point[2] = 0;
        _last_number_of_samples = 0;
        //*/

        _controller_is_up = _mode == "fake_controller";
        _robot_controller_ready  = _mode == "fake_controller";

        client_connect_to_ros();
        init_classifiers(_load_exp);
    }



    /**
     * @brief Destroy all the publishers, subscribers, servers and clients.
     */
    void client_disconnect_from_ros() override
    {
        cafer_core::DBManager db_request;
        std::string supervisor_name =
                ros::names::parentNamespace(cafer_core::ros_nh->getNamespace()) + "/babbling";

        //PLACEHOLDER: This is necessary as the supervisor node is supposed to do this...
        ClientDescriptor supervisor;
        if (find_by_name(supervisor_name, supervisor)) {
            db_request.type = static_cast<uint8_t>(DatabaseManager::Request::STOP_RECORDING);
            db_request.id = supervisor.id;
            _db_request_publisher->publish(db_request);
        }
        else {
            ROS_ERROR_STREAM("BABBLING_NODE : Unable to find supervisor at: " << supervisor_name);
        }
        _is_finish_pub.reset();
        _input_cloud_pub.reset();

        for(auto& pub : _dataset_pub)
            pub.second.reset();
        for(auto& pub : _gmm_arch_pub)
            pub.second.reset();


        _target_point_pub.reset();
        _target_info_pub.reset();
        _goal_point_pub.reset();
        _target_point_pub.reset();
        _target_info_pub.reset();
        _terminated.reset();

        release();
    }

    /**
     * @brief Main function of the node.
     * Update all states according of video inputs and the other nodes.
     */
    void update() override
    {
        std::string supervisor_name =
                ros::names::parentNamespace(cafer_core::ros_nh->getNamespace()) + "/babbling";
        bool no_input_cloud = true;
        cafer_core::DBManager db_request;
        Vector3d pose_camera, pose_robot, normal_camera, normal_robot /*center_point_camera, center_point_robot*/;
        geometry_msgs::PointStamped target_msg;
        pose_robot(0) = 0; pose_robot(1) = 0; pose_robot(2) = 0;


        ip::PointCloudT::Ptr input_cloud(new ip::PointCloudT);
        if(!retrieve_input_cloud(input_cloud))
            return;
        else no_input_cloud = false;

        if (!no_input_cloud) {

            //* EXPERT MODE
            if ((_method == "expert" || _mode == "experiment" || _mode == "fake_controller") && !_background_saved) {
                std::cout << "BABBLING_NODE : take the background" << std::endl;
                _background.reset(new ip::PointCloudT);
                *_background = *input_cloud;
                std::cout << "BABBLING_NODE : done" << std::endl;

                std::cout << "BABBLING_NODE : Press enter to start.";
                std::cin.ignore();
                _background_saved = true;
                return;
            }
            //*/

            //* Waiting for the controller Node to be up
            if(_controller_is_up){
                _robot_controller_ready = true;
                _controller_is_up = false;
                _controller_status_sub.reset();
            }

            //* SEND THE POSITION TARGET TO THE CONTROLLER NODE
            if (_robot_controller_ready && _db_ready) {

                ROS_INFO_STREAM("BABBLING : controller and db_manager are ready. Start next iteration");
                //Goal for the _robot controller
//                std::vector<int> indexes;
//                pcl::removeNaNFromPointCloud(*input_cloud,*input_cloud,indexes);

                if(!_compute_supervoxels(input_cloud)){
                    ROS_INFO_STREAM("BABBLING_NODE : unable to compute supervoxels");
                    return;
                }
                if (!_compute_relevance_map()) {
                    ROS_INFO_STREAM("BABBLING_NODE : unable to compute relevance map");
                    return;
                }
                if(!_compute_choice_map(_sv,_lbl)){
                    ROS_INFO_STREAM("BABBLING_NODE : unable to choose next area to explore");
                    return;
                }

                if(_mode != "fake_controller")
                {
                    pose_camera << _sv.centroid_.x, _sv.centroid_.y, _sv.centroid_.z;
                    normal_camera << _sv.normal_.normal[0], _sv.normal_.normal[1], _sv.normal_.normal[2];
                    std::string camera_frame, base_frame;
                    ROS_WARN_STREAM("BABBLING_NODE : Robot name is : " << _robot);
                    rm::define_frames(_robot, base_frame, camera_frame);

                    babbling::tf_base_conversion(normal_robot, normal_camera, camera_frame, base_frame);

                    babbling::tf_base_conversion(pose_robot, pose_camera, camera_frame, base_frame);

                    _poseGoal.header = _images_sub->get_depth().header;
                    _poseGoal.target_pose.resize(3);
                    _poseGoal.target_pose[0] = pose_robot(0);
                    _poseGoal.target_pose[1] = pose_robot(1);
                    _poseGoal.target_pose[2] = pose_robot(2);

                    _poseGoal.target_normal.resize(3);
                    _poseGoal.target_normal[0] = normal_robot(0);
                    _poseGoal.target_normal[1] = normal_robot(1);
                    _poseGoal.target_normal[2] = normal_robot(2);

                    _target_point = _poseGoal.target_pose;




                    ROS_INFO_STREAM(
                                "BABBLING_NODE : pose in camera in frame " << pose_camera[0] << " " << pose_camera[1] << " " << pose_camera[2]);



                    ROS_INFO_STREAM(
                                "BABBLING_NODE : Going to pose " << _poseGoal.target_pose[0] << " " << _poseGoal.target_pose[1] <<
                                                                                               " " << _poseGoal.target_pose[2]);
                }
                //* Enable recording to DB
                //PLACEHOLDER: This is necessary as the supervisor node is supposed to do this...
                ClientDescriptor supervisor;
                if (find_by_name(supervisor_name, supervisor)) {
                    db_request.type = static_cast<uint8_t>(DatabaseManager::Request::RECORD_DATA);
                    db_request.name = "/dream_babbling/babbling";
                    db_request.id = supervisor.id;
                    _db_request_publisher->publish(db_request);
                }
                else {
                    ROS_ERROR_STREAM("BABBLING_NODE : Unable to find supervisor at: " << supervisor_name);
                }
                //*/

                if(_mode != "fake_controller")
                    _client_controller->sendGoal(_poseGoal);

                _publish_rviz_feedback();
                _nb_trials++;
                if(_mode == "fake_controller") _nb_trials = _max_trials;

                if(_mode == "experiment"){
                    std::string output_file;
                    cafer_core::ros_nh->getParam("experiment/soi/output_file",output_file);
                    rm::score_computation<Babbling> sc(this,output_file);
                    sc.compute_scores_results(_counter_iter,_nb_false_pos,_nb_false_neg);
                }

                if(_counter_iter > 0){
                    if(_method == "gmm"){
                        if(_gmm_class[_modality].get_samples().size() > _last_number_of_samples){
                            for(auto& classifier: _gmm_class){
                                classifier.second.update();
//                                ROS_INFO_STREAM(classifier.second.print_info());
                            }
                            _last_number_of_samples = _gmm_class[_modality].get_samples().size();
                        }
                    }else if(_method == "mcs"){
                        if(_mcs.get_nb_samples() > _last_number_of_samples){
                            _mcs.update();
                            _last_number_of_samples = _mcs.get_nb_samples();
                        }
                    }
                }
                _counter_iter++;

                _db_ready = false;
                _robot_controller_ready = false;


            } else if(_robot_controller_ready && !_db_ready){
                ROS_INFO_STREAM("Waiting for db_manager");
                return;
            }
            //*/

//            if(_target_point.size() != 0){
//                target_msg.point.x = _target_point[0];
//                target_msg.point.y = _target_point[1];
//                target_msg.point.z = _target_point[2];
//                target_msg.header = _images_sub->get_depth().header;
//                _target_point_pub->publish(target_msg);
//            }




            //* Retrieve the state of the controller
            if(_mode != "fake_controller"){
                _client_controller->waitForResult(ros::Duration(1.0));
                if (_client_controller->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    _controller_succeed = true;
                else if(_client_controller->getState() == actionlib::SimpleClientGoalState::ABORTED)
                    _controller_aborted = true;
            }else _controller_succeed = true;

           if(_controller_succeed)
               if ((_method == "nnmap" || _method == "gmm" || _method == "mcs")
                       && (_mode == "exploration" || _mode == "experiment" || _mode == "fake_controller"))
                       _call_motion_service();

           if((_is_neg || _controller_aborted) && _nb_trials < _max_trials){
               ROS_INFO_STREAM("BABBLING : RETRY same position");
               _nb_trials++;
                _client_controller->sendGoal(_poseGoal);
                _is_neg = false;
                _controller_aborted = false;
                _controller_succeed = false;
               return;
           }

            if (_controller_succeed) {
                _target_info_pub->publish(_target_info_msg);

                _controller_succeed = false;


                //* Publish SOI classifier data for the DB Manager
                if(_method == "nnmap"){
                    for(const auto& classifier: _nnmap_class)
                        if (classifier.second.dataset_size() != 0)
                            _dataset_pub[classifier.first]->publish(training_data_to_ros_msg(
                                                                        classifier.first,
                                                                        classifier.second.get_samples()));
                }
                else if(_method == "gmm")
                {
                    for(const auto& classifier: _gmm_class){
                        if (classifier.second.dataset_size() != 0){
                            _dataset_pub[classifier.first]->publish(training_data_to_ros_msg(
                                                                        classifier.first,
                                                                        classifier.second.get_samples()));
                            std::stringstream sstream;
                            boost::archive::text_oarchive oarch(sstream);
                            oarch << classifier.second;
                            dream_babbling::gmm_archive msg;
                            msg.archive.data = sstream.str();
                            msg.type.data = classifier.first;
                            _gmm_arch_pub[classifier.first]->publish(msg);
                        }
                    }
                }
                else if(_method == "mcs"){
                    for(const auto& classifier: _mcs.access_classifiers()){
                        if (classifier.second->dataset_size() != 0){
                            _dataset_pub[classifier.first]->publish(training_data_to_ros_msg(
                                                                        classifier.first,
                                                                        classifier.second->get_samples()));
                            std::stringstream sstream;
                            boost::archive::text_oarchive oarch(sstream);
                            oarch << *(dynamic_cast<iagmm::GMM*>(classifier.second.get()));
                            dream_babbling::gmm_archive msg;
                            msg.archive.data = sstream.str();
                            msg.type.data = classifier.first;
                            _gmm_arch_pub[classifier.first]->publish(msg);
                        }
                    }
                }
                //*/

                //Wait for the DB to receive data.
                std::this_thread::sleep_for(std::chrono::milliseconds(150));

                _robot_controller_ready = true;
                //* Disable recording to DB
                //PLACEHOLDER: This is necessary as the supervisor node is supposed to do this...
                ClientDescriptor supervisor;
                if (find_by_name(supervisor_name, supervisor)) {
                    db_request.type = static_cast<uint8_t>(DatabaseManager::Request::STOP_RECORDING);
                    db_request.name = "/dream_babbling/babbling";
                    db_request.id = supervisor.id;
                    _db_request_publisher->publish(db_request);
                }
                else {
                    ROS_ERROR_STREAM("BABBLING_NODE : Unable to find supervisor at: " << supervisor_name);
                }
                //*/

                ROS_INFO_STREAM("BABBLING_NODE : Position reached.");
                _reset_world = true;
                _nb_trials = 0;
                _is_neg = false;


//                if(_db_ready && _method == "gmm" && _mode == "exploration")
//                    for(auto& classifier: _gmm_class)
//                        classifier.second.update_model();

//                if(_method == "gmm")
//                    for(auto& classifier: _gmm_class)
//                        ROS_INFO_STREAM("BABBLING NODE : " << classifier.first << " "  << classifier.second.print_info());

            }

            else if (_controller_aborted) {
                std::cout << "BABBLING_NODE : position wasn't reachable" << std::endl;
                _controller_aborted = false;
                _robot_controller_ready = true;

                //PLACEHOLDER: This is necessary as the supervisor node is supposed to do this...
                ClientDescriptor supervisor;
                if (find_by_name(supervisor_name, supervisor)) {
                    db_request.type = static_cast<uint8_t>(DatabaseManager::Request::STOP_RECORDING);
                    db_request.name = "/dream_babbling/babbling";
                    db_request.id = supervisor.id;
                    _db_request_publisher->publish(db_request);
                }
                else {
                    ROS_ERROR_STREAM("BABBLING_NODE : Unable to find supervisor at: " << supervisor_name);
                }
                _nb_trials = 0;
                _is_neg = false;
            }
            else {
                ROS_INFO_STREAM("BABBLING_NODE : Waiting for controller...");
            }

        }
        publish_terminated();
    }

    void publish_terminated(){
        std_msgs::Bool terminated;
        terminated.data = is_finish();
        _terminated->publish(terminated);
    }

    bool is_finish()
    {
        return _last_number_of_samples>=_nb_iter/* || !_possible_choice*/;
    }


private:
    std::unique_ptr<ros::ServiceClient> _client_motion; /**<client to recieve information from the motion detector*/
    std::unique_ptr<actionlib::SimpleActionClient<pose_goalAction>> _client_controller;/**<action client to communicate with the controller*/
    std::unique_ptr<Subscriber> _controller_status_sub;


    std::unique_ptr<Publisher> _db_request_publisher; /**<publisher to publish a request to the DB Manager*/
    std::unique_ptr<Subscriber> _db_status_subscriber; /**<subscriber to retrieve the status of the DB Manager*/
    std::unique_ptr<Publisher> _target_info_pub;
    std::unique_ptr<Publisher> _target_point_pub;

    pose_goalGoal _poseGoal;

    dream_babbling::target_info _target_info_msg;


    std::unique_ptr<Publisher> _is_finish_pub;
    std::unique_ptr<Publisher> _terminated;

    std::string _robot;

    pcl::Supervoxel<ip::PointT> _sv; /**< current supervoxel chosen for exploration */
    uint32_t _lbl; /**< label of the current supervoxel */

    bool _is_neg = false;

    std::vector<std::string> _mcs_mod_mapping;

    bool _robot_controller_ready;
    bool _controller_is_up = false;
    bool _controller_succeed = false;
    bool _controller_aborted = false;
    bool _db_ready;
    bool _waiting_for_input_cloud = true;
    bool _possible_choice = true;
    bool _reset_world = true;
    bool _motion_checked = true;
    bool _no_sim;

    std::vector<double> _target_point;

    int _last_number_of_samples = 0;
    int _counter_iter = 0;
    int _nb_iter;
    int _max_trials;
    int _nb_trials = 0;

    int _nb_false_pos = 0;
    int _nb_false_neg = 0;

    boost::random::mt19937 gen;

    //Publisher for storing dataset
    std::map<std::string,std::unique_ptr<Publisher>> _dataset_pub;
    std::map<std::string,std::unique_ptr<Publisher>> _gmm_arch_pub;


    //*Publisher for visual feedback (to be commented if don't needed)
    std::unique_ptr<Publisher> _goal_point_pub;
    std::unique_ptr<Publisher> _input_cloud_pub;
    //*/

    void _controller_status_cb(const actionlib_msgs::GoalStatusArrayConstPtr& msg){
        _controller_is_up = true;
    }


    /**
     * @brief Visual feedback for RVIZ
     */
    void _publish_rviz_feedback()
    {
        publish_feedback();

        //visualisation of goal point
        sensor_msgs::PointCloud2 point_goal_msg;
        ip::PointCloudXYZ point_goal;
        point_goal.push_back(pcl::PointXYZ(_sv.centroid_.x, _sv.centroid_.y, _sv.centroid_.z));
        pcl::toROSMsg(point_goal, point_goal_msg);
        point_goal_msg.header = _images_sub->get_depth().header;
        _goal_point_pub->publish(point_goal_msg);
    }




    /**
     * Ask motion_detector if the scene has changed between first and last iteration's frames.
     */
    void _call_motion_service()
    {
        ROS_INFO_STREAM("BABBLING_NODE : Enter in call_motion_service");
        int label = 0;
        cv::Mat mask;
        sensor_msgs::Image mask_msg;
        cv_bridge::CvImage converter;
        dream_babbling::is_moving srv;
        //        _sv_checked.clear();

//        _soi.supervoxel_to_mask(_lbl, mask);

//        converter.image = mask;
//        converter.encoding = sensor_msgs::image_encodings::MONO8;
//        converter.toImageMsg(mask_msg);
        if(_mode != "fake_controller"){
            srv.request.supervoxel_center.x = _sv.centroid_.x;
            srv.request.supervoxel_center.y = _sv.centroid_.y;
            srv.request.supervoxel_center.z = _sv.centroid_.z;
            sensor_msgs::PointCloud2 sv_msg;
            pcl::toROSMsg(*(_sv.voxels_),sv_msg);
            srv.request.supervoxel_cloud = sv_msg;
        }

        if (_mode == "fake_controller" || _client_motion->call(srv)) {
            if ((_mode == "fake_controller" && rm::is_in_cloud(_sv.centroid_,_background))
                    || srv.response.has_moved.data == 0) {
                label = 0;
                _is_neg = true;
                ROS_INFO_STREAM("BABBLING_NODE : negative sample");
            }
            else {
                label = 1;
                _is_neg = false;
//                ip::AdjacencyMap adj_map = _soi.getAdjacencyMap();
//                auto ite_range = adj_map.equal_range(_lbl);

//                for(auto it = ite_range.first; it != ite_range.second; it++){
//                    if(!is_in_cloud(_soi.getSupervoxels()[it->second]->centroid_,_background)){
//                        if(_method == "nnmap"){
//                            for(auto& classifier : _nnmap_class){
//                                ROS_INFO_STREAM(classifier.first);
//                                classifier.second.add(_soi.get_feature(it->second,classifier.first),1);
//                                ROS_INFO_STREAM("BABBLING_NODE : size of dataset : " << classifier.second.dataset_size());
//                            }

//                        }
//                        else if(_method == "gmm"){
//                            for(auto& classifier : _gmm_class){
//                                ROS_INFO_STREAM(classifier.first);
//                                int ind = classifier.second.append(_soi.get_feature(it->second,classifier.first),1);
//                                classifier.second.update_model(ind,1);
//                                ROS_INFO_STREAM("BABBLING_NODE : size of dataset : " << classifier.second.dataset_size());
//                            }
//                        }
//                        else if(_method == "mcs"){
//                            _mcs.add(_soi.get_features(it->second),1);
//                            _mcs.update();
//                        }
//                    }
//                }

                ROS_INFO_STREAM("BABBLING_NODE : positive sample");
            }
            if(_nb_trials >= _max_trials || !_is_neg){
                _target_info_msg.reward = label;
                _target_info_msg.target_position.x = _target_point[0];
                _target_info_msg.target_position.y = _target_point[1];
                _target_info_msg.target_position.z = _target_point[2];

                if(_mode=="experiment"){
                    if(rm::is_in_cloud(_sv.centroid_,_background) && label == 1)
                        _nb_false_pos++;
                    if(!rm::is_in_cloud(_sv.centroid_,_background) && label == 0)
                        _nb_false_neg++;
                }
                _motion_checked = true;

                //            _mcs.update_parameters(label);
                if(_method == "nnmap"){
                    for(auto& classifier : _nnmap_class){
                        ROS_INFO_STREAM(classifier.first);
                        classifier.second.add(_soi.get_feature(_lbl,classifier.first),label);
                        ROS_INFO_STREAM("BABBLING_NODE : size of dataset : " << classifier.second.dataset_size());
                    }
                }
                else if(_method == "gmm"){
                    for(auto& classifier : _gmm_class){
                        ROS_INFO_STREAM(classifier.first);
                        classifier.second.add(_soi.get_feature(_lbl,classifier.first),label);
                        ROS_INFO_STREAM("BABBLING_NODE : size of dataset : " << classifier.second.dataset_size());
                    }
                }
                else if(_method == "mcs"){
                    _mcs.add(_soi.get_features(_lbl),label);
                }
            }

        }
        else { ROS_WARN_STREAM("BABBLING_NODE : unable to contact service motion detection"); }
    }

    void _done_callback(const cafer_core::DBManagerConstPtr& status_msg)
    {
        std::string supervisor_name =
                ros::names::parentNamespace(cafer_core::ros_nh->getNamespace()) + "/babbling";
        //PLACEHOLDER: This is necessary as the supervisor node is supposed to do this...
        ClientDescriptor supervisor;

        if (find_by_name(supervisor_name, supervisor)) {
            if (status_msg->id == supervisor.id) {
                if (static_cast<DatabaseManager::Response>(status_msg->type) ==
                    DatabaseManager::Response::STATUS_READY) {
                    _db_ready = true;
                    std_msgs::Bool is_finish;
                    is_finish.data = 1;
                    _is_finish_pub->publish(is_finish);
                }
            }
        }
        else {
            ROS_ERROR_STREAM("BABBLING_NODE : Unable to find supervisor at: " << supervisor_name);
        }


    }







};

int main(int argc, char** argv)
{
    std::string node_name;
//    std::string output_file;
    XmlRpc::XmlRpcValue cafer;


    tbb::task_scheduler_init init;

    node_name = global::parse_arg(argc, argv, "babbling_node");

    cafer_core::init(argc, argv, node_name);
    cafer_core::ros_nh->getParam("cafer", cafer);
//    cafer_core::ros_nh->getParam("experiment/soi/output_file",output_file);

    Babbling babbling(cafer["mgmt"], cafer["type"], cafer["freq"], cafer["uuid"]);

    babbling.wait_for_init();
//    std::this_thread::sleep_for(std::chrono::seconds(3));
    babbling.spin();

    while (ros::ok() && (!babbling.get_terminate()) && !babbling.is_finish()) {
        babbling.spin();
        babbling.update();
        babbling.sleep();
    }

    for(int i = 0 ; i < 2; ++i ){
        babbling.spin();
        babbling.publish_terminated();
        babbling.sleep();
    }

//    if(!babbling.write_results(output_file))
//        ROS_ERROR_STREAM("unable to open " << output_file);

    ROS_INFO_STREAM("BABBLING : ros::ok = " << ros::ok()
                    << " terminate = " << babbling.get_terminate()
                    << " is finish = " << babbling.is_finish());

    ROS_INFO_STREAM("BABBLING : is finish");

    return 0;
}
