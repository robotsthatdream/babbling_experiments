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


        XmlRpc::XmlRpcValue wks;
        ros_nh->getParam("experiment/workspace", wks);

        babbling::init_workspace(wks,_workspace);


        cafer_core::ros_nh->getParam("/dream_babbling/params", glob_params);
        cafer_core::ros_nh->getParam("experiment", exp_params);
        ros_nh->getParam("/dream_babbling/babbling/topics/data",datamanager_topics);
        ros_nh->getParam("no_sim",no_sim);


        _nb_iter = std::stoi(exp_params["soi"]["number_of_iteration"]);
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



        //publisher to publish the information if the iteration is finish or not
        _is_finish_pub.reset(new Publisher(ros_nh->advertise<std_msgs::Bool>("is_finish",5)));


        _target_point_pub.reset(
                            new Publisher(ros_nh->advertise<geometry_msgs::PointStamped>(glob_params["target_topic"],5)));


    }

    /**
     * @brief initialisation method
     */
    void init() override
    {

        //* init the attributes
        _is_init = true;
        _robot_controller_ready = false;

        _target_point.resize(3);
        _target_point[0] = 0; _target_point[1] = 0; _target_point[2] = 0;
        _last_number_of_samples = 0;
        //*/


        client_connect_to_ros();
        init_classifiers(_load_exp);
    }



    /**
     * @brief Destroy all the publishers, subscribers, servers and clients.
     */
    void client_disconnect_from_ros() override
    {

        _is_finish_pub.reset();
        _input_cloud_pub.reset();
;


        _target_point_pub.reset();
        _target_info_pub.reset();
        _goal_point_pub.reset();
        _target_point_pub.reset();
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
        pose_goalGoal poseGoal;
        Vector3d pose_camera, pose_robot, normal_camera, normal_robot /*center_point_camera, center_point_robot*/;
        geometry_msgs::PointStamped target_msg;
        pose_robot(0) = 0; pose_robot(1) = 0; pose_robot(2) = 0;


        ip::PointCloudT::Ptr input_cloud(new ip::PointCloudT);
        if(!retrieve_input_cloud(input_cloud))
            return;
        else no_input_cloud = false;

        if (!no_input_cloud) {


            //* Waiting for the controller Node to be up
            if(_controller_is_up){
                _robot_controller_ready = true;
                _controller_is_up = false;
                _controller_status_sub.reset();
            }

            //* SEND THE POSITION TARGET TO THE CONTROLLER NODE
            if (_robot_controller_ready) {
//            if (_robot_controller_ready) {

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


                pose_camera << _sv.centroid_.x, _sv.centroid_.y, _sv.centroid_.z;
                normal_camera << _sv.normal_.normal[0], _sv.normal_.normal[1], _sv.normal_.normal[2];
                std::string camera_frame, base_frame;
                ROS_WARN_STREAM("BABBLING_NODE : Robot name is : " << _robot);
                rm::define_frames(_robot, base_frame, camera_frame);

                babbling::tf_base_conversion(normal_robot, normal_camera, camera_frame, base_frame);

                babbling::tf_base_conversion(pose_robot, pose_camera, camera_frame, base_frame);

                poseGoal.header = _images_sub->get_depth().header;
                poseGoal.target_pose.resize(3);
                poseGoal.target_pose[0] = pose_robot(0);
                poseGoal.target_pose[1] = pose_robot(1);
                poseGoal.target_pose[2] = pose_robot(2);

                poseGoal.target_normal.resize(3);
                poseGoal.target_normal[0] = normal_robot(0);
                poseGoal.target_normal[1] = normal_robot(1);
                poseGoal.target_normal[2] = normal_robot(2);

                _target_point = poseGoal.target_pose;



                ROS_INFO_STREAM(
                            "BABBLING_NODE : pose in camera in frame " << pose_camera[0] << " " << pose_camera[1] << " " << pose_camera[2]);


                ROS_INFO_STREAM(
                            "BABBLING_NODE : Going to pose " << poseGoal.target_pose[0] << " " << poseGoal.target_pose[1] <<
                                                                                           " " << poseGoal.target_pose[2]);



                _client_controller->sendGoal(poseGoal);
                _publish_rviz_feedback();




                _counter_iter++;

                _robot_controller_ready = false;


            } else if(_robot_controller_ready){
//            } else if(_robot_controller_ready){
                ROS_INFO_STREAM("Waiting for db_manager");
                return;
            }
            //*/




            //* Retrieve the state of the controller
            _client_controller->waitForResult(ros::Duration(1.0));
            if (_client_controller->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                _controller_succeed = true;
            else if(_client_controller->getState() == actionlib::SimpleClientGoalState::ABORTED)
                _controller_aborted = true;


            if (_controller_succeed) {
                _target_info_pub->publish(_target_info_msg);

                _controller_succeed = false;



                ROS_INFO_STREAM("BABBLING_NODE : Position reached.");
                _reset_world = true;

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


    std::unique_ptr<Publisher> _target_point_pub;




    std::unique_ptr<Publisher> _is_finish_pub;
    std::unique_ptr<Publisher> _terminated;

    std::string _robot;

    pcl::Supervoxel<ip::PointT> _sv; /**< current supervoxel chosen for exploration */
    uint32_t _lbl; /**< label of the current supervoxel */


    std::vector<std::string> _mcs_mod_mapping;

    bool _robot_controller_ready;
    bool _controller_is_up = false;
    bool _controller_succeed = false;
    bool _controller_aborted = false;
    bool _waiting_for_input_cloud = true;
    bool _possible_choice = true;
    bool _reset_world = true;
    bool _motion_checked = true;
    bool _no_sim;

    std::vector<double> _target_point;

    int _last_number_of_samples = 0;
    int _counter_iter = 0;
    int _nb_iter;

    int _nb_false_pos = 0;
    int _nb_false_neg = 0;

    boost::random::mt19937 gen;




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
};

int main(int argc, char** argv)
{
    std::string node_name;
//    std::string output_file;
    XmlRpc::XmlRpcValue cafer;


    tbb::task_scheduler_init init;

    node_name = global::parse_arg(argc, argv, "fake_babbling_node");

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
