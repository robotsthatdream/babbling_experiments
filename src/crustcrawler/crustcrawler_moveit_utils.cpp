//
// Created by phlf on 24/03/16.
//
#include <iostream>
#include <string>
#include <boost/timer.hpp>
#include <vector>
#include <cafer_core/cafer_core.hpp>

#include <dream_babbling/pose_goalAction.h>
#include <actionlib/server/simple_action_server.h>

#include <ros/callback_queue.h>
#include <fstream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <boost/timer.hpp>

#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "crustcrawler_mover_utils/crustcrawler_mover.hpp"
#include <crustcrawler_mover_utils/move_crustcrawler_arm.h>
#include <crustcrawler_core_msgs/EndEffectorCommand.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

using namespace dream_babbling;
using namespace cafer_core;

class Controller: public Component{
    using Component::Component; // C++11 requirement to inherit the constructor

public :

    void jostateCallback(const sensor_msgs::JointState::ConstPtr& jo_state)
    {
        _arranged_joints_positions[0] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                 jo_state->name.end(),
                                                                                                 _crustcrawler_mover->global_parameters.get_crustcrawler_arm_joints_names()[0]))];
        _arranged_joints_positions[1] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                 jo_state->name.end(),
                                                                                                 _crustcrawler_mover->global_parameters.get_crustcrawler_arm_joints_names()[1]))];
        _arranged_joints_positions[2] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                 jo_state->name.end(),
                                                                                                 _crustcrawler_mover->global_parameters.get_crustcrawler_arm_joints_names()[2]))];
        _arranged_joints_positions[3] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                 jo_state->name.end(),
                                                                                                 _crustcrawler_mover->global_parameters.get_crustcrawler_arm_joints_names()[3]))];
        _arranged_joints_positions[4] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                 jo_state->name.end(),
                                                                                                 _crustcrawler_mover->global_parameters.get_crustcrawler_arm_joints_names()[4]))];
        _arranged_joints_positions[5] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                 jo_state->name.end(),
                                                                                                 _crustcrawler_mover->global_parameters.get_crustcrawler_arm_joints_names()[5]))];


        _joints_pose_feedback.joints_positions = _arranged_joints_positions;
        if(_start_pub_joint_fb)
            _serv->publishFeedback(_joints_pose_feedback);
    }

    void client_connect_to_ros(){
        XmlRpc::XmlRpcValue glob_params;
        std::stringstream display_params;

        cafer_core::ros_nh->getParam("/dream_babbling/params", glob_params);
        for (auto& param:glob_params) {
            display_params << "\t" << param.first << ":\t" << param.second << std::endl;
        }

        ROS_INFO_STREAM("CONTROLLER : Global parameters retrieved:" << std::endl << display_params.str());

        _number_of_trials = std::stod(glob_params["number_of_trials"]);

        ROS_WARN_STREAM("CONTROLLER : Number of trials is : " << _number_of_trials);

        _serv.reset(new actionlib::SimpleActionServer<pose_goalAction>(*cafer_core::ros_nh, glob_params["controller_server"],
                    boost::bind(&Controller::execute, this, _1),false));

        ROS_WARN_STREAM("CONTROLLER : Trying to initialize the robot mover class, name space is: " << cafer_core::ros_nh->getNamespace());
        _crustcrawler_mover.reset(new crustcrawler_mover::CRUSTCRAWLER_Mover(*cafer_core::ros_nh));
        _move_crustcrawler_arm.reset(new ros::ServiceClient(cafer_core::ros_nh->serviceClient<crustcrawler_mover_utils::move_crustcrawler_arm>("/move_crustcrawler_arm")));
        _sub_joint_states.reset(new ros::Subscriber(cafer_core::ros_nh->subscribe("/crustcrawler/joint_states",1,&Controller::jostateCallback, this)));
        _reset_world.reset(new ros::ServiceClient(cafer_core::ros_nh->serviceClient<std_srvs::Empty>("/gazebo/reset_world")));
        _gripper_command_publisher.reset(new ros::Publisher(cafer_core::ros_nh->advertise<crustcrawler_core_msgs::EndEffectorCommand>
                                                            ("/crustcrawler/end_effector/gripper/command", 1, this)));
    }

    void init(){
        _arranged_joints_positions = std::vector<double>(6,0);
        connect_to_ros();

        usleep(2e6);

        _serv->start();
        _is_init = true;

        _home_variable_values.insert ( std::pair<std::string, double>("joint_1",  0.1) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_2", -0.1) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_3", -0.9) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_4", -0.1) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_5", -1.0) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_6",  0.1) );

        home_values_ = {0.1, -0.1, -0.9, -0.1, -1.0, 0.1};

        _collision_object.header.frame_id = "/base";
        _collision_object.id = "box1";
        _primitive.type = _primitive.BOX;
        _primitive.dimensions.resize(3);
        _primitive.dimensions[0] = 0.08;
        _primitive.dimensions[1] = 0.08;
        _primitive.dimensions[2] = 0.1;
        _box_pose.position.x =  0.;
        _box_pose.position.y =  0.;
        _box_pose.position.z =  0.;

        _collision_object.primitives.push_back(_primitive);
        _collision_object.primitive_poses.push_back(_box_pose);
        _collision_object.operation = _collision_object.ADD;

        ros::AsyncSpinner my_spinner(4);
        my_spinner.start();

        _crustcrawler_mover->group->setPlannerId(static_cast<std::string>(_crustcrawler_mover->global_parameters.get_planner_parameters()["planner_id"]));
        _crustcrawler_mover->group->setPlanningTime(std::stod(_crustcrawler_mover->global_parameters.get_planner_parameters()["planning_time"]));

        ROS_WARN_STREAM("CONTROLLER : The planner ID is : " << static_cast<std::string>(_crustcrawler_mover->global_parameters.get_planner_parameters()["planner_id"]));

        //add a table infront and a wall behind
        tf::Quaternion q;
        geometry_msgs::PoseStamped front_pose;
        front_pose.header.frame_id = "base";
        front_pose.header.stamp = ros::Time::now();

        front_pose.pose.position.x = 0.2;
        front_pose.pose.position.y = 0.0;
        front_pose.pose.position.z = -0.06;
        q.setEuler(0.0, 0.0, M_PI/2);
        front_pose.pose.orientation.w = q.getW();
        front_pose.pose.orientation.x = q.getX();
        front_pose.pose.orientation.y = q.getY();
        front_pose.pose.orientation.z = q.getZ();

        _crustcrawler_mover->add_world_object("front_table", front_pose, {0.5, 1.5, 0.01});

        //back wall artificial obstacle
        geometry_msgs::PoseStamped back_pose;
        back_pose.header.frame_id = "base";
        back_pose.header.stamp = ros::Time::now();

        back_pose.pose.position.x = -0.1;
        back_pose.pose.position.y = 0.0;
        back_pose.pose.position.z = 0.5;
        q.setEuler(0.0, 0.0, M_PI/2);
        back_pose.pose.orientation.w = q.getW();
        back_pose.pose.orientation.x = q.getX();
        back_pose.pose.orientation.y = q.getY();
        back_pose.pose.orientation.z = q.getZ();

        _crustcrawler_mover->add_world_object("back_wall", back_pose, {0.01, 1.5, 1.0});

        go_home();

        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        _robot_model = robot_model_loader.getModel();

        ROS_INFO("Add an object into the world");
        _my_scene.world.collision_objects.push_back(_collision_object);
        _my_scene.is_diff = true;
        _crustcrawler_mover->publish_psm_msg(_my_scene);
    }

    //get largest difference between elements of two vectors
    double largest_difference(std::vector<double> &first, std::vector<double> &second){
        Eigen::VectorXd difference(first.size());
        double my_max = 0;
        for(size_t j = 0; j < first.size(); ++j)
            difference(j) = fabs(first[j] - second[j]);
        for(size_t j = 0; j < first.size(); ++j){
            if(difference(j) > my_max)
                my_max = difference(j);
        }
        return my_max;
    }

    void octomap_manipulation(bool manipulate){
        _crustcrawler_mover->global_parameters.set_adding_octomap_to_acm(manipulate);
        _crustcrawler_mover->call_service_get_ps();
        _crustcrawler_mover->publish_psm_msg();
    }

    bool go_home(){
        int32_t back_home = 0;
        if(largest_difference(home_values_,
                              _arranged_joints_positions) > 0.01){
            _crustcrawler_mover->group->setStartState(*_crustcrawler_mover->group->getCurrentState());

            _crustcrawler_mover->group->setJointValueTarget(_home_variable_values);

            if(_crustcrawler_mover->group->plan(_group_plan)){
                _start_pub_joint_fb = false;
                back_home = _crustcrawler_mover->group->execute(_group_plan);
                _crustcrawler_mover->group->setStartState(*_crustcrawler_mover->group->getCurrentState());
            }
        }
        else
            //I am already at home
            back_home = 1;
        if(back_home == 1)
            return true;
        else
            return false;
    }

    bool reverse_back_trajectory(moveit::planning_interface::MoveGroup::Plan& a_plan){
        _reverse_plan.trajectory_.joint_trajectory.header.stamp = ros::Time::now();
        _reverse_plan.trajectory_.joint_trajectory.header.frame_id = a_plan.trajectory_.joint_trajectory.header.frame_id;
        _reverse_plan.trajectory_.joint_trajectory.joint_names = a_plan.trajectory_.joint_trajectory.joint_names;

        int j = a_plan.trajectory_.joint_trajectory.points.size() - 1;
        trajectory_processing::IterativeParabolicTimeParameterization time_param;
        robot_trajectory::RobotTrajectory my_robot_trajectory(_robot_model, _crustcrawler_mover->group->getName());
        robot_state::RobotState r_state_holder(_robot_model);
        for(size_t i = 0; i < a_plan.trajectory_.joint_trajectory.points.size() && j >= 0; i++){
            //ROS_WARN_STREAM("TEST : In the for loop trying to construct the inverse trajectory with size : " << a_plan.trajectory_.joint_trajectory.points.size());
            moveit::core::jointTrajPointToRobotState(a_plan.trajectory_.joint_trajectory, j, r_state_holder);
            my_robot_trajectory.insertWayPoint(i, r_state_holder, 0.1);
            j--;
        }

        if(!time_param.computeTimeStamps(my_robot_trajectory))
            ROS_WARN("TEST : Time parametrization for the solution path failed.");

        my_robot_trajectory.getRobotTrajectoryMsg(_reverse_plan.trajectory_);
        moveit::core::robotStateToRobotStateMsg(*_crustcrawler_mover->group->getCurrentState(), _reverse_plan.start_state_);
        return(_crustcrawler_mover->group->execute(_reverse_plan));
    }

    void open_gripper(){
        _gripper_command.args = "{position: 100.0}";
        _gripper_command.command = "go";
        _gripper_command_publisher->publish(_gripper_command);
    }

    void close_gripper(){
        _gripper_command.args = "{position: 0.0}";
        _gripper_command.command = "go";
        _gripper_command_publisher->publish(_gripper_command);
    }

    void execute(const pose_goalGoalConstPtr& poseGoal)
    {
        ROS_INFO_STREAM("CONTROLLER : Received new goal, I am planning for X = " << poseGoal->target_pose[0]
                << ", Y = " << poseGoal->target_pose[1]
                << " and Z = " << poseGoal->target_pose[2]);
        _goal = poseGoal->target_pose;

        _box_pose.position.x = _goal[0];
        _box_pose.position.y = _goal[1];
        _box_pose.position.z = _goal[2];

        //make sure that the octomap is in the collision world
        octomap_manipulation(true);
        //make sure you are at home position
        if(!_at_home){
            go_home();
        }

        open_gripper();
        usleep(1e6);
        close_gripper();

        bool arm_retracted = false;

        int iteration = 0;
        bool valid_plan = false;

        while(!valid_plan && iteration < _number_of_trials){
            //always plan for target directly by including the octomap and just remove the area around the sv
            _collision_object.primitive_poses.clear();
            _collision_object.primitive_poses.push_back(_box_pose);

            _my_scene.world.collision_objects.clear();
            _my_scene.world.collision_objects.push_back(_collision_object);
            _my_scene.is_diff = true;
            _crustcrawler_mover->publish_psm_msg(_my_scene);

            _crustcrawler_mover->global_parameters.set_adding_octomap_to_acm(false);
            _crustcrawler_mover->call_service_get_ps();
            _crustcrawler_mover->global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_names.push_back("box1");
            _crustcrawler_mover->global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_values.push_back(true);
            _crustcrawler_mover->publish_psm_msg();

            _crustcrawler_mover->group->setStartState(*_crustcrawler_mover->group->getCurrentState());
            _crustcrawler_mover->group->setPositionTarget(_box_pose.position.x, _box_pose.position.y, _box_pose.position.z);


            if(_crustcrawler_mover->group->plan(_group_plan)){
                _crustcrawler_mover->group->execute(_group_plan);
                usleep(3e6);
                //retract the arm back to the approach pose for safe return home, without hitting other objects
                arm_retracted = reverse_back_trajectory(_group_plan);
                _touch_object_success = true;
                valid_plan = true;
                ROS_INFO_STREAM("CONTROLLER : A valid plan is found");
            }
            else{
                _touch_object_success = false;
                ROS_WARN_STREAM("CONTROLLER : Failed to find plan, in iteration: " << iteration );
            }
            ROS_WARN_STREAM("CONTROLLER : ******************: " << iteration << " :*******************");
            iteration++;
        }
        if(arm_retracted)
            octomap_manipulation(true);

        _at_home = go_home();


        //std::cin.ignore();
        if(_touch_object_success){
            ROS_INFO_STREAM("CONTROLLER : The motion was successful !!");
            _total_number_of_successful_interactions++;
            ROS_WARN_STREAM("CONTROLLER : Total number of successful interactions till now is : " << _total_number_of_successful_interactions);
            _serv->setSucceeded();
        }
        else{
            ROS_WARN_STREAM("CONTROLLER : Something went wrong");
            _total_number_of_failed_interactions++;
            ROS_WARN_STREAM("CONTROLLER : Total number of failed interactions till now is : " << _total_number_of_failed_interactions);
            _serv->setAborted();
        }

        _goal.clear();
        _total_number_of_interactions = _total_number_of_failed_interactions + _total_number_of_successful_interactions;
        ROS_WARN_STREAM("CONTROLLER : Total number of interactions till now is : " << _total_number_of_interactions);
        //_reset_world->call(_reset_world_req, _reset_world_res);
        ROS_WARN_STREAM("CONTROLLER : ************************,, SEND NEW GOAL ,,*********************");
    }



    void client_disconnect_from_ros(){}
    void update(){}

private:
    std::shared_ptr<actionlib::SimpleActionServer<pose_goalAction>> _serv;
    std::shared_ptr<ros::ServiceClient> _reset_world;
    std::shared_ptr<ros::Publisher> _scene_publisher, _gripper_command_publisher;
    std::shared_ptr<ros::Subscriber> _sub_joint_states, _sub_eef_state;
    robot_model::RobotModelPtr _robot_model;
    moveit_msgs::PlanningScene _new_scene;
    std::unique_ptr<ros::ServiceClient> _move_crustcrawler_arm;
    std::vector<double> _goal;
    std_srvs::Empty::Request _reset_world_req;
    std_srvs::Empty::Response _reset_world_res;

    moveit_msgs::CollisionObject _collision_object;
    shape_msgs::SolidPrimitive _primitive;
    geometry_msgs::Pose _box_pose;
    moveit_msgs::PlanningScene _my_scene;

    crustcrawler_mover::CRUSTCRAWLER_Mover::Ptr _crustcrawler_mover;
    crustcrawler_core_msgs::EndEffectorCommand _gripper_command;
    dream_babbling::pose_goalFeedback _joints_pose_feedback;
    std::map<std::string, double> _home_variable_values;
    moveit::planning_interface::MoveGroup::Plan _group_plan, _reverse_plan;

    bool _touch_object_success = false, _start_pub_joint_fb = false, _at_home = false;
    std::vector<double> home_values_, _arranged_joints_positions;
    int _number_of_trials = 5;
    int _total_number_of_interactions = 0, _total_number_of_successful_interactions = 0, _total_number_of_failed_interactions = 0;

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
    XmlRpc::XmlRpcValue cafer;

    node_name = parse_arg(argc, argv, "controller_node");

    cafer_core::init(argc, argv, node_name);
    cafer_core::ros_nh->getParam("cafer", cafer);
    ROS_INFO_STREAM(cafer_core::ros_nh->getNamespace());

    Controller controller(cafer["mgmt"], cafer["type"], cafer["freq"],cafer["uuid"]);

    controller.wait_for_init();
    controller.spin();

    ROS_INFO_STREAM("CONTROLLER : Robot controller ready !");

    while (ros::ok() && (!controller.get_terminate())) {
        controller.spin();
    }
    return 0;
}
