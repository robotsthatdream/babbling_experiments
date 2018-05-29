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


    void poseCallback(const tf2_msgs::TFMessageConstPtr& msg){
        if(!_goal.empty() && !_goal_normal.empty()){
            static tf::TransformBroadcaster br;
            tf::Transform transform;

            //Set and publish a frame for the goal position
            transform.setOrigin( tf::Vector3(_goal[0], _goal[1], _goal[2]) );
            tf::Quaternion q;
            q.setRPY(_goal_normal[0], _goal_normal[1], _goal_normal[2]);
            //q.setRPY(0, 0, 0);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "goal_frame"));


            //Set and publish a frame for all approach poses
            for(size_t i = 0; i < _target.size(); i++){
                transform.setOrigin( tf::Vector3(_target[i].pose.position.x, _target[i].pose.position.y, _target[i].pose.position.z) );
                q.setW(_target[i].pose.orientation.w);
                q.setX(_target[i].pose.orientation.x);
                q.setY(_target[i].pose.orientation.y);
                q.setZ(_target[i].pose.orientation.z);
                transform.setRotation(q);
                //                                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "goal_frame", "approach_frame" + std::to_string(i)));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "approach_frame" + std::to_string(i)));
            }

            if(!_final_goal.empty()){
                for(size_t i = 0; i < _final_goal.size(); i++){
                    //_final_goal[i].header.frame
                    transform.setOrigin( tf::Vector3(_final_goal[i].pose.position.x, _final_goal[i].pose.position.y, _final_goal[i].pose.position.z) );
                    //q.setRPY(0, M_PI/2, 0);
                    q.setRPY(0, 0, 0);
                    transform.setRotation(q);
                    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "final_goal_frame" + std::to_string(i)));

                }
            }
        }
    }


    void client_connect_to_ros(){
        XmlRpc::XmlRpcValue glob_params;
        std::stringstream display_params;

        cafer_core::ros_nh->getParam("/dream_babbling/params", glob_params);
        for (auto& param:glob_params) {
            display_params << "\t" << param.first << ":\t" << param.second << std::endl;
        }

        _number_of_trials = std::stoi(glob_params["number_of_trials"]);
        ROS_INFO_STREAM("CONTROLLER : Global parameters retrieved:" << std::endl << display_params.str());

        _serv.reset(new actionlib::SimpleActionServer<pose_goalAction>(*cafer_core::ros_nh, glob_params["controller_server"],
                    boost::bind(&Controller::execute, this, _1),false));

        ROS_WARN_STREAM("CONTROLLER : Trying to initialize the robot mover class, name space is: " << cafer_core::ros_nh->getNamespace());
        _crustcrawler_mover.reset(new crustcrawler_mover::CRUSTCRAWLER_Mover(*cafer_core::ros_nh));
        _move_crustcrawler_arm.reset(new ros::ServiceClient(cafer_core::ros_nh->serviceClient<crustcrawler_mover_utils::move_crustcrawler_arm>("/move_crustcrawler_arm")));
        _sub_joint_states.reset(new ros::Subscriber(cafer_core::ros_nh->subscribe("/crustcrawler/joint_states",1,&Controller::jostateCallback, this)));
        _reset_world.reset(new ros::ServiceClient(cafer_core::ros_nh->serviceClient<std_srvs::Empty>("/gazebo/reset_world")));
        _gripper_command_publisher.reset(new ros::Publisher(cafer_core::ros_nh->advertise<crustcrawler_core_msgs::EndEffectorCommand>
                                                            ("/crustcrawler/end_effector/gripper/command", 1, this)));
        _tf_subscriber.reset(new ros::Subscriber(cafer_core::ros_nh->subscribe("/tf", 10, &Controller::poseCallback, this)));
    }

    void init(){
        _arranged_joints_positions = std::vector<double>(6,0);
        connect_to_ros();

        ros::Duration(1).sleep();

        _serv->start();
        _is_init = true;

        _home_variable_values.insert ( std::pair<std::string, double>("joint_1", -1.3) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_2", -0.3) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_3", -1.1) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_4",  0.0) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_5", -0.5) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_6",  0.0) );

        home_values_ = {-1.3, -0.3, -1.1, 0.0, -0.5, 0.0};

        ros::AsyncSpinner my_spinner(4);
        my_spinner.start();

        _goal.clear();
        _goal_normal.clear();
        _target.clear();
        _final_goal.clear();

        _crustcrawler_mover->group->setPlannerId(static_cast<std::string>(_crustcrawler_mover->global_parameters.get_planner_parameters()["planner_id"]));
        _crustcrawler_mover->group->setPlanningTime(std::stod(_crustcrawler_mover->global_parameters.get_planner_parameters()["planning_time"]));

        ROS_WARN_STREAM("CONTROLLER : The planner ID is : " << static_cast<std::string>(_crustcrawler_mover->global_parameters.get_planner_parameters()["planner_id"]));

        _crustcrawler_mover->global_parameters.set_approach_radius(std::stod(_crustcrawler_mover->global_parameters.get_planner_parameters()["approach_radius"]));
        _approach_radius = _crustcrawler_mover->global_parameters.get_approach_radius();
        _straight_line_threshold = _approach_radius/(_approach_radius + fabs(_extension_distance));

        //add a table infront and a wall behind
        tf::Quaternion q;
        geometry_msgs::PoseStamped front_pose;
        front_pose.header.frame_id = "base";
        front_pose.header.stamp = ros::Time::now();

        front_pose.pose.position.x = 0.2;
        front_pose.pose.position.y = 0.0;
        front_pose.pose.position.z = 0.0;
        q.setEuler(0.0, 0.0, M_PI/2);
        front_pose.pose.orientation.w = q.getW();
        front_pose.pose.orientation.x = q.getX();
        front_pose.pose.orientation.y = q.getY();
        front_pose.pose.orientation.z = q.getZ();

        _crustcrawler_mover->add_world_object("front_table", front_pose, {0.5, 1.5, 0.01});

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

    void get_approach_point(std::vector<double>& poseGoal,
                            Eigen::Vector3d& approach_point){
        //srand(time(NULL));
        double u = rand() / (RAND_MAX + 1.0);
        double v = rand() / (RAND_MAX + 1.0);
        //double theta = M_PI/3 * u;
        //double phi = M_PI/3 * v;
        double theta = M_PI * u + M_PI/2; //azimuth angle in the X-Y plane from M_PI/2 - 3*M_PI/2
        double phi = acos(0.5*v + 0.5); //altitude angle between Z axis and the point, just for the upper half of the sphere so it goes from 60 to 0
        double x = _approach_radius*sin(phi)*cos(theta);
        double y = _approach_radius*sin(phi)*sin(theta);
        double z = _approach_radius*cos(phi);
        _point_keeper.x = x;
        _point_keeper.y = y;
        _point_keeper.z = z;
        approach_point(0) = poseGoal[0] + x;
        approach_point(1) = poseGoal[1] + y;
        approach_point(2) = poseGoal[2] + z;
    }

    geometry_msgs::PoseStamped get_target_pose(std::vector<double>& poseGoal){
        Eigen::Vector3d approach_point;
        get_approach_point(poseGoal, approach_point);

        tf::Vector3 V(-_point_keeper.x, -_point_keeper.y, -_point_keeper.z);
        V.normalize();
        tf::Vector3 VP(V.getX(), V.getY(), 0);
        double yaw = atan2(V.getY(), V.getX());
        double pitch = atan2(VP.length() ,V.getZ());

        tf::Quaternion q;
        q.setRPY(0, pitch, yaw);

        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = "base";
        target_pose.pose.position.x = approach_point(0);
        target_pose.pose.position.y = approach_point(1);
        target_pose.pose.position.z = approach_point(2);
        target_pose.pose.orientation.w = q.w();
        target_pose.pose.orientation.x = q.x();
        target_pose.pose.orientation.y = q.y();
        target_pose.pose.orientation.z = q.z();
        return target_pose;
    }

    void octomap_manipulation(bool manipulate){
        _crustcrawler_mover->global_parameters.set_adding_octomap_to_acm(manipulate);
        _crustcrawler_mover->call_service_get_ps();
        _crustcrawler_mover->publish_psm_msg();
    }

    bool go_home(){
        int32_t back_home = 0;
        if(largest_difference(home_values_,
                              _arranged_joints_positions) > 0.15){
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

    geometry_msgs::PoseStamped get_extended_goal(geometry_msgs::PoseStamped first_point, std::vector<double> second_point){
        //solve for the point with distance dist from the second point
        double d = sqrt(pow(second_point[0] - first_point.pose.position.x, 2) +
                pow(second_point[1] - first_point.pose.position.y, 2) +
                pow(second_point[2] - first_point.pose.position.z, 2));

        double u = _extension_distance/d;

        geometry_msgs::PoseStamped final_goal;
        final_goal.header.frame_id = "base";
        final_goal.pose.position.x = (1 - u)*second_point[0] + u*first_point.pose.position.x;
        final_goal.pose.position.y = (1 - u)*second_point[1] + u*first_point.pose.position.y;
        final_goal.pose.position.z = (1 - u)*second_point[2] + u*first_point.pose.position.z;


        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        final_goal.pose.orientation.w = q.getW();
        final_goal.pose.orientation.x = q.getX();
        final_goal.pose.orientation.y = q.getY();
        final_goal.pose.orientation.z = q.getZ();

        return final_goal;
    }

    std::vector<geometry_msgs::PoseStamped> get_random_targets_around_sphere(int number_target, std::vector<double> my_goal){
        //for(size_t i = 0; i < my_goal.size(); i++)
        //ROS_ERROR_STREAM("get_random_targets_around_sphere point : " << i << " is : " << my_goal[i]);
        std::vector<geometry_msgs::PoseStamped> targets;
        _final_goal.clear();
        for(int i = 0; i < number_target; i++){
            geometry_msgs::PoseStamped new_approach_point = get_target_pose(my_goal);
            //ROS_ERROR_STREAM("approach point : " << i << " X = " << new_approach_point.pose.position.x
            //               << " Y = " << new_approach_point.pose.position.y
            //             << " Z = " << new_approach_point.pose.position.z);
            targets.push_back(new_approach_point);
            _final_goal.push_back(get_extended_goal(new_approach_point, my_goal));
        }
        return targets;
    }

    int get_index_of_pose(std::vector<geometry_msgs::PoseStamped> targets, geometry_msgs::Pose winning_target){
        std::vector<double> first = {winning_target.position.x, winning_target.position.y, winning_target.position.z};
        double biggest_difference = std::numeric_limits<double>::infinity();
        int index;
        for(size_t i = 0; i < targets.size(); i++){
            std::vector<double> second = {targets[i].pose.position.x, targets[i].pose.position.y, targets[i].pose.position.z};
            double this_difference = largest_difference(first, second);
            if(this_difference < biggest_difference){
                index = i;
                biggest_difference = this_difference;
            }
        }

        if(biggest_difference < 0.01){
            //ROS_WARN_STREAM("CONTROLLER : Found the index of the chosen approach pose, it is : " << index << " and the difference is: " << biggest_difference);
            // ROS_WARN_STREAM("CONTROLLER : The position is: X = " << targets[index].pose.position.x
            // << ", Y = " << targets[index].pose.position.y
            //  << ", Z = " << targets[index].pose.position.z);
            return index;
        }
        else
            return -1;
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
        ROS_INFO_STREAM("CONTROLLER : Number of trials is : " << _number_of_trials);
        _goal = poseGoal->target_pose;
        _goal_normal = poseGoal->target_normal;
        if(_first_goal){
            _first_goal = false;
            _target = get_random_targets_around_sphere(_number_of_points, _goal);
        }

        //make sure that the octomap is in the collision world
        octomap_manipulation(true);
        //make sure you are at home position
        if(!_at_home){
            go_home();
        }

        bool reversed = false;

        int iteration = 0;
        bool complete_plan = false;

        while(!complete_plan && iteration < _number_of_trials){
            //always plan for the approach with the octomap in the obstacle domain
            octomap_manipulation(false);

            std::vector<double> my_goal = poseGoal->target_pose;
            _target = get_random_targets_around_sphere(_number_of_points, my_goal);
            int32_t plan_result;
            _arm = "arm";

            moveit::planning_interface::MoveGroup::Plan test_plan;
            _crustcrawler_mover->group->clearPoseTargets();
            _crustcrawler_mover->group->setStartState(*_crustcrawler_mover->group->getCurrentState());
            _crustcrawler_mover->group->setPoseTargets(_target);
            plan_result = _crustcrawler_mover->group->plan(test_plan);
            _start_state_second_trajectory = _crustcrawler_mover->group->getCurrentState();

            if(plan_result){
                moveit::core::jointTrajPointToRobotState(test_plan.trajectory_.joint_trajectory,
                                                         test_plan.trajectory_.joint_trajectory.points.size() - 1,
                                                         *_start_state_second_trajectory);

                geometry_msgs::Pose winning_pose;
                tf::poseEigenToMsg(_start_state_second_trajectory->getGlobalLinkTransform("the_gripper"), winning_pose);

                ROS_ERROR_STREAM("CONTROLLER : The winning position according to the plan is: X = " << winning_pose.position.x
                                 << ", Y = " << winning_pose.position.y
                                 << ", Z = " << winning_pose.position.z);

                int index = get_index_of_pose(_target, winning_pose);

                ROS_INFO("------------------------------------------------------------------------");

                ROS_ERROR_STREAM("CONTROLLER : The winning position from the list of targets is: X = " << _target[index].pose.position.x
                                 << ", Y = " << _target[index].pose.position.y
                                 << ", Z = " << _target[index].pose.position.z);

                //std::cin.ignore();

                //so if the first part is reachable with the obstacles on the way, make sure the rest could be done in straight line given that the obstacle is removed
                octomap_manipulation(true);
                std::vector<geometry_msgs::Pose> waypoints;
                geometry_msgs::Pose poses = _target[index].pose;
                waypoints.push_back(poses);
                poses.position.x = _final_goal[index].pose.position.x;
                poses.position.y = _final_goal[index].pose.position.y;
                poses.position.z = _final_goal[index].pose.position.z;
                waypoints.push_back(poses);

                ROS_INFO_STREAM("CONTROLLER : Index is : " << index << " :----------------------------------------------");
                moveit_msgs::RobotTrajectory robot_trajectory;
                double fraction;
                //_crustcrawler_mover->group->setStartState(*_start_state_second_trajectory);
                fraction = _crustcrawler_mover->group->computeCartesianPath(waypoints, 0.05, 10.0, robot_trajectory);

                ROS_WARN_STREAM("CONTROLLER : fraction solved of desired path in this trial is: " <<
                                fraction << " And the success threshold is: " << _straight_line_threshold);

                if(fraction >= _straight_line_threshold){
                    complete_plan = true;
                    _crustcrawler_mover->group->execute(test_plan);

                    ros::Duration(2).sleep();

                    fraction = _crustcrawler_mover->group->computeCartesianPath(waypoints, 0.05, 10.0, robot_trajectory);
                    moveit::planning_interface::MoveGroup::Plan test_touch_plan;
                    test_touch_plan.trajectory_ = robot_trajectory;

                    moveit::core::robotStateToRobotStateMsg(*_crustcrawler_mover->group->getCurrentState(), test_touch_plan.start_state_);
                    _crustcrawler_mover->group->setStartState(*_crustcrawler_mover->group->getCurrentState());
                    _touch_object_success = _crustcrawler_mover->group->execute(test_touch_plan);

                    //execute the trajectory in reverse
                    reverse_back_trajectory(test_touch_plan);
                    reversed = reverse_back_trajectory(test_plan);

                    ROS_INFO_STREAM("CONTROLLER : A valid plan from start with final straight line motion is found");
                }
                else{
                    _touch_object_success = false;
                    ROS_WARN_STREAM("CONTROLLER : Failed to find straight line motion plan for last segment, in iteration: " << iteration);
                }
            }
            else{
                _touch_object_success = false;
                ROS_WARN_STREAM("CONTROLLER : Failed to find plan for the approaching part, in iteration: " << iteration );
            }
            ROS_WARN_STREAM("CONTROLLER : ******************: " << iteration << " :*******************");
            iteration++;
            _total_iteration++;
        }
        if(reversed)
            octomap_manipulation(true);

        _at_home = go_home();

        //ROS_INFO("********** ENTER ***********");
        //std::cin.ignore();
        if(_touch_object_success){
            _successful_iterations++;
            ROS_INFO_STREAM("CONTROLLER : The motion was successful !!");
            _serv->setSucceeded();
        }
        else{
            _failed_iteration++;
            ROS_WARN_STREAM("CONTROLLER : Something went wrong");
            _serv->setAborted();
        }

        _goal.clear();
        _goal_normal.clear();
        _target.clear();
        _final_goal.clear();

        ROS_WARN_STREAM("CONTROLLER : FAILED ITERATION **************: " << _failed_iteration << " :*******************");
        ROS_WARN_STREAM("CONTROLLER : SUCCESSFUL ITERATION **************: " << _successful_iterations << " :*******************");
        ROS_WARN_STREAM("CONTROLLER : TOTAL **************: " << _total_iteration << " :*******************");
        ROS_WARN_STREAM("CONTROLLER : ************************,, SEND NEW GOAL ,,*********************");
    }



    void client_disconnect_from_ros(){}
    void update(){}

private:
    std::shared_ptr<actionlib::SimpleActionServer<pose_goalAction>> _serv;
    std::shared_ptr<ros::ServiceClient> _reset_world;
    std::shared_ptr<ros::Publisher> _scene_publisher, _gripper_command_publisher;
    std::shared_ptr<ros::Subscriber> _sub_joint_states, _sub_eef_state, _tf_subscriber;
    robot_model::RobotModelPtr _robot_model;
    moveit_msgs::PlanningScene _new_scene;
    std::unique_ptr<ros::ServiceClient> _move_crustcrawler_arm;
    std::vector<double> _goal, _goal_normal;
    std::vector<geometry_msgs::PoseStamped> _target, _final_goal;
    geometry_msgs::Point _point_keeper;

    std_srvs::Empty::Request _reset_world_req;
    std_srvs::Empty::Response _reset_world_res;

    moveit_msgs::CollisionObject _collision_object;
    shape_msgs::SolidPrimitive _primitive;
    geometry_msgs::Pose _box_pose;
    moveit_msgs::PlanningScene _my_scene;

    tf::StampedTransform _transform;
    tf::TransformListener _listener;
    std::string _parent_frame = "/base";
    std::string _child_frame_id;

    robot_state::RobotStatePtr _start_state_second_trajectory, _current_robot_state;

    crustcrawler_mover::CRUSTCRAWLER_Mover::Ptr _crustcrawler_mover;
    crustcrawler_core_msgs::EndEffectorCommand _gripper_command;
    dream_babbling::pose_goalFeedback _joints_pose_feedback;
    std::map<std::string, double> _home_variable_values;
    moveit::planning_interface::MoveGroup::Plan _group_plan, _reverse_plan;

    bool _touch_object_success = false, _start_pub_joint_fb = false, _first_goal = true, _at_home = false;
    std::vector<double> home_values_, _arranged_joints_positions;
    int _number_of_trials = 2, _number_of_points = 10, _total_iteration = 0, _failed_iteration = 0, _successful_iterations = 0;
    double _extension_distance = 0.02, _straight_line_threshold, _approach_radius, _retract_distance = 0.1;
    std::string _arm;
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
