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

#include <baxter_mover_utils/baxter_mover.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>

using namespace dream_babbling;
using namespace cafer_core;

class Controller: public Component{
    using Component::Component; // C++11 requirement to inherit the constructor

public :

    void jostateCallback(const sensor_msgs::JointState::ConstPtr& jo_state)
    {
        _arranged_left_joints_positions[0] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                      jo_state->name.end(),
                                                                                                      _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[0]))];
        _arranged_left_joints_positions[1] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                      jo_state->name.end(),
                                                                                                      _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[1]))];
        _arranged_left_joints_positions[2] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                      jo_state->name.end(),
                                                                                                      _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[2]))];
        _arranged_left_joints_positions[3] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                      jo_state->name.end(),
                                                                                                      _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[3]))];
        _arranged_left_joints_positions[4] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                      jo_state->name.end(),
                                                                                                      _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[4]))];
        _arranged_left_joints_positions[5] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                      jo_state->name.end(),
                                                                                                      _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[5]))];
        _arranged_left_joints_positions[6] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                      jo_state->name.end(),
                                                                                                      _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[6]))];

        //right arm joint states arranged
        _arranged_right_joints_positions[0] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                       jo_state->name.end(),
                                                                                                       _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[0]))];
        _arranged_right_joints_positions[1] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                       jo_state->name.end(),
                                                                                                       _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[1]))];
        _arranged_right_joints_positions[2] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                       jo_state->name.end(),
                                                                                                       _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[2]))];
        _arranged_right_joints_positions[3] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                       jo_state->name.end(),
                                                                                                       _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[3]))];
        _arranged_right_joints_positions[4] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                       jo_state->name.end(),
                                                                                                       _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[4]))];
        _arranged_right_joints_positions[5] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                       jo_state->name.end(),
                                                                                                       _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[5]))];
        _arranged_right_joints_positions[6] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                       jo_state->name.end(),
                                                                                                       _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[6]))];
        _joints_pose_feedback.joints_positions = _arranged_left_joints_positions;
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
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "goal_frame", "approach_frame" + std::to_string(i)));
            }

            if(!_final_goal.empty()){
                for(size_t i = 0; i < _final_goal.size(); i++){
                    //_final_goal[i].header.frame
                    transform.setOrigin( tf::Vector3(_final_goal[i].pose.position.x, _final_goal[i].pose.position.y, _final_goal[i].pose.position.z) );
                    q.setRPY(0, 0, 0);
                    transform.setRotation(q);
                    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "approach_frame" + std::to_string(i), "final_goal_frame" + std::to_string(i)));

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

        ROS_INFO_STREAM("CONTROLLER : Global parameters retrieved:" << std::endl << display_params.str());

        _serv.reset(new actionlib::SimpleActionServer<pose_goalAction>(*cafer_core::ros_nh, glob_params["controller_server"],
                    boost::bind(&Controller::execute, this, _1),false));

        ROS_WARN_STREAM("CONTROLLER : Trying to initialize the robot mover class, name space is: " << cafer_core::ros_nh->getNamespace());
        _baxter_mover.reset(new baxter_mover::BAXTER_Mover(*cafer_core::ros_nh));
        _move_baxter_arm.reset(new ros::ServiceClient(cafer_core::ros_nh->serviceClient<baxter_mover_utils::move_baxter_arm>("/move_baxter_arm")));
        _sub_joint_states.reset(new ros::Subscriber(cafer_core::ros_nh->subscribe("/joint_states",1,&Controller::jostateCallback, this)));
        _tf_subscriber.reset(new ros::Subscriber(cafer_core::ros_nh->subscribe("/tf", 10, &Controller::poseCallback, this)));
    }

    void init(){
        _arranged_left_joints_positions = std::vector<double>(7,0);
        _arranged_right_joints_positions = std::vector<double>(7,0);
        connect_to_ros();

        ros::Duration(1).sleep();

        _serv->start();
        _is_init = true;

        _home_variable_values.insert ( std::pair<std::string, double>("left_s0",  1.47) );
        _home_variable_values.insert ( std::pair<std::string, double>("left_s1", -0.67) );
        _home_variable_values.insert ( std::pair<std::string, double>("left_e0", -1.06) );
        _home_variable_values.insert ( std::pair<std::string, double>("left_e1",  1.42) );
        _home_variable_values.insert ( std::pair<std::string, double>("left_w0",  0.75) );
        _home_variable_values.insert ( std::pair<std::string, double>("left_w1",  1.24) );
        _home_variable_values.insert ( std::pair<std::string, double>("left_w2",  1.43) );

        _secondary_home.insert ( std::pair<std::string, double>("right_s0", -1.52) );
        _secondary_home.insert ( std::pair<std::string, double>("right_s1", -0.79) );
        _secondary_home.insert ( std::pair<std::string, double>("right_e0",  1.17) );
        _secondary_home.insert ( std::pair<std::string, double>("right_e1",  1.73) );
        _secondary_home.insert ( std::pair<std::string, double>("right_w0", -0.74) );
        _secondary_home.insert ( std::pair<std::string, double>("right_w1",  1.13) );
        _secondary_home.insert ( std::pair<std::string, double>("right_w2", -1.21) );

        left_home_values_ = {1.47, -0.67, -1.06, 1.42, 0.75, 1.24, 1.43};
        right_home_values_ = {-1.52, -0.79, -1.17, 1.73, -0.74, 1.13, -1.21};
        _home_cartesian_left = {0.368, 0.714, 0.915};
        _home_cartesian_right = {0.117, -0.619, 0.649};

        ros::AsyncSpinner my_spinner(4);
        my_spinner.start();

        _goal.clear();
        _goal_normal.clear();
        _target.clear();
        _final_goal.clear();
        _transformed_target.clear();
        _transformed_final_goal.clear();

        _baxter_mover->group->setPlannerId(static_cast<std::string>(_baxter_mover->global_parameters.get_planner_parameters()["planner_id"]));
        _baxter_mover->group->setPlanningTime(std::stod(_baxter_mover->global_parameters.get_planner_parameters()["planning_time"]));

        _baxter_mover->secondary_group->setPlannerId(static_cast<std::string>(_baxter_mover->global_parameters.get_planner_parameters()["planner_id"]));
        _baxter_mover->secondary_group->setPlanningTime(std::stod(_baxter_mover->global_parameters.get_planner_parameters()["planning_time"]));

        ROS_WARN_STREAM("CONTROLLER : The planner ID is : " << static_cast<std::string>(_baxter_mover->global_parameters.get_planner_parameters()["planner_id"]));


        _baxter_mover->global_parameters.set_approach_radius(std::stod(_baxter_mover->global_parameters.get_planner_parameters()["approach_radius"]));
        _approach_radius = _baxter_mover->global_parameters.get_approach_radius();
        _straight_line_threshold = _approach_radius/(_approach_radius + fabs(_extension_distance));

        go_home_left();
        go_home_right();

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
        double theta = M_PI/3 * u;
        double phi = M_PI/3 * v;
        double x = _approach_radius*sin(phi)*cos(theta);
        double y = _approach_radius*sin(phi)*sin(theta);
        double z = _approach_radius*cos(phi);
        _point_keeper.x = x;
        _point_keeper.y = y;
        _point_keeper.z = z;
        approach_point(0) = x;
        approach_point(1) = y;
        approach_point(2) = z;
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
        target_pose.header.frame_id = "goal_frame";
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
        _baxter_mover->global_parameters.set_adding_octomap_to_acm(manipulate);
        _baxter_mover->call_service_get_ps();
        _baxter_mover->publish_psm_msg();
    }

    bool go_home_left(){
        int32_t back_home = 0;
        if(largest_difference(left_home_values_,
                              _arranged_left_joints_positions) > 0.15){
            _baxter_mover->group->setStartState(*_baxter_mover->group->getCurrentState());

            _baxter_mover->group->setJointValueTarget(_home_variable_values);

            if(_baxter_mover->group->plan(_group_plan)){
                _start_pub_joint_fb = false;
                back_home = _baxter_mover->group->execute(_group_plan);
                _baxter_mover->group->setStartState(*_baxter_mover->group->getCurrentState());
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

    bool go_home_right(){
        int32_t back_home = 0;
        if(largest_difference(right_home_values_,
                              _arranged_right_joints_positions) > 0.15){
            _baxter_mover->secondary_group->setStartState(*_baxter_mover->secondary_group->getCurrentState());

            _baxter_mover->secondary_group->setJointValueTarget(_secondary_home);

            if(_baxter_mover->secondary_group->plan(_group_plan)){
                _start_pub_joint_fb = false;
                back_home = _baxter_mover->secondary_group->execute(_group_plan);
                _baxter_mover->secondary_group->setStartState(*_baxter_mover->secondary_group->getCurrentState());
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

    std::vector<geometry_msgs::PoseStamped> get_random_targets_around_sphere(int number_target, std::vector<double> my_goal){
        std::vector<geometry_msgs::PoseStamped> targets;
        _final_goal.clear();
        for(int i = 0; i < number_target; i++){
            geometry_msgs::PoseStamped new_approach_point = get_target_pose(my_goal);
            geometry_msgs::PoseStamped extended_pose;
            targets.push_back(new_approach_point);
            extended_pose.pose.position.z += _approach_radius + _extension_distance;
            _final_goal.push_back(extended_pose);
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

    bool reverse_back_trajectory(moveit::planning_interface::MoveGroup::Plan& a_plan, std::string arm){
        _revers_plan.trajectory_.joint_trajectory.header.stamp = ros::Time::now();
        _revers_plan.trajectory_.joint_trajectory.header.frame_id = a_plan.trajectory_.joint_trajectory.header.frame_id;
        _revers_plan.trajectory_.joint_trajectory.joint_names = a_plan.trajectory_.joint_trajectory.joint_names;

        int j = a_plan.trajectory_.joint_trajectory.points.size() - 1;
        trajectory_processing::IterativeParabolicTimeParameterization time_param;
        robot_trajectory::RobotTrajectory my_robot_trajectory(_robot_model, arm);
        robot_state::RobotState r_state_holder(_robot_model);
        for(size_t i = 0; i < a_plan.trajectory_.joint_trajectory.points.size() && j >= 0; i++){
            //ROS_WARN_STREAM("TEST : In the for loop trying to construct the inverse trajectory with size : " << a_plan.trajectory_.joint_trajectory.points.size());
            moveit::core::jointTrajPointToRobotState(a_plan.trajectory_.joint_trajectory, j, r_state_holder);
            my_robot_trajectory.insertWayPoint(i, r_state_holder, 0.1);
            j--;
        }

        if(!time_param.computeTimeStamps(my_robot_trajectory))
            ROS_WARN("TEST : Time parametrization for the solution path failed.");
        my_robot_trajectory.getRobotTrajectoryMsg(_revers_plan.trajectory_);
        if(strcmp("left_arm", arm.c_str()) == 0){
            moveit::core::robotStateToRobotStateMsg(*_baxter_mover->group->getCurrentState(), _revers_plan.start_state_);
            return(_baxter_mover->group->execute(_revers_plan));
        }
        else{
            moveit::core::robotStateToRobotStateMsg(*_baxter_mover->secondary_group->getCurrentState(), _revers_plan.start_state_);
            return(_baxter_mover->secondary_group->execute(_revers_plan));
        }
    }

    //transform approach points and final goals to the planning frame "base"
    bool transform_tf(std::vector<geometry_msgs::PoseStamped> poses_to_transform,
                      std::vector<geometry_msgs::PoseStamped>& output_transform,
                      std::string child_frame_name){
        output_transform.clear();
        //make sure the goal frame is at the current goal
        for(size_t jojo = 0; jojo < poses_to_transform.size(); jojo++){
            try{
                _child_frame_id = child_frame_name + std::to_string(jojo);
                _parent_frame = "base";
                _listener.lookupTransform(_parent_frame, _child_frame_id, ros::Time(0), _transform);
                geometry_msgs::PoseStamped current_target;
                current_target.header.frame_id = _parent_frame;

                current_target.pose.position.x = _transform.getOrigin().getX() ;
                current_target.pose.position.y = _transform.getOrigin().getY() ;
                current_target.pose.position.z = _transform.getOrigin().getZ() ;
                current_target.pose.orientation.w = _transform.getRotation().getW();
                current_target.pose.orientation.x = _transform.getRotation().getX();
                current_target.pose.orientation.y = _transform.getRotation().getY();
                current_target.pose.orientation.z = _transform.getRotation().getZ();
                output_transform.push_back(current_target);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }
        if(output_transform.size() == poses_to_transform.size())
            return true;
        else
            return false;
    }

    // Computes the distance between two std::vectors
    template <typename T>
    double	vectors_distance(const std::vector<T>& a, const std::vector<T>& b)
    {
        std::vector<double>	auxiliary;

        std::transform (a.begin(), a.end(), b.begin(), std::back_inserter(auxiliary),//
                        [](T element1, T element2) {return pow((element1-element2),2);});
        auxiliary.shrink_to_fit();

        return  sqrt(std::accumulate(auxiliary.begin(), auxiliary.end(), 0.0));
    } // end template vectors_distance

    void execute(const pose_goalGoalConstPtr& poseGoal)
    {
        ROS_INFO_STREAM("CONTROLLER : Received new goal, I am planning for X = " << poseGoal->target_pose[0]
                << ", Y = " << poseGoal->target_pose[1]
                << " and Z = " << poseGoal->target_pose[2]);
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
            go_home_left();
            go_home_right();
        }

        bool reversed = false;

        int iteration = 0;
        bool complete_plan = false;

        while(!complete_plan && iteration < _number_of_trials){
            //always plan for the approach with the octomap in the obstacle domain
            octomap_manipulation(false);
            //                        try{
            //                            _listener.lookupTransform("base", "goal_frame", ros::Time(0), _transform);
            //                        }
            //                        catch (tf::TransformException &ex) {
            //                                ROS_ERROR("%s",ex.what());
            //                                ros::Duration(1.0).sleep();
            //                            }
            //                        std::vector<double> second = {_transform.getOrigin().getX(), _transform.getOrigin().getY(), _transform.getOrigin().getZ()};
            std::vector<double> second(3, 0);
            //while(largest_difference(_goal, second) > 0.001){
            while(vectors_distance(_goal, second) > 0.001){
                try{
                    _listener.lookupTransform("base", "goal_frame", ros::Time(0), _transform);
                    ROS_INFO_STREAM("CONTROLLER : Largest difference is: " << largest_difference(_goal, second));
                    //                                ROS_ERROR_STREAM("CONTROLLER : goal frame is at: X = " << _transform.getOrigin().getX()
                    //                                                 << ", Y = " << _transform.getOrigin().getY()
                    //                                                 << ", Z = " << _transform.getOrigin().getZ());
                    second = {_transform.getOrigin().getX(), _transform.getOrigin().getY(), _transform.getOrigin().getZ()};
                    ros::Duration(1.0).sleep();
                }
                catch (tf::TransformException &ex) {
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();
                }
            }

            std::vector<double> vector_1(3, 0);
            while((vectors_distance(_goal, vector_1) - _extension_distance) > 0.01){
                _target = get_random_targets_around_sphere(_number_of_points, poseGoal->target_pose);

                //transform approach points and final goals to the planning frame "base"
                if(!transform_tf(_target, _transformed_target, "approach_frame"))
                    continue;
                if(!transform_tf(_final_goal, _transformed_final_goal, "final_goal_frame"))
                    continue;
                if(_transformed_target.empty() || _transformed_final_goal.empty())
                    continue;

                //                                for(size_t trtr = 0; trtr < _transformed_final_goal.size(); trtr++){
                //                                        ROS_ERROR_STREAM("CONTROLLER : Transformerd final goal element: " << trtr << " is: X = " << _transformed_final_goal[trtr].pose.position.x
                //                                                         << ", Y = " << _transformed_final_goal[trtr].pose.position.y
                //                                                         << ", Z = " << _transformed_final_goal[trtr].pose.position.z);
                //                                    }

                vector_1 = {_transformed_final_goal[0].pose.position.x,
                            _transformed_final_goal[0].pose.position.y,
                            _transformed_final_goal[0].pose.position.z};

                ROS_INFO_STREAM("CONTROLLER : The distance between current goal and first final goal frame is : " << vectors_distance(_goal, vector_1));
                ROS_INFO("*************************************************");
                ros::Duration(1).sleep();
            }
            int32_t plan_result;

            //                        if(iteration%2 == 0)
            //                            _arm = "left_arm";
            //                        else
            //                            _arm = "right_arm";
            if(poseGoal->target_pose[1] >= 0)
                _arm = "left_arm";
            else
                _arm = "right_arm";

            moveit::planning_interface::MoveGroup::Plan test_plan;
            if(strcmp("left_arm", _arm.c_str()) == 0){
                _baxter_mover->group->clearPoseTargets();
                _baxter_mover->group->setStartState(*_baxter_mover->group->getCurrentState());
                _baxter_mover->group->setPoseTargets(_transformed_target);
                plan_result = _baxter_mover->group->plan(test_plan);
                _start_state_second_trajectory = _baxter_mover->group->getCurrentState();
            }
            else{
                _baxter_mover->secondary_group->clearPoseTargets();
                _baxter_mover->secondary_group->setStartState(*_baxter_mover->secondary_group->getCurrentState());
                _baxter_mover->secondary_group->setPoseTargets(_transformed_target);
                plan_result = _baxter_mover->secondary_group->plan(test_plan);
                _start_state_second_trajectory = _baxter_mover->secondary_group->getCurrentState();
            }

            if(plan_result){
                moveit::core::jointTrajPointToRobotState(test_plan.trajectory_.joint_trajectory,
                                                         test_plan.trajectory_.joint_trajectory.points.size() - 1,
                                                         *_start_state_second_trajectory);

                geometry_msgs::Pose winning_pose;
                if(strcmp("left_arm", _arm.c_str()) == 0)
                    tf::poseEigenToMsg(_start_state_second_trajectory->getGlobalLinkTransform("left_gripper"), winning_pose);
                else
                    tf::poseEigenToMsg(_start_state_second_trajectory->getGlobalLinkTransform("right_gripper"), winning_pose);

                ROS_ERROR_STREAM("CONTROLLER : The winning position according to the plan is: X = " << winning_pose.position.x
                                 << ", Y = " << winning_pose.position.y
                                 << ", Z = " << winning_pose.position.z);

                int index = get_index_of_pose(_transformed_target, winning_pose);

                ROS_INFO("------------------------------------------------------------------------");

                ROS_ERROR_STREAM("CONTROLLER : The winning position from the list of targets is: X = " << _transformed_target[index].pose.position.x
                                 << ", Y = " << _transformed_target[index].pose.position.y
                                 << ", Z = " << _transformed_target[index].pose.position.z);

                //std::cin.ignore();

                //so if the first part is reachable with the obstacles on the way, make sure the rest could be done in straight line given that the obstacle is removed
                octomap_manipulation(true);
                std::vector<geometry_msgs::Pose> waypoints;
                waypoints.push_back(_transformed_target[index].pose);
                waypoints.push_back(_transformed_final_goal[index].pose);
                ROS_INFO_STREAM("CONTROLLER : Index is : " << index << " :----------------------------------------------");
                moveit_msgs::RobotTrajectory robot_trajectory;
                double fraction;
                if(strcmp("left_arm", _arm.c_str()) == 0){
                    _baxter_mover->group->setStartState(*_start_state_second_trajectory);
                    fraction = _baxter_mover->group->computeCartesianPath(waypoints, 0.01, 0.0, robot_trajectory);
                }
                else{
                    _baxter_mover->secondary_group->setStartState(*_start_state_second_trajectory);
                    fraction = _baxter_mover->secondary_group->computeCartesianPath(waypoints, 0.01, 0.0, robot_trajectory);
                }
                ROS_WARN_STREAM("CONTROLLER : fraction solved of desired path in this trial is: " <<
                                fraction << " And the success threshold is: " << _straight_line_threshold);

                if(fraction >= _straight_line_threshold){
                    complete_plan = true;
                    if(strcmp("left_arm", _arm.c_str()) == 0)
                        _baxter_mover->group->execute(test_plan);
                    else
                        _baxter_mover->secondary_group->execute(test_plan);

                    ros::Duration(1).sleep();

                    moveit::planning_interface::MoveGroup::Plan test_touch_plan;
                    test_touch_plan.trajectory_ = robot_trajectory;

                    if(strcmp("left_arm", _arm.c_str()) == 0){
                        moveit::core::robotStateToRobotStateMsg(*_baxter_mover->group->getCurrentState(), test_touch_plan.start_state_);
                        _baxter_mover->group->setStartState(*_baxter_mover->group->getCurrentState());
                        _touch_object_success = _baxter_mover->group->execute(test_touch_plan);
                    }
                    else{
                        moveit::core::robotStateToRobotStateMsg(*_baxter_mover->secondary_group->getCurrentState(), test_touch_plan.start_state_);
                        _baxter_mover->secondary_group->setStartState(*_baxter_mover->secondary_group->getCurrentState());
                        _touch_object_success = _baxter_mover->secondary_group->execute(test_touch_plan);
                    }
                    //execute the trajectory in reverse
                    reverse_back_trajectory(test_touch_plan, _arm);
                    reversed = reverse_back_trajectory(test_plan, _arm);

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
        if(strcmp("left_arm", _arm.c_str()) == 0)
            _at_home = go_home_left();
        else
            _at_home = go_home_right();

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
        _transformed_target.clear();
        _transformed_final_goal.clear();

        ROS_WARN_STREAM("CONTROLLER : TOTAL NUMBER OF SUCCESSFUL ITERATION **************: " << _successful_iterations << " :*******************");
        ROS_WARN_STREAM("CONTROLLER : TOTAL NUMBER OF FAILED ITERATION **************: " << _failed_iteration << " :*******************");
        ROS_WARN_STREAM("CONTROLLER : TOTAL NUMBER OF ITERATION **************: " << _total_iteration << " :*******************");
        ROS_WARN_STREAM("CONTROLLER : ************************,, SEND NEW GOAL ,,*********************");
    }



    void client_disconnect_from_ros(){}
    void update(){}

private:
    std::shared_ptr<actionlib::SimpleActionServer<pose_goalAction>> _serv;
    std::shared_ptr<ros::Publisher> _scene_publisher;
    std::shared_ptr<ros::Subscriber> _sub_joint_states, _tf_subscriber;
    robot_model::RobotModelPtr _robot_model;
    moveit_msgs::PlanningScene _new_scene;
    std::unique_ptr<ros::ServiceClient> _move_baxter_arm;
    std::vector<double> _goal, _goal_normal;
    std::vector<geometry_msgs::PoseStamped> _target, _transformed_target, _final_goal, _transformed_final_goal;
    geometry_msgs::Point _point_keeper;


    tf::StampedTransform _transform;
    tf::TransformListener _listener;
    std::string _parent_frame = "/base";
    std::string _child_frame_id;

    robot_state::RobotStatePtr _start_state_second_trajectory, _current_robot_state;

    Eigen::VectorXd _pose_home;
    baxter_mover_utils::move_baxter_arm::Request _motion_request;
    baxter_mover_utils::move_baxter_arm::Response _motion_response;
    baxter_mover::BAXTER_Mover::Ptr _baxter_mover;
    dream_babbling::pose_goalFeedback _joints_pose_feedback;
    std::map<std::string, double> _home_variable_values, _secondary_home;
    moveit::planning_interface::MoveGroup::Plan _group_plan, _approach_plan, _touch_plan, _retract_plan, _revers_plan;
    std::vector<geometry_msgs::Pose> waypoints_;


    double fraction_;
    moveit_msgs::RobotTrajectory robot_trajectory_;
    geometry_msgs::Pose _push_pose;
    std::shared_ptr<robot_state::RobotState> _robot_state;
    bool _touch_object_success = false, _start_pub_joint_fb = false, _first_goal = true, _at_home = false;
    std::vector<double> arm_joint_values_, left_home_values_, right_home_values_, _arranged_right_joints_positions, _arranged_left_joints_positions, _home_cartesian_left,
    _home_cartesian_right, _current_left_eef_cartesian_position, _current_right_eef_cartesian_position;
    int _number_of_trials = 2, _number_of_points = 10, _total_iteration = 0, _failed_iteration = 0, _successful_iterations = 0;
    double _extension_distance = 0.02, _straight_line_threshold, _approach_radius, _retract_distance = 0.2;
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
