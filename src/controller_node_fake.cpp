//
// Created by phlf on 24/03/16.
//
#include <cafer_core/cafer_core.hpp>

#include <dream_babbling/pose_goalAction.h>
#include <actionlib/server/simple_action_server.h>
#include "globals.h"

using namespace dream_babbling;
using namespace cafer_core;

class Controller : public Component {
    using Component::Component;


public :

    void client_connect_to_ros() override
    {
        cafer_core::ros_nh->setParam("delay", 10.0);
        _serv.reset(new actionlib::SimpleActionServer<pose_goalAction>(*cafer_core::ros_nh, ros::this_node::getName(),
                                                                       boost::bind(&Controller::execute, this, _1),
                                                                       false));
    }

    void init() override
    {
        client_connect_to_ros();
        _serv->start();
    }


    void execute(const pose_goalGoalConstPtr& poseGoal)
    {
        double delay;
        cafer_core::ros_nh->getParamCached("delay", delay);

        ROS_INFO_STREAM("CONTROLLER: I AM FAKE");

        if (delay < 0.0 || delay >= 60.0) {
            ROS_WARN("Valid delay bounds: [0.0s, 60.0s[");
            delay = 2.0;
        }
        ros::Duration(delay).sleep();

        ROS_INFO_STREAM("CONTROLLER: Position reached");
        _serv->setSucceeded();
    }

    void client_disconnect_from_ros() override
    {}

    void update() override
    {}

private:
    std::unique_ptr <actionlib::SimpleActionServer<pose_goalAction>> _serv;
    dream_babbling::pose_goalFeedback _joints_pose_feedback;
};


int main(int argc, char** argv)
{
    std::string node_name;
    XmlRpc::XmlRpcValue cafer;

    cafer_core::init(argc, argv, node_name);
    cafer_core::ros_nh->getParam("cafer", cafer);

    node_name = global::parse_arg(argc, argv, "controller_node");

    cafer_core::init(argc, argv, node_name);

    Controller controller(cafer["mgmt"], cafer["type"], cafer["freq"], cafer["uuid"]);

    controller.wait_for_init();
    controller.spin();

    ROS_INFO_STREAM("CONTROLLER: Fake controller ready !");

    while (ros::ok() && (!controller.get_terminate())) {
        controller.spin();
        controller.update();
        controller.sleep();
    }

    return 0;
}
