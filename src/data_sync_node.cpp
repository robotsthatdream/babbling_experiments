#include <iostream>
#include <memory>

#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dream_babbling/sync_dataset.h>
#include <dream_babbling/rgbd_motion_data.h>

#include <baxter_core_msgs/EndpointState.h>

#include "globals.h"


#include <cafer_core/cafer_core.hpp>

#include <std_msgs/Bool.h>


using namespace cafer_core;
namespace mf = message_filters;
using namespace dream_babbling;

class DataSynchronizer : public Component{
    using Component::Component;

public:

    using SyncPolicy = mf::sync_policies::ApproximateTime
    <rgbd_motion_data,baxter_core_msgs::EndpointState>;


    void init(){
        client_connect_to_ros();
    }

    void client_connect_to_ros(){
        XmlRpc::XmlRpcValue glob_params;
        XmlRpc::XmlRpcValue datamanager_topics;
        ros_nh->getParam("/dream_babbling/params",glob_params);
        ros_nh->getParam("/dream_babbling/babbling/topics/data",datamanager_topics);

        //* Output the values of all parameters
        std::stringstream display_params;
        for (auto& param:glob_params) {
            display_params << "\t" << param.first << ":\t" << param.second << std::endl;
        }
        for (auto& param:datamanager_topics) {
            display_params << "\t" << param.first << ":\t" << param.second << std::endl;
        }
        ROS_INFO_STREAM("BABBLING_NODE : Parameters retrieved:" << std::endl << display_params.str());
        //*/

        _controller_sub.reset(
                    new mf::Subscriber<baxter_core_msgs::EndpointState>(*ros_nh,
                            static_cast<std::string>(glob_params["endeffector_pose_topic"]),5));
        _rgbd_motion_sub.reset(
                    new mf::Subscriber<rgbd_motion_data>(
                        *ros_nh,static_cast<std::string>(datamanager_topics["motion"]),5));
        _target_sub.reset(
                    new Subscriber(
                        ros_nh->subscribe(static_cast<std::string>(glob_params["target_topic"]),5,&DataSynchronizer::target_cb,this)));


        _synchronizer.reset(
                    new mf::Synchronizer<SyncPolicy>(
                        SyncPolicy(5),*_rgbd_motion_sub,*_controller_sub));

        _synchronizer->registerCallback(boost::bind(&DataSynchronizer::callback,this,_1,_2));

        _data_pub.reset(
                    new Publisher(ros_nh->advertise<sync_dataset>(
                                      static_cast<std::string>(datamanager_topics["sync_dataset"]),5)));

        _babbling_terminated.reset(new ros::Subscriber(ros_nh->subscribe("/dream_babbling/babbling_node/babbling_terminated",5,&DataSynchronizer::babbling_term_cb,this)));

    }

    void client_disconnect_from_ros(){
        _controller_sub.reset();
        _rgbd_motion_sub.reset();
        _target_sub.reset();
        _synchronizer.reset();
        _data_pub.reset();
        _babbling_terminated.reset();
    }


    void babbling_term_cb(const std_msgs::BoolConstPtr msg){
        if(msg->data == 1){
            ROS_INFO_STREAM("BABBLING IS FINISH");
            is_finish = true;
        }
    }

    void update(){
        if(is_finish)
            exit(0);
    }

//    void endpoint_cb(const baxter_core_msgs::EndpointStateConstPtr& eep){
//        _endpoint_msg = eep->pose;
//    }

    void target_cb(const geometry_msgs::PointStampedConstPtr& target){
        _target_pos_msg = *target;
    }

    void callback(const rgbd_motion_dataConstPtr& rgbdmd,const baxter_core_msgs::EndpointStateConstPtr& eep){

        ROS_INFO_STREAM("enter in callback");

        double dist = distance({eep->pose.position.x,eep->pose.position.y,eep->pose.position.z},
                               {_target_pos_msg.point.x,_target_pos_msg.point.y,_target_pos_msg.point.z});
        ROS_INFO_STREAM(dist);
        int reward = (dist < 0.05 ? 1 : 0);

        _data_msg.header = rgbdmd->header;
        _data_msg.pose = eep->pose;

        _data_msg.reward = reward;
        _data_msg.rgb = rgbdmd->rgb;
        _data_msg.depth = rgbdmd->depth;
        _data_pub->publish(_data_msg);


    }

    double distance(std::vector<double> p1,std::vector<double> p2){
        return std::sqrt((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]) + (p1[2]-p2[2])*(p1[2]-p2[2]));
    }

private:
    std::unique_ptr<mf::Subscriber<baxter_core_msgs::EndpointState>> _controller_sub;
    std::unique_ptr<mf::Subscriber<rgbd_motion_data>> _rgbd_motion_sub;
    std::unique_ptr<Subscriber> _target_sub;
    boost::shared_ptr<mf::Synchronizer<SyncPolicy>> _synchronizer;
    std::unique_ptr<Publisher> _data_pub;
    geometry_msgs::Pose _endpoint_msg;
    geometry_msgs::PointStamped _target_pos_msg;
    sync_dataset _data_msg;
    bool is_finish = false;
    std::unique_ptr<ros::Subscriber> _babbling_terminated;

};

int main(int argc, char** argv){

    std::string node_name;
    XmlRpc::XmlRpcValue cafer;

    global::parse_arg(argc,argv,"data_sync_node");

    init(argc,argv,node_name);
    ros_nh->getParam("cafer",cafer);

    DataSynchronizer data_synchronizer(cafer["mgmt"], cafer["type"], cafer["freq"], cafer["uuid"]);

    data_synchronizer.wait_for_init();
    data_synchronizer.spin();

    while(ros::ok() && !data_synchronizer.get_terminate()){
        data_synchronizer.spin();
        data_synchronizer.update();
        data_synchronizer.sleep();
    }

    return 0;
}
