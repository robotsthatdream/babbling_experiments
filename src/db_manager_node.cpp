#include "globals.h"
#include <cafer_core/cafer_core.hpp>
#include <data/data_controller_feedback.hpp>
#include <data/data_motion_rgbd.hpp>
#include <data/data_dataset.hpp>
#include <data/data_gmm.hpp>
#include <data/sync_data.hpp>
#include <data/data_target_info.hpp>
#include <boost/algorithm/string.hpp>

using namespace cafer_core;


class DBM_node : public DatabaseManager{
    using DatabaseManager::DatabaseManager;

public:


    void init(){
        client_connect_to_ros();

        while(!add_wave("/dream_babbling/babbling")){
            spin();
            update();
            sleep();
        }

        init_wave(_connected_waves.begin()->second->id);
    }

    bool init_wave(uint32_t id){


        auto it = _connected_waves.find(id);
        if(it == _connected_waves.end())
            return false;
        shared_ptr<DatabaseManager::_Wave> wave_1(it->second);

        std::vector<std::string> string_split;
        std::map<std::string,std::unique_ptr<ManagerQueue<DataDataset>>> dataset_managers;
        std::map<std::string,std::unique_ptr<ManagerQueue<DataGMM>>> gmm_managers;
        for(const auto& topic : wave_1->data_topics){
            boost::algorithm::split(string_split,topic.first,boost::is_any_of("_"));
            if(string_split[0] == "dataset")
                dataset_managers.emplace(topic.first,std::unique_ptr<ManagerQueue<DataDataset>>(
                                             new ManagerQueue<DataDataset>));
            else if(string_split[0] == "gmm")
                gmm_managers.emplace(topic.first,std::unique_ptr<ManagerQueue<DataGMM>>(
                                         new ManagerQueue<DataGMM>));

        }
    //    std::unique_ptr<ManagerQueue<DataController>> controller_manager{new ManagerQueue<DataController>()};
    //    std::unique_ptr<ManagerQueue<DataMotionRGBD>> motion_rgbd_manager{new ManagerQueue<DataMotionRGBD>()};
        std::unique_ptr<ManagerQueue<SyncData>> sync_data_manager{new ManagerQueue<SyncData>()};
        std::unique_ptr<ManagerQueue<DataTargetInfo>> target_info_manager{new ManagerQueue<DataTargetInfo>()};

        if (wave_1 != nullptr) {//&& wave_2 != nullptr) {

            for(auto& manager : dataset_managers){
                wave_1->add_manager(manager.second.release(), wave_1->data_topics[manager.first]);
            }
            for(auto& manager : gmm_managers){
                wave_1->add_manager(manager.second.release(), wave_1->data_topics[manager.first]);
            }

    //        wave_1->add_manager(motion_rgbd_manager.release(), wave_1->data_topics["motion"]);
    //        wave_1->add_manager(controller_manager.release(), wave_1->data_topics["joints_values"]);
            wave_1->add_manager(sync_data_manager.release(),wave_1->data_topics["sync_dataset"]);
            wave_1->add_manager(target_info_manager.release(),wave_1->data_topics["target_info"]);
        }
    }

    void update(){
        for(const auto& w : _connected_waves){
            if(w.second->managers.empty())
                init_wave(w.second->id);
        }
    }
};


int main(int argc, char** argv)
{
    std::string node_name;
    XmlRpc::XmlRpcValue cafer;

    node_name = global::parse_arg(argc, argv, "db_manager_node");
    cafer_core::init(argc, argv, node_name);
    cafer_core::ros_nh->getParam("cafer", cafer);

    DBM_node db_manager(cafer["mgmt"], cafer["type"], cafer["freq"]);


    // Retrieve WATCHDOG messages from the management topic.
    db_manager.wait_for_init();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    db_manager.spin();




//    db_manager.add_wave("/dream_babbling/classifier_evaluation");




    //At this points, waves have to be declared at compile-time for the DB Manager to use them.
    //db_manager.add_wave()

    while (ros::ok() && (!db_manager.get_terminate())) {
        db_manager.spin();
        db_manager.update();
        db_manager.sleep();
    }

    return EXIT_SUCCESS;
}
