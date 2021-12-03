
//
// Created by Alex on 28.09.21.
//

#include <diploma_2021/logic/perception/map_perceptor_node.hh>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <diploma_2021/transport/transport.hh>

#include <diploma_2021/logic/perception/perceptor/door_perceptor.hh>
#include <bcr_transport/visualization/visual_logger.hh>


namespace diploma_2021::logic::perception {

MapPerceptorNode::~MapPerceptorNode() {}

MapPerceptorNode::MapPerceptorNode(const diploma_2021::logic::world::WorldHandler* worldHandler, diploma_2021::transport::Transport & transport) :
        Node("map_perceptor"), world_(worldHandler), transport_(transport) {
    using namespace std::chrono_literals;
    cfg_ = std::make_unique<diploma_2021::logic::localization::sensor_processing::LidarProcessorConfig>(get_node_parameters_interface());
    perceptorsCreator_.insert({diploma_2021::model::LinkType::DoorRoot,
                               [&](boost::uuids::uuid uuid, const diploma_2021::logic::world::WorldHandler* handler) -> std::unique_ptr<Perceptor> {
                                    return std::make_unique<DoorPerceptor>(uuid, handler, *cfg_);
                                }
    });

    timer_ = this->create_wall_timer(500ms, [this] { Tick(); });
}

void MapPerceptorNode::Tick() {
    if (!world_->Proxy().MapLoaded()) {
        return;
    } else if (!mapLoaded_){
        InitPerceptors();
    }

    double ts;
    state_ = world_->Proxy().MapState(ts);
    RecalculateMapState();
    PublishMapState();
    DrawPerceptors();
}

void MapPerceptorNode::RecalculateMapState() {
    for (const auto &p : perceptors_) {
        p->Percept(state_);
    }
    world_->Proxy().Update(new diploma_2021::logic::world::MapStateUpdateAction(state_));
}

void MapPerceptorNode::PublishMapState() {
    bcr::transport::proto::world::MapState mapState_msg;//conventions?
    for(const auto &uuid: state_.JointUUIDs()) {
        auto values = state_.GetJointState(uuid)->Values();
        if (!values.empty()) {
            auto jointState_msg = mapState_msg.add_joints();
            auto utf8uuid = boost::lexical_cast<std::string>(uuid);
            jointState_msg->set_uuid(utf8uuid.c_str());
            for (const auto &v: values) {
                jointState_msg->add_values(v);
            }
        }
    }
    transport_.RobotStateOuterPub()->Publish(transport_.Config()->MapStateTopic(), mapState_msg);
}

void MapPerceptorNode::InitPerceptors() {
    RCLCPP_INFO(get_logger(), "Starting initialization of perceptors");
    sassert(!world_->Proxy().Map().expired());
    sassert(perceptors_.empty());

    auto links = world_->Proxy().Map().lock()->Links();
    for (const auto &l : links) {
        if (perceptorsCreator_.find(l->Type()) != perceptorsCreator_.end()) {
            perceptors_.push_back(perceptorsCreator_[l->Type()](l->Uuid(), world_));
            auto dl = dynamic_cast<diploma_2021::model::DoorLink *>(world_->Proxy().Map().lock()->SearchLink(l->UuidString()));
            RCLCPP_INFO(get_logger(), "Initializing perceptor of type=[%s] with name=[%s] and uuid=[%s]; MinMax Angle=[%lf][%lf]"
                        , ToString(l->Type()), l->Name().c_str(), l->UuidString().c_str(), dl->GetDoorData().MinAngle, dl->GetDoorData().MaxAngle);
            
        }
    }
    mapLoaded_ = true;

    RCLCPP_INFO(get_logger(), "Finished initialization of perceptors");
}

void MapPerceptorNode::DrawPerceptors() {
    bcr::transport::visualization::Graphics g;
    for (const auto &p: perceptors_) {
        p->Draw(g);
    }
    bcr::transport::visualization::VisualLogger::Instance()->Draw("map_perceptor", "perception_result",
          [&](bcr::transport::visualization::Graphics &graphics) {
                graphics = g;
          });
}
}