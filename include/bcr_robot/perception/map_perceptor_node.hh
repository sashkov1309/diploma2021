//
// Created by Alex on 28.09.21.
//

#ifndef DIPLOMA_2021_MAP_PERCEPTOR_NODE_HH
#define DIPLOMA_2021_MAP_PERCEPTOR_NODE_HH

#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <bcr_transport/core/publisher.hh>
#include <diploma_2021/logic/localization/sensor_processing/lidar_processor_config.hh>

namespace diploma_2021::tools::status {
class StatusHelper;
};

namespace diploma_2021::transport{
    class Transport;
}


namespace diploma_2021::logic::perception {

class Perceptor;

class MapPerceptorNode : public rclcpp::Node {
public:
    explicit MapPerceptorNode(const diploma_2021::logic::world::WorldHandler* worldHandler, diploma_2021::transport::Transport & transport);
    ~MapPerceptorNode() override;

    void Tick();

    void InitPerceptors();

private:
    void PublishMapState();
    void RecalculateMapState();
    void DrawPerceptors();

    const diploma_2021::logic::world::WorldHandler* world_;
    diploma_2021::model::map::MapState state_;
    rclcpp::TimerBase::SharedPtr timer_;
    diploma_2021::transport::Transport & transport_;

    std::map<diploma_2021::model::LinkType, std::function<std::unique_ptr<Perceptor>(boost::uuids::uuid, const diploma_2021::logic::world::WorldHandler*)>> perceptorsCreator_;
    std::vector<std::unique_ptr<Perceptor>> perceptors_;
    std::unique_ptr<diploma_2021::logic::localization::sensor_processing::LidarProcessorConfig> cfg_;
    bool mapLoaded_ = false;
};

}

#endif //DIPLOMA_2021_MAP_PERCEPTOR_NODE_HH
