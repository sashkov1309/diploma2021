//
// Created by alex on 30.09.21.
//

#ifndef DIPLOMA_2021_DOOR_PERCEPTOR_HH
#define DIPLOMA_2021_DOOR_PERCEPTOR_HH

#include <diploma_2021/logic/perception/perceptor/perceptor.hh>
#include <diploma_2021/logic/localization/sensor_processing/lidar_processor_config.hh>

namespace diploma_2021::logic::perception {

class DoorPerceptor : public Perceptor {
    struct Lidars {
        somatic::math::Vector3 fl, fr, br, bl;
    };

public:
    DoorPerceptor(boost::uuids::uuid linkUuid, const diploma_2021::logic::world::WorldHandler* world, const diploma_2021::logic::localization::sensor_processing::LidarProcessorConfig &cfg);

    void Percept(diploma_2021::model::map::MapState &state) override;

    void Draw(bcr::transport::visualization::Graphics &g) override;
private:
    std::vector<Eigen::Vector2f> inside_;
    std::map<float, std::pair<std::vector<somatic::math::Vector3>, std::vector<somatic::math::LineSegment>>> mapCut_;
    const float inc = 0.0174533f; // 1 deg
    const float recognitionDistance_ = 3.5f;

    Lidars lidars_;

    std::map<float, std::vector<somatic::math::LineSegment>> visibleLines_;
};
}
#endif //DIPLOMA_2021_DOOR_PERCEPTOR_HH
