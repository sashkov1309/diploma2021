//
// Created by alex on 30.09.21.
//

#ifndef DIPLOMA_2021_PERCEPTOR_HH
#define DIPLOMA_2021_PERCEPTOR_HH

#include <world/world_handler.hh>
#include <map/map_state.hh>

namespace diploma_2021::logic::perception {

class Perceptor {
public:
    Perceptor(boost::uuids::uuid linkUuid, const diploma_2021::logic::world::WorldHandler* world);
    virtual ~Perceptor();

    virtual void Percept(diploma_2021::model::map::MapState &state) = 0;
    virtual void Draw(bcr::transport::visualization::Graphics &g) = 0;

protected:
    boost::uuids::uuid uuid_;
    const diploma_2021::logic::world::WorldHandler* world_;
};
}
#endif //DIPLOMA_2021_PERCEPTOR_HH
