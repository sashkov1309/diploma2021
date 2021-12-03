//
// Created by alex on 30.09.21.
//

#include "diploma_2021/logic/perception/perceptor/perceptor.hh"

namespace diploma_2021::logic::perception {
Perceptor::Perceptor(boost::uuids::uuid linkUuid, const diploma_2021::logic::world::WorldHandler *world) : world_(world), uuid_(linkUuid) {

}

    Perceptor::~Perceptor() {

    }
}