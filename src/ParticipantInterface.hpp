#include <rmf_traffic/schedule/Participant.hpp>

#ifndef PARTICIPANTINTERFACE_HPP
#define PARTICIPANTINTERFACE_HPP

int test();

rmf_traffic::schedule::Participant create_participant(
    std::string id, 
    Eigen::Vector3d detectionLocation);

#endif //PARTICIPANTINTERFACE_HPP