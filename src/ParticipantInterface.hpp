#include <rmf_traffic/schedule/Participant.hpp>

#ifndef PARTICIPANTINTERFACE_HPP
#define PARTICIPANTINTERFACE_HPP

int test();

void create_participant(
    int id, 
    Eigen::Vector3d detectionLocation);

#endif //PARTICIPANTINTERFACE_HPP