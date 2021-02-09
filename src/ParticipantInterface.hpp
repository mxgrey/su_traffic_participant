#include <rmf_traffic/schedule/Participant.hpp>

#ifndef PARTICIPANTINTERFACE_HPP
#define PARTICIPANTINTERFACE_HPP

// extern std::shared_ptr<SuTrafficNode> node;

void create_participant(
    int id, 
    Eigen::Vector3d detectionLocation);

void update_participant(
    int id, 
    Eigen::Vector3d detectionLocation);

#endif //PARTICIPANTINTERFACE_HPP