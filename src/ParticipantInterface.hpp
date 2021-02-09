#include <rmf_traffic/schedule/Participant.hpp>

#ifndef PARTICIPANTINTERFACE_HPP
#define PARTICIPANTINTERFACE_HPP

// extern std::shared_ptr<SuTrafficNode> node;

extern std::map<int, Eigen::Vector3d> detections;

void create_participant(
    int id, 
    Eigen::Vector3d detectionLocation);

void update_participant(
    int id, 
    Eigen::Vector3d detectionLocation);

void print_detection_map();

#endif //PARTICIPANTINTERFACE_HPP