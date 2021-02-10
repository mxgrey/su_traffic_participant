#include <rmf_traffic/schedule/Participant.hpp>

#ifndef PARTICIPANTINTERFACE_HPP
#define PARTICIPANTINTERFACE_HPP

// extern std::shared_ptr<SuTrafficNode> node;

extern std::map<int, Eigen::Vector3d> map;

void create_participant(
    int id, 
    Eigen::Vector3d detectionLocation);

void update_participant(
    int id, 
    Eigen::Vector3d detectionLocation);

void print_detection_map();

std::pair<bool, int> calculate_distance(Eigen::Vector3d newPos);

#endif //PARTICIPANTINTERFACE_HPP