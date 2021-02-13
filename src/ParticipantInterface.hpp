#include <rmf_traffic/schedule/Participant.hpp>

#ifndef PARTICIPANTINTERFACE_HPP
#define PARTICIPANTINTERFACE_HPP

// extern std::shared_ptr<SuTrafficNode> node;

extern std::array<std::string, 2> texts = {"wheelchair", "cone"};

extern std::map<int, Eigen::Vector3d> map;

void create_participant(
    int id, 
    Eigen::Vector3d detectionLocation);

void remove_participant(
    int id);

void print_detection_map();

std::pair<bool, int> calculate_distance(Eigen::Vector3d newPos);

#endif //PARTICIPANTINTERFACE_HPP