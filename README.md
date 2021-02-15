# scene understanding traffic participant

1. subscriber to scene understanding manager (SUM) package to get location and type of detections
2. create and delete traffic participant per detection
3. update location of detection using `Trajectory` 

## Install

* follow the [rmf_demos](https://github.com/osrf/rmf_demos/blob/master/docs/installation.md) page for installation 

* In the same workspace, 

```bash
cd rmf_demo_ws/src

git clone git@gitlab.com:imda_dsl/vama-2/su_traffic_participant.git

colcon build --packages-select su_traffic_participant
```

## Run

* This project had been tested with rmf_demos office simulation

```bash
source /opt/ros/foxy/setup.bash
cd rmf_demo_ws
source install/setup.bash
ros2 launch demos office.launch.xml
```

Send a loop request. eg. from pantry to station_1

Open workspace in another terminal and source

```bash
ros2 run su_traffic_participant listener
```

## Usage for Testing and Demostration

Open workspace in another terminal and source. Send mock detection in form of `su_msgs` using `ros2 topic pub` command. 

* near entrance of pantry

```bash
ros2 topic pub --once /su_detections su_msgs/ObjectsLocation "{robot_id: 'ROBOT_123', objects:[{object_class: 'cone', object_locations: [{center:[16.4, -6.89, -0.01], dimensions:[10,10,10], yaw: 0.0}]}]}"
```

near main entrance

```bash
ros2 topic pub --once /su_detections su_msgs/ObjectsLocation "{robot_id: 'ROBOT_123', objects:[{object_class: 'cone', object_locations: [{center:[14.0, -4.0, -0.01], dimensions:[10,10,10], yaw: 0.0}]}]}"
```

2 objects detections in single message

```bash
ros2 topic pub --once /su_detections su_msgs/ObjectsLocation "{robot_id: 'ROBOT_123', objects:[{object_class: 'people', object_locations: [{center:[14.0, -4.0, -0.01], dimensions:[10,10,10], yaw: 0.0}]}, {object_class: 'wheelchair', object_locations: [{center:[14.0, -4.0, -0.01], dimensions:[10,10,10], yaw: 0.0}]}]}"
```

* update the location of a similar nearby participant

near main entrance

eg. first detection

```bash
ros2 topic pub --once /su_detections su_msgs/ObjectsLocation "{robot_id: 'ROBOT_123', objects:[{object_class: 'cone', object_locations: [{center:[16.4, -6.89, -0.01], dimensions:[10,10,10], yaw: 0.0}]}]}"
```

eg. second detection

```bash
ros2 topic pub --once /su_detections su_msgs/ObjectsLocation "{robot_id: 'ROBOT_123', objects:[{object_class: 'cone', object_locations: [{center:[16.0, -6.89, -0.01], dimensions:[10,10,10], yaw: 0.0}]}]}"
```



