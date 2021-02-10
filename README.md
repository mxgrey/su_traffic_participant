# scene understanding traffic participant

1. subscriber to scene understanding manager (SUM) package to get location and type of detections
2. create and delete traffic participant per detection
3. update location of detection using `Trajectory` 

## Usage for Demostration

* send mock detection in form of `su_msgs`


near entrance of pantry

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
