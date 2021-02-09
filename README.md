# scene understanding traffic participant

1. subscriber to scene understanding manager (SUM) package to get location and type of detections
2. create and delete traffic participant per detection
3. update location of detection using `Trajectory` 

## Usage for Demostration

* send a mock detection 
  
  `ros2 topic pub --once /su_detections geometry_msgs/Point "{x: 1, y: 2, z: 3}"`

near entrance of pantry

```bash
ros2 topic pub --once /su_detections su_msgs/ObjectsLocation "{robot_id: 'ROBOT_123', objects:[{object_class: 'cone', object_locations: [{center:[16.4, -6.89, -0.01], dimensions:[10,10,10], yaw: 0.0}]}]}"
```

near main entrance

```bash
ros2 topic pub --once /su_detections su_msgs/ObjectsLocation "{robot_id: 'ROBOT_123', objects:[{object_class: 'cone', object_locations: [{center:[14.0, -4.0, -0.01], dimensions:[10,10,10], yaw: 0.0}]}]}"
```

ros2 topic pub --once /su_detections su_msgs/ObjectsLocation "{robot_id: 'TRUE', objects:[{object_class: 'cone', object_locations: [{center:[14.0, -4.0, -0.01], dimensions:[10,10,10], yaw: 0.0}]}]}"



