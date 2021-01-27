# traffic_participant_controller

api for create update and delete of traffic participant. specifically from scene understanding manager with wheelchair and cone detection only. 

```mermaid
graph TB

  Publisher --> Subscriber
  subgraph "Scene Uds Manager"
  Publisher(publisher topic for all detections)
  end

  subgraph "rmf_fleet_adapter"
  Node(read-only fleet adapter node)
  end

  subgraph "detection adapter drivers"
  Driver(Driver 1)
  Driver2(Driver 2)
  Driver3(Driver 3)
  end

  subgraph "Traffic Particpant Controller"
  Subscriber(subscriber topic for all detections)
  Subscriber --> FindClass[filter for wheelchair and cones detections only] --> Detection((Detection Object))
  Detection --> Proximity[check for proximity based on detection type]
  Proximity -- location of object --> Create(create traffic participant)
  DB(Detection in-memory DB) --> DBops(save, update, delete)
  Create --- C1[launch driver app w publisher to fleet_states] --- C2[launch detection adapter node] --> DB
  C1 --> Driver
  C2 --> Node
  Proximity --> Update(update existing traffic participant) 
  Update --- U1[publish new location to /fleet_states] --> TimerReset
  CountdownTimer[countdown timer] --- TimerReset[reset timer]
  CountdownTimer --- TimerStop[countdown timer stops] --> Delete(delete existing traffic participant)
  Delete --- D1[remove fleet driver app and fleet adapter node] --> DB
  style Subscriber stroke:#f66,stroke-width:4px
  style Create stroke:#f66,stroke-width:4px
  style Update stroke:#f66,stroke-width:4px
  style Delete stroke:#f66,stroke-width:4px
  style FindClass fill:#ccf,stroke-width:2px,stroke-dasharray: 5, 5
  style Proximity fill:#ccf,stroke-width:2px,stroke-dasharray: 5, 5
  style CountdownTimer fill:#ccf,stroke-width:2px,stroke-dasharray: 5, 5
  style DB fill:#ccf,stroke-width:2px,stroke-dasharray: 5, 5

end
```

design reference to confluence page(https://imda-dsl.atlassian.net/wiki/spaces/VAMA/pages/470286350/Alternate+flow+for+lift+integration+and+obstacle)

## components details

- adapter driver: similar to fleet adapater drivers where its job is to transmit `rmf_fleet_msgs/FleetState` messages out to the `fleet_states` topic. It is a ROS 2 application (using either rclcpp or rclpy) which we will refer to as the Fleet Driver.

## software design

1. Proximity checker 
- for wheelchair (dynamic object): 2m x width of traffic lane
- for cones (static object): 0.5m x width of traffic lane

2. Countdown Timer to check if existing traffic participant exceed a time threshold. This may need multi-threading. 
- Each thread created for a new traffic participant and start counting down. Once countdown is up, traffic particpant would be deleted. 
- If existing participant gets updated, countdown should reset. 

## test scenario

1. static images 

- location of these detection would change according to the current location of robot as camera location is at a relative location. 

## TODO:

1. how to run launch.xml file using ROS python library? to launch fleet adapter node in rmf_fleet_adapter package.

## follow-up

1. `detection_states` topic be created that is integrated with `rmf_traffic`. detection traffic participants are riding on `fleet_states` topic since `rmf_fleet_adapter` library is utilised. 

