# traffic_participant_controller

api for create update and delete of traffic participant. specifically from scene understanding manager with wheelchair and cone detection only. 

```mermaid
graph TB

  Publisher --> Subscriber
  subgraph "Scene Uds Manager"
  Publisher(publisher topic for all detections)
  end

  subgraph "Traffic Particpant Controller"
  Subscriber(subscriber topic for all detections)
  Subscriber --> FindClass[filter for wheelchair and cones detections only] --> Detection((Detection Object))
  Detection --> Proximity[check for proximity based on detection type]
  Proximity -- location of object --> Create(create traffic participant)
  Create -- launch read-only fleet adapter ROS node w location param --> Save(save in-memory db)
  Proximity --> Update(update existing traffic participant)
  CountdownTimer[check if existing traffic participant exceed a time threshold] --> Delete(delete existing traffic participant)

end
```

design reference to confluence page(https://imda-dsl.atlassian.net/wiki/spaces/VAMA/pages/470286350/Alternate+flow+for+lift+integration+and+obstacle)

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

