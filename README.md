# traffic_participant_controller

api for create update and delete of traffic participant. specifically from scene understanding manager with wheelchair and cone detection only. 

```mermaid
graph TB

  Publisher --> Subscriber
  subgraph "Scene Uds Manager"
  Publisher(Publisher topic)
  end

  subgraph "Traffic Particpant Controller"
  Node1[Node 1] --> Node2[Node 2]
  Subscriber(Subscriber topic)
  Subscriber -- filter for wheelchair and cones detections only --> database
end
```
