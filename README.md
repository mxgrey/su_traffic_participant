# traffic_participant_controller

api for create update and delete of traffic participant. specifically from scene understanding manager with wheelchair and cone detection only. 

```mermaid
graph TD;
    subgraph "Scene Uds Manager"
    Publisher(Publisher topic)
    Publisher --> Subscriber: all types of detections

    subgraph "Traffic Particpant Controller"
    Subscriber(Subscriber topic)
    Subscriber -- filter for wheelchair and cones detections only --> database
```

```mermaid
graph TD;
  A-->B;
  A-->C;
  B-->D;
  C-->D;
```
