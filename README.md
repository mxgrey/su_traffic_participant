# traffic_participant_controller

api for create update and delete of traffic participant. specifically from scene understanding manager with wheelchair and cone detection only. 

```mermaid
graph TB

  Publisher --> Node2
  subgraph "Scene Uds Manager"
  Publisher(Publisher topic)
  end

  subgraph "Traffic Particpant Controller"
  Node1[Node 1] --> Node2[Node 2]
  Subscriber(Subscriber topic)
  Subscriber -- filter for wheelchair and cones detections only --> database
end
```

```mermaid
graph TB

  SubGraph1 --> SubGraph1Flow
  subgraph "SubGraph 1 Flow"
  SubGraph1Flow(SubNode 1)
  SubGraph1Flow -- Choice1 --> DoChoice1
  SubGraph1Flow -- Choice2 --> DoChoice2
  end

  subgraph "Main Graph"
  Node1[Node 1] --> Node2[Node 2]
  Node2 --> SubGraph1[Jump to SubGraph1]
  SubGraph1 --> FinalThing[Final Thing]
end
```


```mermaid
graph TD;
  A-->B;
  A-->C;
  B-->D;
  C-->D;
```
