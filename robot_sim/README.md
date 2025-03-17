# Robot Simulator Package

This package simulate robot topics and request, it is useful for testing
before implementing the bridge between the packages and the robot. (All of these functionalities can be changed by required needs).

> The mark beside the functionalities means if it is implemented

### Topics 

- **Move Topic** [ ]
    - Topic type: ```Subscription```
    - Topic name: ```/sancho/move```
    - Message type: ```geomtry_msgs/msg/Point32```

- **Whisper Topic** [ ]
    - Topic type: ```Subscription```
    - Topic name: ```/sancho/whisper```
    - Message type: ```std_msgs/msg/String```

- **Play Sound Topic** [ ]
    - Topic type: ```Subscription```
    - Topic name: ```/sancho/sound```
    - Message type: ```-```

- **Battery Topic** [ ]
    - Topic type: ```Publisher```
    - Topic name: ```/sancho/battery```
    - Message type: ```std_msgs/msg/Int32```

### Requests
- Large Language Model Service [ ]
    - Request node: **-**
    - Request type: **-**