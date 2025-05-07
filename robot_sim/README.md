# Robot Simulator Package

This package simulate robot topics and request, it is useful for testing
before implementing the bridge between the packages and the robot. (All of these functionalities can be changed by required needs).

> The mark beside the functionalities means if it is implemented

### Topics 

- **Whisper Topic** [x]
    - Topic type: ```Publisher```
    - Topic name: ```/sancho/whisper```
    - Message type: ```std_msgs/msg/String```

- **TTS Topic** [ ]
    - Topic type: ```Subscription```
    - Topic name: ```/sancho/sound```
    - Message type: ```std_msgs/msg/String```

