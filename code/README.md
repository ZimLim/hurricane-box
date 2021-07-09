# Code Readme

Hurricane Box
Quest 3

Similar to a flight data recorder, our device would make recordings based on what we have in the course kit.
The key features of the hurrican box are:

Measurements temperature, acceleration, and possibly other values (such as battery level)
Data sent from ESP32 sensors over WiFi
Remote access through a portal to display a real-time data at remote client, in graphical form
Remote access through a portal to control LEDs and to configure features at a remote client
Webcam sourcing video, embedded in a single portal window

We were successfully able to: connect wirelessly and have remote access, control the alert LED from a remote interface with a simple on/off concept, prove a status report of temperature, vibration, battery level in real-time at remote client in strip-chart or equivalent way, using a web browser. 

Investigative question: What are steps you can take to make your device and system low power? 
1) the LED can send short pulse width modulation or in other words the voltage can output for less time
2) Also you can use pwm to power the whole system that way the device only turns on periodically and doesn’t output continuous voltage

Hazim Halim, Nirmal Patel, Yasmin Almousa
