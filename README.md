# remoteimu
Library rebuilding IMU attitude from IMU data send through UDP socket

## Installation instructions
### Dependencies
* [state-observation] (you might have to install my [fork] in case the latest pull request has not been accepted).
* Optionnally, [gepetto-viewer-corba], if you want to try it directly after installation.

## Instructions
### Try it
* On your Android phone, install [SensorStream IMU+GPS].
* Launch it and enter the hostname corresponding to your computer and the port 6000.
* Select UDP Stream and start streaming.
* On your computer, launch `remoteimu-viz`.

### Developpers
Create a class that inherits from `remoteimu::MouseEventSender` and reimplement the method `mouseEvent`.

```c++
UDPServer* server (new UDPServer (host, port));
Mouse mouse (server, 60);
MyMouseEventSender* myEventSender = new MyMouseEventSender();
mouse.setMouseEventSender (myEventSender);
mouse.handleEvents (true);
```

## To do
* Enhancing the SensorDataParser in order to handle inputs from other streams.

[state-observation]:https://github.com/stack-of-tasks/sot-state-observation
[fork]:https://github.com/jmirabel/state-observation
[gepetto-viewer-corba]:https://github.com/humanoid-path-planner/gepetto-viewer-corba
[Sensorstream IMU+GPS]:https://play.google.com/store/apps/details?id=de.lorenz_fenster.sensorstreamgps
