# FisheyeCalib
Aims at calibration of fisheye camera

Fisheye camera calibration aims at calibrating the camera to get a straight image on which operations could be made. Theory files and the Calibration.cpp directly from the Opencv has been attached. It's important as MAV localisation depends upon it.

First of all. See the calibration.cpp. There it is ought not to be complied using the cmake . and make. Just doing 
g++ calibration.cpp
is enough. It would generate an a.out file. A default.xml file is to be made which has the camera parameters(it is attached too). 
After the calibration a out_camera_data.xml and out_camera_data.yml is generated(this case attached to it). Using this data we would do the final calibration
See the closed issues link to get knowledge about the problems faced and solved. Open issues are yet to be solved 

You can get the alibrate.cpp file from opencv git link too. Link:- opencv/samples/cpp/tutorial_code/calib3d/camera_calibration.

change the name in_VID5.xml to deafault.xml/
