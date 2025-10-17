ros2 run orbbec_camera list_camera_profile_mode_node 
[10/17 09:11:34.127537][info][7958][Context.cpp:68] Context created with config: default config!
[10/17 09:11:34.127547][info][7958][Context.cpp:73] Work directory=/home/robot/ros2_ws, SDK version=v1.10.27-20250925-0549823
[10/17 09:11:34.127554][info][7958][LinuxPal.cpp:32] createObPal: create LinuxPal!
[10/17 09:11:34.474972][warning][7958][OpenNIDeviceInfo.cpp:190] New openni device matched.
[10/17 09:11:34.474980][warning][7958][OpenNIDeviceInfo.cpp:190] New openni device matched.
[10/17 09:11:34.475057][info][7958][LinuxPal.cpp:166] Create PollingDeviceWatcher!
[10/17 09:11:34.475088][info][7958][DeviceManager.cpp:15] Current found device(s): (2)
[10/17 09:11:34.475114][info][7958][DeviceManager.cpp:24] 	- Name: SV1301S_U3, PID: 0x0614, SN/ID: , Connection: USB3.0
[10/17 09:11:34.475116][info][7958][DeviceManager.cpp:24] 	- Name: SV1301S_U3, PID: 0x0614, SN/ID: , Connection: USB3.0
[10/17 09:11:34.475120][info][7958][Pipeline.cpp:15] Try to create pipeline with default device.
[10/17 09:11:34.475720][info][7958][OpenNIHostProtocol.cpp:722] Hardware versions: FW=5.8.23 (14), HW=0, Chip=7, Sensor=0, SYS=12
[10/17 09:11:34.475947][warning][7958][OpenNIHostProtocol.cpp:739] Get usb core type failed!
[10/17 09:11:34.478069][info][7958][OpenNISensorFirmware.cpp:1190] Sensor serial number:AY0F7010783
[10/17 09:11:34.478417][info][7958][OpenNISensorFirmware.cpp:1218] Firmware version RD3013
[10/17 09:11:34.478631][info][7958][OpenNISensorFirmware.cpp:1224] Device frequency 50
[10/17 09:11:34.516349][warning][7958][OpenNIHostProtocol.cpp:1112] Host Protocol sub cmd not supported!
[10/17 09:11:34.516357][warning][7958][OpenNISensorFirmware.cpp:153] OpenNI2 camera don't support Watchdog function!
[10/17 09:11:34.516448][info][7958][AbstractDevice.cpp:120] 	- Firmware version: RD3013
[10/17 09:11:34.516634][info][7958][OpenNIDevice.cpp:821] Init depth filter params.
[10/17 09:11:34.516640][info][7958][OpenNIDevice.cpp:40] OpenNI device created! PID: 0x0614, SN: AY0F7010783
[10/17 09:11:34.516666][info][7958][DeviceManager.cpp:150] Device created successfully! Name: SV1301S_U3, PID: 0x0614, SN/ID: 
[10/17 09:11:34.516678][warning][7958][ObException.cpp:5] Current device does not support frame sync!
[10/17 09:11:34.516723][warning][7958][Pipeline.cpp:45] Execute failure! A libobsensor_exception has occurred!
	 - where:45#Pipeline
	 - msg:Current device does not support frame sync!
	 - type:N11libobsensor31unsupported_operation_exceptionE
[10/17 09:11:34.516731][info][7958][Pipeline.cpp:47] Pipeline created with device: {name: SV1301S_U3, sn: AY0F7010783}, @0x64654293F3E0
[10/17 09:11:34.516770][info][7958][OpenNIDevice.cpp:689] IR sensor has been created!
OB_SENSOR_IR profile: 640x400 30fps OB_FORMAT_Y10
OB_SENSOR_IR profile: 320x200 5fps OB_FORMAT_Y10
OB_SENSOR_IR profile: 320x200 10fps OB_FORMAT_Y10
OB_SENSOR_IR profile: 320x200 15fps OB_FORMAT_Y10
OB_SENSOR_IR profile: 320x200 30fps OB_FORMAT_Y10
OB_SENSOR_IR profile: 320x200 60fps OB_FORMAT_Y10
OB_SENSOR_IR profile: 640x400 5fps OB_FORMAT_Y10
OB_SENSOR_IR profile: 640x400 10fps OB_FORMAT_Y10
OB_SENSOR_IR profile: 640x400 15fps OB_FORMAT_Y10
OB_SENSOR_IR profile: 640x400 60fps OB_FORMAT_Y10
OB_SENSOR_IR profile: 1280x800 15fps OB_FORMAT_Y10
OB_SENSOR_IR profile: 1280x800 30fps OB_FORMAT_Y10
[10/17 09:11:34.536888][info][7958][OpenNIDevice.cpp:641] Color sensor has been created!
OB_SENSOR_COLOR profile: 640x480 30fps OB_FORMAT_MJPG
OB_SENSOR_COLOR profile: 1920x1080 30fps OB_FORMAT_MJPG
OB_SENSOR_COLOR profile: 1280x720 30fps OB_FORMAT_MJPG
OB_SENSOR_COLOR profile: 640x480 60fps OB_FORMAT_MJPG
OB_SENSOR_COLOR profile: 640x480 15fps OB_FORMAT_MJPG
OB_SENSOR_COLOR profile: 640x480 10fps OB_FORMAT_MJPG
OB_SENSOR_COLOR profile: 640x480 5fps OB_FORMAT_MJPG
OB_SENSOR_COLOR profile: 1920x1080 30fps OB_FORMAT_RGB
OB_SENSOR_COLOR profile: 1280x720 30fps OB_FORMAT_RGB
OB_SENSOR_COLOR profile: 640x480 60fps OB_FORMAT_RGB
OB_SENSOR_COLOR profile: 640x480 30fps OB_FORMAT_RGB
OB_SENSOR_COLOR profile: 640x480 15fps OB_FORMAT_RGB
OB_SENSOR_COLOR profile: 640x480 10fps OB_FORMAT_RGB
OB_SENSOR_COLOR profile: 640x480 5fps OB_FORMAT_RGB
OB_SENSOR_COLOR profile: 1920x1080 30fps OB_FORMAT_BGRA
OB_SENSOR_COLOR profile: 1280x720 30fps OB_FORMAT_BGRA
OB_SENSOR_COLOR profile: 640x480 60fps OB_FORMAT_BGRA
OB_SENSOR_COLOR profile: 640x480 30fps OB_FORMAT_BGRA
OB_SENSOR_COLOR profile: 640x480 15fps OB_FORMAT_BGRA
OB_SENSOR_COLOR profile: 640x480 10fps OB_FORMAT_BGRA
OB_SENSOR_COLOR profile: 640x480 5fps OB_FORMAT_BGRA
[10/17 09:11:34.537181][info][7958][OpenNIDevice.cpp:395] Depth sensor has been created!
OB_SENSOR_DEPTH profile: 640x400 30fps OB_FORMAT_Y11
OB_SENSOR_DEPTH profile: 320x200 5fps OB_FORMAT_Y11
OB_SENSOR_DEPTH profile: 320x200 5fps OB_FORMAT_Y12
OB_SENSOR_DEPTH profile: 320x200 10fps OB_FORMAT_Y11
OB_SENSOR_DEPTH profile: 320x200 10fps OB_FORMAT_Y12
OB_SENSOR_DEPTH profile: 320x200 15fps OB_FORMAT_Y11
OB_SENSOR_DEPTH profile: 320x200 15fps OB_FORMAT_Y12
OB_SENSOR_DEPTH profile: 320x200 30fps OB_FORMAT_Y11
OB_SENSOR_DEPTH profile: 320x200 30fps OB_FORMAT_Y12
OB_SENSOR_DEPTH profile: 320x200 60fps OB_FORMAT_Y11
OB_SENSOR_DEPTH profile: 320x200 60fps OB_FORMAT_Y12
OB_SENSOR_DEPTH profile: 640x400 5fps OB_FORMAT_Y11
OB_SENSOR_DEPTH profile: 640x400 5fps OB_FORMAT_Y12
OB_SENSOR_DEPTH profile: 640x400 10fps OB_FORMAT_Y11
OB_SENSOR_DEPTH profile: 640x400 10fps OB_FORMAT_Y12
OB_SENSOR_DEPTH profile: 640x400 15fps OB_FORMAT_Y11
OB_SENSOR_DEPTH profile: 640x400 15fps OB_FORMAT_Y12
OB_SENSOR_DEPTH profile: 640x400 30fps OB_FORMAT_Y12
OB_SENSOR_DEPTH profile: 640x400 60fps OB_FORMAT_Y11
OB_SENSOR_DEPTH profile: 640x400 60fps OB_FORMAT_Y12
OB_SENSOR_DEPTH profile: 1280x800 15fps OB_FORMAT_Y11
OB_SENSOR_DEPTH profile: 1280x800 15fps OB_FORMAT_Y12
OB_SENSOR_DEPTH profile: 1280x800 30fps OB_FORMAT_Y11
OB_SENSOR_DEPTH profile: 1280x800 30fps OB_FORMAT_Y12
Current device not support depth work mode!
[10/17 09:11:34.537419][info][7958][Pipeline.cpp:75] Pipeline destroyed! @0x64654293F3E0
[10/17 09:11:34.537427][info][7958][OpenNIDevice.cpp:44] OpenNI device destroyed! PID: 0x0614, SN: AY0F7010783
[10/17 09:11:34.537433][info][7958][OpenNIVideoSensor.cpp:1021] OpenNIVideoSensor destroyed! sensorType=OB_SENSOR_IR
[10/17 09:11:34.537440][info][7958][VideoSensor.cpp:303] VideoSensor destroyed, @OB_SENSOR_COLOR
[10/17 09:11:34.537446][info][7958][OpenNIVideoSensor.cpp:1021] OpenNIVideoSensor destroyed! sensorType=OB_SENSOR_DEPTH
[10/17 09:11:34.551981][info][7958][ObLibuvcDevicePort.cpp:70] uvc_close done.
[10/17 09:11:34.551998][info][7958][ObLibuvcDevicePort.cpp:71] ~ObLibuvcDevicePort done
