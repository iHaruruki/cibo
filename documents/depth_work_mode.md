ros2 run orbbec_camera list_depth_work_mode_node 
[10/17 09:08:58.633073][info][7548][Context.cpp:68] Context created with config: default config!
[10/17 09:08:58.633086][info][7548][Context.cpp:73] Work directory=/home/robot/ros2_ws, SDK version=v1.10.27-20250925-0549823
[10/17 09:08:58.633093][info][7548][LinuxPal.cpp:32] createObPal: create LinuxPal!
[10/17 09:08:58.977132][warning][7548][OpenNIDeviceInfo.cpp:190] New openni device matched.
[10/17 09:08:58.977142][warning][7548][OpenNIDeviceInfo.cpp:190] New openni device matched.
[10/17 09:08:58.977263][info][7548][LinuxPal.cpp:166] Create PollingDeviceWatcher!
[10/17 09:08:58.977352][info][7548][DeviceManager.cpp:15] Current found device(s): (2)
[10/17 09:08:58.977377][info][7548][DeviceManager.cpp:24] 	- Name: SV1301S_U3, PID: 0x0614, SN/ID: , Connection: USB3.0
[10/17 09:08:58.977381][info][7548][DeviceManager.cpp:24] 	- Name: SV1301S_U3, PID: 0x0614, SN/ID: , Connection: USB3.0
[10/17 09:08:58.977386][info][7548][Pipeline.cpp:15] Try to create pipeline with default device.
[10/17 09:08:58.978072][info][7548][OpenNIHostProtocol.cpp:722] Hardware versions: FW=5.8.23 (14), HW=0, Chip=7, Sensor=0, SYS=12
[10/17 09:08:58.978501][warning][7548][OpenNIHostProtocol.cpp:739] Get usb core type failed!
[10/17 09:08:58.980876][info][7548][OpenNISensorFirmware.cpp:1190] Sensor serial number:AY0F7010783
[10/17 09:08:58.981227][info][7548][OpenNISensorFirmware.cpp:1218] Firmware version RD3013
[10/17 09:08:58.981536][info][7548][OpenNISensorFirmware.cpp:1224] Device frequency 50
[10/17 09:08:59.019966][warning][7548][OpenNIHostProtocol.cpp:1112] Host Protocol sub cmd not supported!
[10/17 09:08:59.019977][warning][7548][OpenNISensorFirmware.cpp:153] OpenNI2 camera don't support Watchdog function!
[10/17 09:08:59.020106][info][7548][AbstractDevice.cpp:120] 	- Firmware version: RD3013
[10/17 09:08:59.020319][info][7548][OpenNIDevice.cpp:821] Init depth filter params.
[10/17 09:08:59.020328][info][7548][OpenNIDevice.cpp:40] OpenNI device created! PID: 0x0614, SN: AY0F7010783
[10/17 09:08:59.020334][info][7548][DeviceManager.cpp:150] Device created successfully! Name: SV1301S_U3, PID: 0x0614, SN/ID: 
[10/17 09:08:59.020362][warning][7548][ObException.cpp:5] Current device does not support frame sync!
[10/17 09:08:59.020440][warning][7548][Pipeline.cpp:45] Execute failure! A libobsensor_exception has occurred!
	 - where:45#Pipeline
	 - msg:Current device does not support frame sync!
	 - type:N11libobsensor31unsupported_operation_exceptionE
[10/17 09:08:59.020468][info][7548][Pipeline.cpp:47] Pipeline created with device: {name: SV1301S_U3, sn: AY0F7010783}, @0x572C2B957D90
Current device not support depth work mode!
[10/17 09:08:59.020485][info][7548][Pipeline.cpp:75] Pipeline destroyed! @0x572C2B957D90
[10/17 09:08:59.020509][info][7548][OpenNIDevice.cpp:44] OpenNI device destroyed! PID: 0x0614, SN: AY0F7010783
[10/17 09:08:59.020883][info][7548][Context.cpp:84] Context destroyed
[ros2run]: Process exited with failure 255
