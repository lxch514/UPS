modbus485.cpp ：实现通过modbus协议的485通信，可以对485的X、Y进行操作
ups_test.cpp : 完整的UPS供电逻辑，当市电恢复时，通过人工确认，进行下一步操作
test_7575 : 风机的手动控制所有功能，也是通过modbus协议，连接3588的485通信实现的
BLR_4000：激光测距
SINDT_485:陀螺仪测角度、GPS定位经纬度等
BLR_SINDT：合并BLR_4000和SINDT_485两个模块到一个串口，同时实现测距、角度、定位等
BLR_SINDT_dual：合并BLR_4000和SINDT_485两个模块到两个串口
AFF500RS.cpp：风速传感器测量风速
multi_sensor.cpp：合并BLR_4000、SINDT_485和AFF500RS三个传感器，用同一个串口通过轮询、多线程工作 