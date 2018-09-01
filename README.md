# yolov3-ros_service_tx2

对应csdn: [https://blog.csdn.net/Dbox_boom/article/details/82286254#ros服务介绍][1]

demo project of an yolo-v3 object-detecting ROS server.

The detect_proc fold defines the srv.
```
sensor_msgs/Image img
---
int32 top_left_x
int32 top_left_y
int32 bottom_right_x
int32 bottom_right_y
int32 lablel
```

the detect_server fold contains the sample server code, and the libs fold in it contains the darknet source code with a Makefile to genrate the shared library libdarknet.so

the demo_client fold contains the  sample client code.

the project is just a demo and it didn't contain the model file. you shold do it yourself.
___________
[1]:https://blog.csdn.net/Dbox_boom/article/details/82286254#ros服务介绍
