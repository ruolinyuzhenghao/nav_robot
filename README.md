# nav_robot
## 介绍
这是一个用于小车或双体无人船自主导航的项目！  
功能模块有全覆盖路径规划(ipa_coverage_planning-noetic_dev)、多目标点巡航与视觉追踪(darknet_yolo)、ros主机与上位机通信发布地图边缘点(pub_map)以及水面船与水下机器人协同导航(pub_odom)。
## 环境
+ ubutnu 20.04
+ Ros noetic
+ opencv 4.6.0
+ python 3.8
## 运行
```
#全覆盖路径规划
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
roslaunch ipa_room_exploration room_exploration_action_server.launch
roslaunch ipa_room_exploration room_exploration_client.launch
#多目标点巡航与视觉追踪
rosrun darknet_yolo xunhang
#ros主机与上位机通信发布地图边缘点
rosrun pub_map pub_mappoints
rosrun pub_map cut_map
#水面船与水下机器人协同导航
rosrun pub_odom pubtostm32
```
## 个人教程
多机器人协同导航：https://www.yuque.com/g/ruolinyuzhenghao/burc03/evv98fhudk6bzq85/collaborator/join?token=oFVUALtOycIosIpL&source=doc_collaborator#
## 引用声明
全覆盖路径规划节点参考[ipa_coverage_planning](https://gitcode.com/ipa320/ipa_coverage_planning.git)，增加了与手机APP通信与可自定义遍历区域的算法。
