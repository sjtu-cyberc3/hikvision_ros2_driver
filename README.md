# 海康威视工业相机ROS2驱动

支持列表：

* GIGE相机（测试相机：MV-CS016-10GC）

* USB相机（未测试）

---

依赖项安装：

1. 在官网下载并安装MVS驱动包。

2. 运行MVS可视化工具，配置相机参数。设置相机名称(user_name, device_id)，本项目将根据相机名称寻找对应设备。

---

独立节点（standalone）运行方式：

```bash
ros2 launch hikvision_ros2_driver standalone.launch.yaml camera_name:=<camera_name>
```

进程内通信（component）运行方式：

```bash
ros2 launch hikvision_ros2_driver component.launch.yaml camera_name:=<camera_name> container_name:=<container_name>
```

* 需要将camera_name替换成对应相机的名称。

* 对于进程内通信，需要提前启动`component_container`，并将container_name设置为对应container的名称。

---

发布话题：

* raw/image [sensor_msgs/msg/Image]: 相机原始数据，格式可以在MVS工具中修改，默认情况下应该是bayer格式。
* info [hikvision_interface/msg/HikImageInfo]: 图像元数据，包括时间戳、曝光长度、增益大小、白平衡参数等。

---

补充:

* 不建议对bayer格式的原始数据使用jpeg压缩。即当相机原始数据格式为bayer时，不建议订阅`raw/image/compressed`话题。

* rosbag录制时建议开启zstd压缩以节省空间。(ros2 bag record -s mcap --storage-preset-profile zstd_fast)