# multi_camera_cooperation

多相机协同观测并估计无人机的状态。

主动视觉部分见https://gitee.com/hezijia/ftservo-control

## 使用方法

### Step1

准备飞机的配置文件，下载 https://gitee.com/hezijia/uav_config.git 处代码，并按照其要求进行配置。

本项目中的配置文件为uav_config/extrinsic_intrinsic/body_four_camera.yaml。

该配置文件由部署的四个相机的内外参以及无人机搭载设备的相关外参组成，其中IRLandmark项需根据人工标记点的布置进行修改、增加和删除。

IRLandmark由四项构成：
- layout_name:红外标记的命名
- number:红外标记点的数量
- layout:红外标记点的布置方案。选取一个红外标记坐标系（一般选取其中一个红外标记点作为坐标系原点），并在layout的第i行中填入第i个红外标记点的坐标
- T_drone_IRLandmark:红外标记坐标系相对于无人机坐标系的变换矩阵

注意对yaml文件进行项的增加或减少时，需要对uav_config/include/uav_config/read_config_drone.h进行相应的修改。


### Step2

对CMakeLists.txt进行配置。

注意更改包括opencv、cv_bridge、uav_config在内的文件路径，若下载到系统默认位置则可以把对应的系统路径设置注释掉。

### Step3

开始编译，一般可以编译通过，有些文件可能在第一次编译的时候才生成，需要编译两次才能成功。

如果还有问题请联系我。

### Step4

下面介绍功能包的主要文件：
- base2servogp_broadcaster.cpp:记录base坐标系到servogroupxx坐标系的坐标变换，并tf树的形式广播出去。
- landmark_detect.cpp:捕获相机的图像，识别红外标记在图像上的坐标，并发布话题"ir_mono/marker_pixel"。
- pnp_cooperation_node_ros.cpp:监听四组相机对无人机位姿的估计值，将其融合得到无人机的最终位姿。
- pnp_target_node_ros.cpp:订阅"ir_mono/marker_pixel"话题，结合红外标记在图像上的坐标和相机的内外参，使用pnp算法解算无人机的位置和姿态。

pnp_target_node_ros.cpp 文件的主要函数为 `landmark_pose_solve()`,其将得到的红外标记按照配置进行排序后进行pnp解算，得到无人机的位姿后存入`T_cam_to_estimation`中并以tf树的形式广播出去。
# multi_camera_cooperation
