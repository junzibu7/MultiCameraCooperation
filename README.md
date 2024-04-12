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

