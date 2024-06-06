#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>

class cam_pos_reader{
public:
    cam_pos_reader(){                                                  
        sub_uav_ = nh_.subscribe("/csj01/mocap/pos", 1, &cam_pos_reader::uav_cb, this);
        sub_pnp_ = nh_.subscribe("/camA/single_cam_process_ros/ir_mono/T_cam_to_estimation", 1, &cam_pos_reader::pnp_cb, this);
        sub_imu_ = nh_.subscribe("/IMU_data", 1, &cam_pos_reader::imu_cb, this);
        // imu orientation
        pub_cam_ = nh_.advertise<geometry_msgs::PoseStamped>("/cam_pos", 1);
        filename = "/home/cius/noetic_ws/src/multi_camera_cooperation/data/cam_pos3.txt";
        uav_mat = Eigen::Matrix4d::Identity();
        pnp_mat = Eigen::Matrix4d::Identity();
        cam_mat = Eigen::Matrix4d::Identity();
        uav_flag = false;
        pnp_flag = false;
        imu_flag = false;

        // 清空当前.txt文件
        std::ofstream file;
        file.open(filename, std::ios::out | std::ios::trunc);
        if (file.is_open()) {
            file.close();
        } else {
            ROS_ERROR("Unable to open file: %s", filename.c_str());
        }
    }

    void uav_cb(const geometry_msgs::PoseStamped& msg) {
        uav_mat.block<3, 1>(0, 3) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
        // Eigen::Quaterniond q(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
        // uav_mat.block<3, 3>(0, 0) = q.toRotationMatrix();
        uav_flag = true;
    }

    void imu_cb(const sensor_msgs::Imu& msg) {
        Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
        uav_mat.block<3, 3>(0, 0) = q.toRotationMatrix();
        imu_flag = true;
    }

    void pnp_cb(const geometry_msgs::TransformStamped& msg) {
        pnp_mat.block<3, 1>(0, 3) << msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z;
        Eigen::Quaterniond q(msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z);
        pnp_mat.block<3, 3>(0, 0) = q.toRotationMatrix();
        pnp_flag = true;
        if(uav_flag && pnp_flag && imu_flag) {
            std::cout<<pnp_mat<<std::endl;
            cam_mat = pnp_mat.inverse() * uav_mat;
            cam_msg.header.stamp = ros::Time::now();
            cam_msg.pose.position.x = cam_mat(0, 3);
            cam_msg.pose.position.y = cam_mat(1, 3);
            cam_msg.pose.position.z = cam_mat(2, 3);
            Eigen::Quaterniond quaternion(cam_mat.block<3,3>(0,0));
            cam_msg.pose.orientation.w = quaternion.w();
            cam_msg.pose.orientation.x = quaternion.x();
            cam_msg.pose.orientation.y = quaternion.y();
            cam_msg.pose.orientation.z = quaternion.z();
            pub_cam_.publish(cam_msg);

            // // // 记录当前帧位置数据
            // ROS_INFO("Cam_pos: (%.3f, %.3f, %.3f)", cam_mat(0, 3), cam_mat(1, 3), cam_mat(2, 3));
            std::ofstream file;
            file.open(filename, std::ios::app);
            if(file.is_open()) {
                file << cam_mat(0, 3) << "    ";
                file << cam_mat(1, 3) << "    ";
                file << cam_mat(2, 3) << "    ";
                file << cam_msg.pose.orientation.w << "    ";
                file << cam_msg.pose.orientation.x << "    ";
                file << cam_msg.pose.orientation.y << "    ";
                file << cam_msg.pose.orientation.z << "    ";
                file << '\n';
            }
            else {
                ROS_ERROR("Unable to open file: %s", filename.c_str());
            }

            uav_flag = false; }
    }
    ros::NodeHandle nh_;
    ros::Subscriber sub_uav_;
    ros::Subscriber sub_pnp_;
    ros::Subscriber sub_imu_;
    ros::Publisher pub_cam_;
    Eigen::Matrix4d uav_mat;
    Eigen::Matrix4d pnp_mat;
    Eigen::Matrix4d cam_mat;
    geometry_msgs::PoseStamped cam_msg;
    bool uav_flag;
    bool pnp_flag;
    bool imu_flag;
    std::string filename;
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "cam_pos_reader");
    std::cout<<"cam_pos_reader start..."<<std::endl;
    cam_pos_reader reader;
    ros::spin();
    return 0;
}