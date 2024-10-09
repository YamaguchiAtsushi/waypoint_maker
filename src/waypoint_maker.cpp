#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <iostream>
#include <cmath>  // atan2関数のため

class WaypointMaker {
public:
    WaypointMaker() {
        // ROSノードハンドルを作成
        ros::NodeHandle nh;

        // Joyメッセージのサブスクライバ
        joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &WaypointMaker::joyCallback, this);

        // 自己位置のサブスクライバ
        pose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10, &WaypointMaker::amclPoseCallback, this);

        // マーカーパブリッシャ
        marker_pub_ = nh.advertise<visualization_msgs::Marker>("waypoint_markers", 10);

        // cmd_velのパブリッシャ
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 10);

        // 初期値を設定
        current_pose_.position.x = 0.0;
        current_pose_.position.y = 0.0;
        current_pose_.orientation.z = 0.0;
        current_pose_.orientation.w = 1.0;
    }

    void run() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();

            // Joy-Conボタンが押された場合にウェイポイントを保存
            if (save_waypoint_) {
                // デバッグ: 現在の位置を表示して確認
                ROS_INFO("現在位置: x = %f, y = %f", current_pose_.position.x, current_pose_.position.y);

                saveWaypointToCSV(current_pose_);

                // Rviz用のマーカーをパブリッシュ
                visualization_msgs::Marker marker = createArrowMarker(waypoint_id_++, current_pose_);
                marker_pub_.publish(marker);

                // 保存フラグをリセット
                save_waypoint_ = false;
            }

            rate.sleep();
        }
    }

private:
    // メンバー変数
    geometry_msgs::Pose current_pose_;
    bool save_waypoint_ = false;  // Joy-Conボタンの押下フラグ
    std::string csv_file_path_ = "/home/yamaguchi-a/catkin_ws/src/waypoint_maker/csv/waypoints.csv";  // CSVファイルのパス
    ros::Subscriber joy_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher cmd_vel_pub_;  // cmd_velのパブリッシャ
    int waypoint_id_ = 0;  // ウェイポイントのID

    // Joyメッセージのコールバック
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
        // ボタン2（○ボタン）でウェイポイントを保存
        if (joy_msg->buttons[2] == 1) {
            save_waypoint_ = true;
        }

        // ジョイスティックの入力に基づいて速度コマンドを作成
        geometry_msgs::Twist twist;
        twist.linear.x = joy_msg->axes[3];  // 右スティック上下で前進後退
        twist.angular.z = joy_msg->axes[0];  // 左スティック左右で旋回
        cmd_vel_pub_.publish(twist);  // /cmd_velトピックにパブリッシュ
    }

    // amcl_poseコールバック関数
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        // msg->pose.poseを使用してgeometry_msgs::Pose型にアクセス
        current_pose_ = msg->pose.pose;

        // デバッグメッセージを表示
        ROS_INFO("Position: x = %f, y = %f", current_pose_.position.x, current_pose_.position.y);
        ROS_INFO("Orientation: z = %f, w = %f", current_pose_.orientation.z, current_pose_.orientation.w);
    }

    // CSVファイルにウェイポイントを保存
    void saveWaypointToCSV(const geometry_msgs::Pose& pose) {
        std::ofstream file(csv_file_path_, std::ios::app);
        if (file.is_open()) {
            // 四元数からヨー角を計算
            double yaw = atan2(2.0 * (pose.orientation.w * pose.orientation.z), 1.0 - 2.0 * (pose.orientation.z * pose.orientation.z));
            
            // x, y, yawをCSVに書き込む
            file << pose.position.x << "," << pose.position.y << "," << yaw << "\n";
            file.close();
            ROS_INFO("ウェイポイントを保存しました: [%f, %f, %f]", pose.position.x, pose.position.y, yaw);
        } else {
            ROS_ERROR("CSVファイルを開けませんでした");
        }
    }

    // Rviz用の矢印マーカーを作成
    visualization_msgs::Marker createArrowMarker(int id, const geometry_msgs::Pose& pose) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";  // 使用する座標系を指定
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoints";
        marker.id = id;
        marker.type = visualization_msgs::Marker::ARROW;  // 矢印マーカーを指定
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose;

        // 矢印の長さと幅、高さを設定
        marker.scale.x = 0.3;  // 矢印の長さ
        marker.scale.y = 0.1;  // 矢印の幅
        marker.scale.z = 0.0;  // 矢印の高さ

        // 矢印の色を設定 (赤色の矢印)
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;  // アルファ値（透明度）

        return marker;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_maker");
    
    WaypointMaker waypoint_maker;  // WaypointMakerオブジェクトを作成
    waypoint_maker.run();           // プログラムの実行

    return 0;
}
