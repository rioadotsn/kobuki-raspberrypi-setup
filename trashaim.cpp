#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// 計算 X 和 Y 軸的誤差
void calculateError(const cv::Point& target_center, const cv::Size& frame_size, int& error_x, int& error_y) {
    cv::Point frame_center(frame_size.width / 2, frame_size.height / 2);
    error_x = target_center.x - frame_center.x;  // X 方向誤差（左負右正）
    error_y = target_center.y - frame_center.y;  // Y 方向誤差（上負下正）
}

// 偵測紅色物體，並計算 X 和 Y 軸誤差
bool detectRedObject(cv::VideoCapture& cap, int& error_x, int& error_y) {
    cv::Mat frame, hsv_frame, mask, resized_frame, edges;
    
    cap >> frame;
    if (frame.empty()) {
        std::cerr << "Error: Unable to grab frame" << std::endl;
        return false;
    }

    // 轉換為 HSV 色彩空間
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

    // 設定紅色範圍
    cv::Mat lower_red, upper_red;
    cv::inRange(hsv_frame, cv::Scalar(0, 190, 150), cv::Scalar(10, 255, 255), lower_red);
    cv::inRange(hsv_frame, cv::Scalar(170, 190, 150), cv::Scalar(180, 255, 255), upper_red);
    
    
    // 合併紅色範圍
    cv::bitwise_or(lower_red, upper_red, mask);

    // 查找輪廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
        double max_area = 0;
        int max_index = -1;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > max_area) {
                max_area = area;
                max_index = i;
            }
        }

        if (max_index >= 0) {
            cv::Rect bounding_box = cv::boundingRect(contours[max_index]);
            cv::rectangle(frame, bounding_box, cv::Scalar(0, 255, 0), 2);

            // 計算紅色區域的中心點
            cv::Point target_center = (bounding_box.tl() + bounding_box.br()) * 0.5;
            cv::circle(frame, target_center, 5, cv::Scalar(0, 0, 255), -1);

            // 計算誤差
            calculateError(target_center, frame.size(), error_x, error_y);

            //cv::resize(frame, resized_frame, cv::Size(), 0.5, 0.5);
            
            //canny 邊緣檢測
            cv::Canny(frame, edges, 50, 150);
            cv::imshow("Red Detection", edges);
            
            return true;
        }
    }
    return false;
}

int main(int argc, char** argv) {

    // PID 控制參數
    double Kp = 0.20;
    double Ki = 0.1;
    double Kd = -0.01;

    double U = 0.0, P = 0.0, I = 0.0, D = 0.0;
    double previousError = 0.0;
    double integralError = 0.0;
    double derivativeError = 0.0;
    double dt = 0.1;  // 預設 dt (初始值)

    // 誤差死區，低於此值視為 0，避免小誤差導致過度控制
    const int ERROR_DEADBAND = 5;

    // 積分項限幅，避免積分累計過大
    const double INTEGRAL_MAX = 100.0;
    const double INTEGRAL_MIN = -100.0;

    // 初始化 ROS 節點
    ros::init(argc, argv, "trashaim");
    ros::NodeHandle nh;

    // 創建一個發布器，發佈到 /mobile_base/commands/velocity 主題
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

    // 設定發布頻率 (10 Hz)
    ros::Rate rate(10);

    // 創建速度消息
    geometry_msgs::Twist move_cmd;

    // 開啟攝像頭
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open camera" << std::endl;
        return -1;
    }

    int error_x = 0, error_y = 0;
    bool paused = false;  // 暫停狀態變數

    // 用於計算 dt 的時間變數
    ros::Time previous_time = ros::Time::now();

    // 主迴圈，使用 ros::ok() 當作退出條件
    while (ros::ok()) {
        // 計算本次迴圈的 dt
        ros::Time current_time = ros::Time::now();
        dt = (current_time - previous_time).toSec();
        previous_time = current_time;

        if (!paused) {
            bool detected = detectRedObject(cap, error_x, error_y);
            if (detected) {
                // 若誤差在死區範圍內則視為 0
                if (std::abs(error_x) < ERROR_DEADBAND) {
                    error_x = 0;
                }

                std::cout << "X Error: " << error_x << ", Y Error: " << error_y << std::endl;

                // PID 控制計算 (這裡以 error_x 為例)
                // 比例項
                P = Kp * error_x;

                // 積分項
                integralError += error_x * dt;

                // 手動限制 integralError 的範圍，防止積分誤差過大
                if (integralError > INTEGRAL_MAX) {
                    integralError = INTEGRAL_MAX;
                } else if (integralError < INTEGRAL_MIN) {
                    integralError = INTEGRAL_MIN;
                }

                I = Ki * integralError;


                // 微分項 (防止 dt 為 0)
                if (dt > 0)
                    derivativeError = (error_x - previousError) / dt;
                else
                    derivativeError = 0.0;
                D = Kd * derivativeError;

                // 控制輸出
                U = P + I + D;

                // 更新前一次的誤差
                previousError = error_x;

                // 可根據需要調整縮放係數，若角速度過快可調低
                move_cmd.angular.z = -0.01 * U;
            } else {
                std::cout << "No target" << std::endl;
                move_cmd.angular.z = 0.0;
                // 當沒有目標時，重置積分項，防止積分風險
                integralError = 0.0;
            }
            // 發布命令
            cmd_vel_pub.publish(move_cmd);
        }

        // 處理 ROS 回調（若有其他訂閱者）
        ros::spinOnce();

        // 控制幀率，避免 CPU 過載
        char key = cv::waitKey(30);
        if (key == 'q') {
            break;  // 按 'q' 退出程式
        } else if (key == 'p') {
            paused = !paused;  // 切換暫停狀態
            if (paused) {
                std::cout << "Detection Paused" << std::endl;
            } else {
                std::cout << "Detection Resumed" << std::endl;
            }
        }

        // 確保迴圈以固定頻率運行
        rate.sleep();
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}





//#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>
//
//int main(int argc, char** argv) {
//    // 初始化 ROS 節點
//    ros::init(argc, argv, "turtlebot2_motion_test");
//    ros::NodeHandle nh;
//
//    // 創建一個發布器，發佈到 /mobile_base/commands/velocity 主題
//    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
//
//    // 設定發布頻率
//    ros::Rate rate(10);
//
//    // 創建速度消息
//    geometry_msgs::Twist move_cmd;
//
//    // 讓機器人前進
//    ROS_INFO("TurtleBot2 is moving forward.");
//    move_cmd.linear.x = 0.2; // 前進速度 (m/s)
//    move_cmd.angular.z = 0.0; // 無旋轉
//
//    for (int i = 0; i < 50; ++i) { // 前進 5 秒鐘 (50 次，每次 0.1 秒)
//        cmd_vel_pub.publish(move_cmd);
//        rate.sleep();
//    }
//
//    // 停止機器人
//    ROS_INFO("TurtleBot2 is stopping.");
//    move_cmd.linear.x = 0.0;
//    cmd_vel_pub.publish(move_cmd);
//    ros::Duration(2.0).sleep(); // 停止 2 秒鐘
//
//    // 讓機器人旋轉
//    ROS_INFO("TurtleBot2 is rotating.");
//    move_cmd.angular.z = 0.5; // 旋轉速度 (rad/s)
//    for (int i = 0; i < 50; ++i) { // 旋轉 5 秒鐘
//        cmd_vel_pub.publish(move_cmd);
//        rate.sleep();
//    }
//
//    // 停止機器人
//    ROS_INFO("TurtleBot2 is stopping.");
//    move_cmd.angular.z = 0.0;
//    cmd_vel_pub.publish(move_cmd);
//
//    ROS_INFO("TurtleBot2 motion test completed.");
//
//    return 0;
//}
