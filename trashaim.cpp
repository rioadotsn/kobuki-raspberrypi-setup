#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// �p�� X �M Y �b���~�t
void calculateError(const cv::Point& target_center, const cv::Size& frame_size, int& error_x, int& error_y) {
    cv::Point frame_center(frame_size.width / 2, frame_size.height / 2);
    error_x = target_center.x - frame_center.x;  // X ��V�~�t�]���t�k���^
    error_y = target_center.y - frame_center.y;  // Y ��V�~�t�]�W�t�U���^
}

// �������⪫��A�íp�� X �M Y �b�~�t
bool detectRedObject(cv::VideoCapture& cap, int& error_x, int& error_y) {
    cv::Mat frame, hsv_frame, mask, resized_frame, edges;
    
    cap >> frame;
    if (frame.empty()) {
        std::cerr << "Error: Unable to grab frame" << std::endl;
        return false;
    }

    // �ഫ�� HSV ��m�Ŷ�
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

    // �]�w����d��
    cv::Mat lower_red, upper_red;
    cv::inRange(hsv_frame, cv::Scalar(0, 190, 150), cv::Scalar(10, 255, 255), lower_red);
    cv::inRange(hsv_frame, cv::Scalar(170, 190, 150), cv::Scalar(180, 255, 255), upper_red);
    
    
    // �X�֬���d��
    cv::bitwise_or(lower_red, upper_red, mask);

    // �d�����
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

            // �p�����ϰ쪺�����I
            cv::Point target_center = (bounding_box.tl() + bounding_box.br()) * 0.5;
            cv::circle(frame, target_center, 5, cv::Scalar(0, 0, 255), -1);

            // �p��~�t
            calculateError(target_center, frame.size(), error_x, error_y);

            //cv::resize(frame, resized_frame, cv::Size(), 0.5, 0.5);
            
            //canny ��t�˴�
            cv::Canny(frame, edges, 50, 150);
            cv::imshow("Red Detection", edges);
            
            return true;
        }
    }
    return false;
}

int main(int argc, char** argv) {

    // PID ����Ѽ�
    double Kp = 0.20;
    double Ki = 0.1;
    double Kd = -0.01;

    double U = 0.0, P = 0.0, I = 0.0, D = 0.0;
    double previousError = 0.0;
    double integralError = 0.0;
    double derivativeError = 0.0;
    double dt = 0.1;  // �w�] dt (��l��)

    // �~�t���ϡA�C�󦹭ȵ��� 0�A�קK�p�~�t�ɭP�L�ױ���
    const int ERROR_DEADBAND = 5;

    // �n�������T�A�קK�n���֭p�L�j
    const double INTEGRAL_MAX = 100.0;
    const double INTEGRAL_MIN = -100.0;

    // ��l�� ROS �`�I
    ros::init(argc, argv, "trashaim");
    ros::NodeHandle nh;

    // �Ыؤ@�ӵo�����A�o�G�� /mobile_base/commands/velocity �D�D
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

    // �]�w�o���W�v (10 Hz)
    ros::Rate rate(10);

    // �Ыسt�׮���
    geometry_msgs::Twist move_cmd;

    // �}���ṳ�Y
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open camera" << std::endl;
        return -1;
    }

    int error_x = 0, error_y = 0;
    bool paused = false;  // �Ȱ����A�ܼ�

    // �Ω�p�� dt ���ɶ��ܼ�
    ros::Time previous_time = ros::Time::now();

    // �D�j��A�ϥ� ros::ok() ��@�h�X����
    while (ros::ok()) {
        // �p�⥻���j�骺 dt
        ros::Time current_time = ros::Time::now();
        dt = (current_time - previous_time).toSec();
        previous_time = current_time;

        if (!paused) {
            bool detected = detectRedObject(cap, error_x, error_y);
            if (detected) {
                // �Y�~�t�b���Ͻd�򤺫h���� 0
                if (std::abs(error_x) < ERROR_DEADBAND) {
                    error_x = 0;
                }

                std::cout << "X Error: " << error_x << ", Y Error: " << error_y << std::endl;

                // PID ����p�� (�o�̥H error_x ����)
                // ��Ҷ�
                P = Kp * error_x;

                // �n����
                integralError += error_x * dt;

                // ��ʭ��� integralError ���d��A����n���~�t�L�j
                if (integralError > INTEGRAL_MAX) {
                    integralError = INTEGRAL_MAX;
                } else if (integralError < INTEGRAL_MIN) {
                    integralError = INTEGRAL_MIN;
                }

                I = Ki * integralError;


                // �L���� (���� dt �� 0)
                if (dt > 0)
                    derivativeError = (error_x - previousError) / dt;
                else
                    derivativeError = 0.0;
                D = Kd * derivativeError;

                // �����X
                U = P + I + D;

                // ��s�e�@�����~�t
                previousError = error_x;

                // �i�ھڻݭn�վ��Y��Y�ơA�Y���t�׹L�֥i�էC
                move_cmd.angular.z = -0.01 * U;
            } else {
                std::cout << "No target" << std::endl;
                move_cmd.angular.z = 0.0;
                // ��S���ؼЮɡA���m�n�����A����n�����I
                integralError = 0.0;
            }
            // �o���R�O
            cmd_vel_pub.publish(move_cmd);
        }

        // �B�z ROS �^�ա]�Y����L�q�\�̡^
        ros::spinOnce();

        // ����V�v�A�קK CPU �L��
        char key = cv::waitKey(30);
        if (key == 'q') {
            break;  // �� 'q' �h�X�{��
        } else if (key == 'p') {
            paused = !paused;  // �����Ȱ����A
            if (paused) {
                std::cout << "Detection Paused" << std::endl;
            } else {
                std::cout << "Detection Resumed" << std::endl;
            }
        }

        // �T�O�j��H�T�w�W�v�B��
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
//    // ��l�� ROS �`�I
//    ros::init(argc, argv, "turtlebot2_motion_test");
//    ros::NodeHandle nh;
//
//    // �Ыؤ@�ӵo�����A�o�G�� /mobile_base/commands/velocity �D�D
//    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
//
//    // �]�w�o���W�v
//    ros::Rate rate(10);
//
//    // �Ыسt�׮���
//    geometry_msgs::Twist move_cmd;
//
//    // �������H�e�i
//    ROS_INFO("TurtleBot2 is moving forward.");
//    move_cmd.linear.x = 0.2; // �e�i�t�� (m/s)
//    move_cmd.angular.z = 0.0; // �L����
//
//    for (int i = 0; i < 50; ++i) { // �e�i 5 ���� (50 ���A�C�� 0.1 ��)
//        cmd_vel_pub.publish(move_cmd);
//        rate.sleep();
//    }
//
//    // ��������H
//    ROS_INFO("TurtleBot2 is stopping.");
//    move_cmd.linear.x = 0.0;
//    cmd_vel_pub.publish(move_cmd);
//    ros::Duration(2.0).sleep(); // ���� 2 ����
//
//    // �������H����
//    ROS_INFO("TurtleBot2 is rotating.");
//    move_cmd.angular.z = 0.5; // ����t�� (rad/s)
//    for (int i = 0; i < 50; ++i) { // ���� 5 ����
//        cmd_vel_pub.publish(move_cmd);
//        rate.sleep();
//    }
//
//    // ��������H
//    ROS_INFO("TurtleBot2 is stopping.");
//    move_cmd.angular.z = 0.0;
//    cmd_vel_pub.publish(move_cmd);
//
//    ROS_INFO("TurtleBot2 motion test completed.");
//
//    return 0;
//}
