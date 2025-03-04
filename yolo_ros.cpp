#include <ros/ros.h>
#include <csignal>
#include <cstdlib>
#include <thread>
#include <atomic>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>


// 追蹤 YOLO Python 進程
std::atomic<pid_t> yolo_pid(0);

// 信號處理函數，確保程式正確釋放資源
void signalHandler(int signum)
{
    ROS_WARN("Interrupt signal (%d) received. Terminating YOLO...", signum);
    if (yolo_pid > 0)
    {
        kill(yolo_pid, SIGTERM); // 正常終止 Python YOLO 進程
        waitpid(yolo_pid, nullptr, 0); // 等待子進程結束，防止殭屍進程
    }
    ros::shutdown(); // 正確關閉 ROS 節點
}

void startYolo()
{
    yolo_pid = fork();
    if (yolo_pid == 0) // 子進程
    {
        execlp("python3", "python3", "-c",
               "from ultralytics import YOLO; YOLO('yolov8n.pt').predict(source=0, show=True)",
               nullptr);
        _exit(EXIT_FAILURE); // execlp 失敗則退出
    }
    else if (yolo_pid < 0)
    {
        ROS_ERROR("Failed to fork YOLO process!");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yolo_ros");
    ros::NodeHandle nh;

    // 設置信號處理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    ROS_INFO("Starting YOLOv8 in Python...");
    
    // 啟動 YOLO 進程
    startYolo();

    // ROS 迴圈
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // 當程式結束時，確保關閉 YOLO 進程
    if (yolo_pid > 0)
    {
        ROS_WARN("Shutting down YOLO...");
        kill(yolo_pid, SIGTERM);
        waitpid(yolo_pid, nullptr, 0);
    }

    ROS_INFO("YOLOv8 terminated.");
    return 0;
}
