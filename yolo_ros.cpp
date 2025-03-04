#include <ros/ros.h>
#include <csignal>
#include <cstdlib>
#include <thread>
#include <atomic>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>


// �l�� YOLO Python �i�{
std::atomic<pid_t> yolo_pid(0);

// �H���B�z��ơA�T�O�{�����T����귽
void signalHandler(int signum)
{
    ROS_WARN("Interrupt signal (%d) received. Terminating YOLO...", signum);
    if (yolo_pid > 0)
    {
        kill(yolo_pid, SIGTERM); // ���`�פ� Python YOLO �i�{
        waitpid(yolo_pid, nullptr, 0); // ���ݤl�i�{�����A�����L�Ͷi�{
    }
    ros::shutdown(); // ���T���� ROS �`�I
}

void startYolo()
{
    yolo_pid = fork();
    if (yolo_pid == 0) // �l�i�{
    {
        execlp("python3", "python3", "-c",
               "from ultralytics import YOLO; YOLO('yolov8n.pt').predict(source=0, show=True)",
               nullptr);
        _exit(EXIT_FAILURE); // execlp ���ѫh�h�X
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

    // �]�m�H���B�z
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    ROS_INFO("Starting YOLOv8 in Python...");
    
    // �Ұ� YOLO �i�{
    startYolo();

    // ROS �j��
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // ��{�������ɡA�T�O���� YOLO �i�{
    if (yolo_pid > 0)
    {
        ROS_WARN("Shutting down YOLO...");
        kill(yolo_pid, SIGTERM);
        waitpid(yolo_pid, nullptr, 0);
    }

    ROS_INFO("YOLOv8 terminated.");
    return 0;
}
