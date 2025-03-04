#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <sys/select.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "usb_joystick_control");
    ros::NodeHandle nh;

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

    int joystick = open("/dev/input/js0", O_RDONLY);
    if (joystick == -1)
    {
        ROS_ERROR("�L�k���}�n��]�� /dev/input/js0");
        return -1;
    }

    struct js_event js;
    geometry_msgs::Twist cmd_vel;

    const double max_speed = 0.5;   // �̤j���ʳt��
    const double max_turn = 2.0;    // �̤j����t��
    const int deadzone = 3000;      // ���ϭȡ]�ھڴ��սվ�^

    ros::Rate loop_rate(100);  // �]�m�`���W�v�� 100Hz

    while (ros::ok())
    {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(joystick, &readfds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000;  // 10 �@����

        int ret = select(joystick + 1, &readfds, NULL, NULL, &timeout);
        if (ret > 0 && FD_ISSET(joystick, &readfds))
        {
            if (read(joystick, &js, sizeof(struct js_event)) == -1)
            {
                ROS_ERROR("Ū���n���J����");
                break;
            }

            if (js.type == JS_EVENT_AXIS)
            {
                if (js.number == 0) // ���k
                {
                    cmd_vel.linear.y = (abs(js.value) < deadzone) ? 0.0 : js.value / 32767.0 * max_speed;
                }
                else if (js.number == 1) // �e��
                {
                    cmd_vel.linear.x = (abs(js.value) < deadzone) ? 0.0 : -js.value / 32767.0 * max_speed;
                }
                else if (js.number == 3) // ����
                {
                    cmd_vel.angular.z = (abs(js.value) < deadzone) ? 0.0 : js.value / 32767.0 * max_turn;
                }
            }
        }

        cmd_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }

    close(joystick);
    return 0;
}
