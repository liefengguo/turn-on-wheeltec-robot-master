#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_LEFT 0x44
#define KEYCODE_RIGHT 0x43

int kfd = 0;
struct termios cooked, raw;

void init_keyboard() {
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
}

void restore_keyboard() {
    tcsetattr(kfd, TCSANOW, &cooked);
}

int read_key() {
    unsigned char c;
    if(read(kfd, &c, 1) < 0) {
        return -1;
    }
    return c;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_controller");
    ros::NodeHandle n;
    ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    geometry_msgs::Twist twist_msg;

    printf("Use arrow keys to move the robot. 'q' to exit.\n");

    init_keyboard();
    bool new_input = false;
    while(ros::ok()) {
        // Check for new keyboard input
        int c = read_key();
        if(c != -1) {
            new_input = true;
        }
        
        // Set twist message values based on keyboard input or default values
        if(new_input) {
            switch(c) {
                case KEYCODE_UP:
                    twist_msg.linear.x = 0.2;
                    break;
                case KEYCODE_DOWN:
                    twist_msg.linear.x = -0.2;
                    break;
                case KEYCODE_LEFT:
                    twist_msg.angular.z = 0.5;
                    break;
                case KEYCODE_RIGHT:
                    twist_msg.angular.z = -0.5;
                    break;
                case 'q':
                    restore_keyboard();
                    return 0;
            }
            new_input = false;
        } else {
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
        }

        twist_pub.publish(twist_msg);
    }
    restore_keyboard();
    return 0;
}