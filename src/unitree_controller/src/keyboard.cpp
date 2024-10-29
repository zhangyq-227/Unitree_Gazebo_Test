#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <termios.h>

// int getch()
// {
// 	int ch;
// 	struct termios oldt;
// 	struct termios newt;

// 	// Store old settings, and copy to new settings
// 	tcgetattr(STDIN_FILENO, &oldt);
// 	newt = oldt;

// 	// Make required changes and apply the settings
// 	newt.c_lflag &= ~(ICANON | ECHO);
// 	newt.c_iflag |= IGNBRK;
// 	newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
// 	newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
// 	newt.c_cc[VMIN] = 0;
// 	newt.c_cc[VTIME] = 1;
// 	tcsetattr(fileno(stdin), TCSANOW, &newt);

// 	// Get the current character
// 	ch = getchar();

// 	// Reapply old settings
// 	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

// 	return ch;
// }


int getch()
{
    struct termios oldt, newt;
    int ch;

    // 获取当前终端属性
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // 禁用规范模式和回显
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // 读取一个字符（阻塞模式）
    ch = getchar();

    // 恢复原来的终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "keyboard_input_node");

	ros::NodeHandle nh;

	ros::Rate loop_rate(100);

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	geometry_msgs::Twist twist;

	long count = 0;

	while (ros::ok())
	{
		twist.linear.x = 0.0;
		twist.linear.y = 0.0;
		twist.linear.z = 0.0;
		twist.angular.x = 0.0;
		twist.angular.y = 0.0;
		twist.angular.z = 0.0;

		int ch = 0;

		ch = getch();

		switch (ch)
		{
		case 'q':
			printf("already quit!\n");
			return 0;

		case 'w':
			twist.linear.x = 0.8;
			printf("move forward!\n");
			break;

		case 's':
			twist.linear.x = -0.6;
			printf("move backward!\n");
			break;

		case 'z':
			twist.linear.y = -0.5;
			printf("move left!\n");
			break;
		case 'x':
			twist.linear.y = 0.5;
			printf("move right!\n");
			break;
		case 'a':
            twist.linear.x = 0.6;
			twist.angular.z = 1.0;
			printf("turn left!\n");
			break;

		case 'd':
            twist.linear.x = 0.6;
			twist.angular.z = -1.0;
			printf("turn right!\n");
			break;

		default:
			printf("Stop!\n");
			break;
		}

		pub.publish(twist);

		// ros::spinOnce();
		// loop_rate.sleep();
	}

	return 0;
}