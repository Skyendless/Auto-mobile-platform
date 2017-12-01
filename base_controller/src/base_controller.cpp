/******************************************************************
基于串口通信的ROS小车基础控制器，功能如下：
1.实现ros控制数据通过固定的格式和串口通信，从而达到控制小车的移动
2.订阅了/cmd_vel主题，只要向该主题发布消息，就能实现对控制小车的移动
3.发布里程计主题/odm

串口通信说明：
1.写入串口
（1）内容：左右轮速度，单位为mm/s
（2）格式：１０字节,[右轮速度４字节][左轮速度４字节][结束符"\r\n"２字节]
2.读取串口
（1）内容：小车x,y坐标，方向角，线速度，角速度，单位依次为：mm,mm,rad,mm/s,rad/s
（2）格式：２１字节，[Ｘ坐标４字节][Ｙ坐标４字节][方向角４字节][线速度４字节][角速度４字节][结束符"\n"１字节]
*******************************************************************/
#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
//以下为串口通讯需要的头文件
#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial/serial.h"
/****************************************************************************/
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
/*****************************************************************************/
#define PI 3.14159
float ratio = 1000.0f ;   //转速转换比例，执行速度调整比例
float D = 0.576f ;    //两轮间距，单位是m
float encoder_ppr = 1024;
float wheel_radius = 0.125;
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度
/****************************************************/
unsigned char data_terminal0=0x0d;  //“/r"字符
unsigned char data_terminal1=0x0a;  //“/n"字符
unsigned char speed_data[10]={0};   //要发给串口的数据
string rec_buffer;  //串口数据接收变量

//发送给下位机的左右轮速度，里程计的坐标和方向
union floatData //union的作用为实现char数组和float之间的转换
{
    float d;
    unsigned char data[4];
}right_speed_data,left_speed_data,position_x,position_y,oriention,vel_linear,vel_angular;
/************************************************************/
void callback(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
    string port("/dev/ttyUSB0");    //小车串口号
    unsigned long baud = 115200;    //小车串口波特率
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000)); //配置串口

    angular_temp = cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
    linear_temp  = cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s

    //将转换好的小车速度分量为左右轮速度
    left_speed_data.d  = linear_temp - 0.5f*angular_temp*D ;
    right_speed_data.d = linear_temp + 0.5f*angular_temp*D ;

    //存入数据到要发布的左右轮速度消息
    left_speed_data.d*=ratio;   //放大１０００倍，将单位从 m/s变为 mm/s
    right_speed_data.d*=ratio;//放大１０００倍，mm/s

    for(int i=0;i<4;i++)    //将左右轮速度存入数组中发送给串口
    {
        speed_data[i]=right_speed_data.data[i];
        speed_data[i+4]=left_speed_data.data[i];
    }

    //在写入串口的左右轮速度数据后加入”/r/n“
    speed_data[8]=data_terminal0;
    speed_data[9]=data_terminal1;
    //写入数据到串口
    my_serial.write(speed_data,10);
}

int main(int argc, char **argv)
{
    string port("/dev/ttyUSB0");//小车串口号
    unsigned long baud = 115200;//小车串口波特率
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));//配置串口

    ros::init(argc, argv, "base_controller");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄

    ros::Subscriber sub = n.subscribe("cmd_vel", 50, callback); //订阅/cmd_vel主题
    ros::Publisher odom_pub= n.advertise<nav_msgs::Odometry>("odom", 20); //定义要发布/odom主题

 //   geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息
    nav_msgs::Odometry CarOdom;//定义里程计对象
    geometry_msgs::Quaternion odom_quat; //四元数变量

    double var_len,var_angle;
    var_len=(50.0f/encoder_ppr*2.0f*PI*wheel_radius)*(50.0f/encoder_ppr*2.0f*PI*wheel_radius);
    var_angle=(0.01f/180.0f*PI)*(0.01f/180.0f*PI);

    //定义covariance矩阵，作用为解决Location和velocity的不同测量的不确定性
    float covariance[36] = {var_len,  0,     0,     0,     0,     0,  // covariance on gps_x
                            0,      var_len, 0,     0,     0,     0,  // covariance on gps_y
                            0,        0,    999,    0,     0,     0,  // covariance on gps_z
                            0,        0,     0,     999,   0,     0,  // large covariance on rot x
                            0,        0,     0,     0,     999,   0,  // large covariance on rot y
                            0,        0,     0,     0,     0,     var_angle};  // large covariance on rot z 

    ros::Rate loop_rate(50);//设置周期休眠时间
    while(ros::ok())
    {
        rec_buffer =my_serial.readline(25,"\n");    //获取串口发送来的数据
        const char *receive_data=rec_buffer.data(); //保存串口发送来的数据
        if(rec_buffer.length()==21) //串口接收的数据长度正确就处理并发布里程计数据消息
        {
            for(int i=0;i<4;i++)//提取X，Y坐标，方向，线速度，角速度
            {
                position_x.data[i]=receive_data[i];
                position_y.data[i]=receive_data[i+4];
                oriention.data[i]=receive_data[i+8];
                vel_linear.data[i]=receive_data[i+12];
                vel_angular.data[i]=receive_data[i+16];
            }
            //将X，Y坐标，线速度缩小1000倍
            position_x.d/=1000; //m
            position_y.d/=1000; //m
            vel_linear.d/=1000; //m/s

            //里程计的偏航角需要转换成四元数才能发布
            odom_quat = tf::createQuaternionMsgFromYaw(oriention.d/180.0f*PI);//将偏航角转换成四元数

            //载入坐标（tf）变换时间戳
            CarOdom.header.stamp = ros::Time::now();
            //发布坐标变换的父坐标系
            CarOdom.header.frame_id = "odom";           
            CarOdom.pose.pose.position.x = position_x.d;     
            CarOdom.pose.pose.position.y = position_y.d;
            CarOdom.pose.pose.position.z = 0.0f;
            CarOdom.pose.pose.orientation = odom_quat;   
            //载入covariance矩阵
            for(int i = 0; i < 36; i++)
            {
             CarOdom.pose.covariance[i] = covariance[i];
            }   

            //里程计的子坐标系
            CarOdom.child_frame_id = "base_footprint";       
            //载入线速度和角速度
            CarOdom.twist.twist.linear.x = vel_linear.d*cos(oriention.d* PI / 180.0f);
            CarOdom.twist.twist.linear.y = vel_linear.d*sin(oriention.d* PI / 180.0f);
            CarOdom.twist.twist.angular.z = vel_angular.d;    
            for(int i = 0; i < 36; i++)
            {
             CarOdom.twist.covariance[i] = covariance[i];
            }  
            //发布里程计
            odom_pub.publish(CarOdom);


            // pub transform


            static tf::TransformBroadcaster br;//定义tf对象
            tf::Quaternion q;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(position_x.d, position_y.d, 0.0));
            q.setRPY(0, 0, oriention.d/180*PI);
            transform.setRotation(q);
            //发布tf坐标变化
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));

            ros::spinOnce();//周期执行
            loop_rate.sleep();//周期休眠
        }
        //程序周期性调用
        //ros::spinOnce();  //callback函数必须处理所有问题时，才可以用到
    }
    return 0;
}

