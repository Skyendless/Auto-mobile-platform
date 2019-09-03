/*
 * Author: huws
 */

#include "minibot_driver.h"


void minibot_driver(ros::NodeHandle nh, ros::NodeHandle private_nh);

serial::Serial my_serial("/dev/minibot", 115200, serial::Timeout::simpleTimeout(100),
                        serial::eightbits, serial::parity_none, serial::stopbits_one, serial::flowcontrol_none);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "minibot_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    minibot_driver(nh, private_nh);

    ros::spin();

    return 0;
}


void minibot_driver(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
    //try to connect the port
    if(my_serial.isOpen())
    {
        std::cout << " succes to open the port.\n" << std::endl;
    }
    else{
        std::cout << " failed to open the port.\n" << std::endl;
        ros::shutdown();
    }


    //订阅节点
    base_control_sub = nh.subscribe("commands/base_control", 10, &baseControlCallback);
    miniarm_sub = nh.subscribe("MotorControl", 10, &miniarmCallback);

    //发布节点
    //status_pub = nh.advertise<minibot_msgs::Status>("status", 0);
    //IMUSensors_pub = nh.advertise<minibot_msgs::IMU>("sensors/IMU", 10);
    //battery_pub = nh.advertise<minibot_msgs::Battery>("sensors/battery", 0);
    //firmware_pub = nh.advertise<minibot_msgs::Firmware>("firmware_info", 10);

    // create a 50Hz timer, used for state machine & polling VESC telemetry
    timer_ = nh.createTimer(ros::Duration(1.0/50.0), &timerCallback);

}


//read buffer  50HZ
void timerCallback(const ros::TimerEvent& event)
{
    
}




//callback function

/*** 
uint8     subpayload header      09 : syncwrite
uint8     length                 25
uint8     type                   0x01 : xl
uint16    adress                 104 : goal velocity
uint16    param_length           20
uint8     id_1                   
int32     param_1
uint8     id_2
int32     param_2
uint8     id_3
int32     param_3
uint8     id_4
int32     param_4
****/

void baseControlCallback(const minibot_msgs::BaseControlConstPtr &base_control)
{
    int32_t wheel_1, wheel_2, wheel_3, wheel_4;
    uint8_t data[27];
    uint8_t *data_p = data;
    Buffer packet;

    uint8_t  subpayload_header = 9;
    uint8_t  servo_type = 1;
    uint8_t  packet_length = 25;
    uint16_t param_address = 104;
    uint16_t param_length = 20;

    uint8_t id_1 = 1;
    uint8_t id_2 = 2;
    uint8_t id_3 = 3;
    uint8_t id_4 = 4;

    wheel_1 = (int32_t)base_control->wheel_speed_one;
    wheel_2 = (int32_t)base_control->wheel_speed_two;
    wheel_3 = (int32_t)base_control->wheel_speed_three;
    wheel_4 = (int32_t)base_control->wheel_speed_four;

    *(data_p + 0) = subpayload_header;
    *(data_p + 1) = packet_length ;
    *(data_p + 2) = servo_type ; 

    *(data_p + 3) = static_cast<uint8_t>((static_cast<uint16_t>(param_address) >> 8 ) & 0xFF)  ;
    *(data_p + 4) = static_cast<uint8_t>(static_cast<uint16_t>(param_address) & 0xFF)  ;

    *(data_p + 5) = static_cast<uint8_t>((static_cast<uint16_t>(param_length) >> 8 ) & 0xFF)  ;
    *(data_p + 6) = static_cast<uint8_t>(static_cast<uint16_t>(param_length) & 0xFF)  ;

    *(data_p + 7) = id_1;
    *(data_p + 8) = static_cast<uint8_t>((static_cast<int32_t>(wheel_1) >> 24) & 0xFF);
    *(data_p + 9) = static_cast<uint8_t>((static_cast<int32_t>(wheel_1) >> 16) & 0xFF);
    *(data_p + 10) = static_cast<uint8_t>((static_cast<int32_t>(wheel_1) >> 8) & 0xFF);
    *(data_p + 11) = static_cast<uint8_t>(static_cast<int32_t>(wheel_1) & 0xFF);

    *(data_p + 12) = id_2;
    *(data_p + 13) = static_cast<uint8_t>((static_cast<int32_t>(wheel_2) >> 24) & 0xFF);
    *(data_p + 14) = static_cast<uint8_t>((static_cast<int32_t>(wheel_2) >> 16) & 0xFF);
    *(data_p + 15) = static_cast<uint8_t>((static_cast<int32_t>(wheel_2) >> 8) & 0xFF);
    *(data_p + 16) = static_cast<uint8_t>(static_cast<int32_t>(wheel_2) & 0xFF);

    *(data_p + 17) = id_3;
    *(data_p + 18) = static_cast<uint8_t>((static_cast<int32_t>(wheel_3) >> 24) & 0xFF);
    *(data_p + 19) = static_cast<uint8_t>((static_cast<int32_t>(wheel_3) >> 16) & 0xFF);
    *(data_p + 20) = static_cast<uint8_t>((static_cast<int32_t>(wheel_3) >> 8) & 0xFF);
    *(data_p + 21) = static_cast<uint8_t>(static_cast<int32_t>(wheel_3) & 0xFF);

    *(data_p + 22) = id_4;
    *(data_p + 23) = static_cast<uint8_t>((static_cast<int32_t>(wheel_4) >> 24) & 0xFF);
    *(data_p + 24) = static_cast<uint8_t>((static_cast<int32_t>(wheel_4) >> 16) & 0xFF);
    *(data_p + 25) = static_cast<uint8_t>((static_cast<int32_t>(wheel_4) >> 8) & 0xFF);
    *(data_p + 26) = static_cast<uint8_t>(static_cast<int32_t>(wheel_4) & 0xFF);

    packet.push_back(0xaa);
    packet.push_back(0x55);
    packet.push_back(sizeof(data));
    for(int i = 0; i < 27 ; i++)
    {
        packet.push_back(data[i]);
    }
    CRC crc_calc;
    crc_calc.process_bytes(&(*data), boost::distance(data));
    uint16_t crc = crc_calc.checksum();
    packet.push_back( static_cast<uint8_t>(crc >> 8) );
    packet.push_back( static_cast<uint8_t>(crc & 0xFF) );

    size_t bytes_wrote = my_serial.write(packet);

    for(auto iter = packet.begin(); iter != packet.end(); iter++)
    {
        printf("%x \t", *iter);
    }
    printf("\n");

    if(bytes_wrote != packet.size())
    {
        std::stringstream ss;
        ss << "wrote " << bytes_wrote << " bytes,but expected " << packet.size() << " bytes.";
        //throw SerialException(ss.str().c_str());
    }
}

void miniarmCallback(const minibot_msgs::MotorOperationConstPtr &mini_arm)
{
    uint8_t cmd, id, isImodule ;
    int32_t servo_param;

    Buffer packet;

    uint8_t  subpayload_header;
    uint8_t  servo_type;

    uint8_t  ax_packet_length;
    uint8_t  xl_packet_length;

    uint8_t  ax_param_address;
    uint16_t xl_param_address;


    cmd = mini_arm->cmd;
    id = mini_arm->id;
    isImodule = mini_arm->isImodule;
    servo_param = mini_arm->parameters;

    if (cmd == 1) //set angle
    {
        *(data_p + 0) = 3; //write
        xl_param_address = XL_GOAL_POSITION;
        ax_param_address = AX_GOAL_POSITION;

        ax_packet_length = 5;
        xl_packet_length = 8;

    }else{
        
    }
    
    if (isImodele)
    {
        *(data_p + 1) = xl_packet_length ;
    }else{
        *(data_p + 1) = ax_packet_length ;
    }
    
    *(data_p + 2) = !isImodule ;      //servo type
    *(data_p + 3) = id ;             //servo ID

    if(isImodule){
        //address
        *(data_p + 4) = static_cast<uint8_t>((static_cast<int16_t>(xl_param_address) >> 8) & 0xFF);
        *(data_p + 5) = static_cast<uint8_t>(static_cast<int16_t>(xl_param_address) & 0xFF);
        //parameters
        *(data_p + 6) = static_cast<uint8_t>((static_cast<int32_t>(servo_param) >> 24) & 0xFF);
        *(data_p + 7) = static_cast<uint8_t>((static_cast<int32_t>(servo_param) >> 16) & 0xFF);
        *(data_p + 8) = static_cast<uint8_t>((static_cast<int32_t>(servo_param) >> 8) & 0xFF);
        *(data_p + 9) = static_cast<uint8_t>(static_cast<int32_t>(servo_param) & 0xFF);
    }else{
        *(data_p + 4) = ax_param_address ;

        *(data_p + 5) = static_cast<uint8_t>((static_cast<int16_t>(servo_param) >> 8) & 0xFF);
        *(data_p + 6) = static_cast<uint8_t>(static_cast<int16_t>(servo_param) & 0xFF);
    }


    packet.push_back(0xaa);
    packet.push_back(0x55);
    packet.push_back(sizeof(data));  //total length
    for(int i = 0; i < sizeof(data) ; i++)
    {
        packet.push_back(data[i]);
    }
    CRC crc_calc;
    crc_calc.process_bytes(&(*data), boost::distance(data));
    uint16_t crc = crc_calc.checksum();
    packet.push_back( static_cast<uint8_t>(crc >> 8) );
    packet.push_back( static_cast<uint8_t>(crc & 0xFF) );

    size_t bytes_wrote = my_serial.write(packet);

    for(auto iter = packet.begin(); iter != packet.end(); iter++)
    {
        printf("%x \t", *iter);
    }
    printf("\n");

    if(bytes_wrote != packet.size())
    {
        std::stringstream ss;
        ss << "wrote " << bytes_wrote << " bytes,but expected " << packet.size() << " bytes.";
        //throw SerialException(ss.str().c_str());
    }
}











