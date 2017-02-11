#include "kinco_motor_rs232.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PointStamped.h"

using namespace std;

bool init()
{
    /*Open USB serial port*/
    frame = open("/dev/ttyUSB0", O_RDWR);
    if (frame < 0)
    {
       std::cout<< "Error " << "opening " << "/dev/ttyUSB0" << std::endl;
       exit(0);
       return false;
    }
    else
    {
       std::cout << "Open device succeeded" << std::endl;
    }
    /*configure port and set baud rate*/
    struct termios options;
    tcgetattr(frame, &options);
    options.c_cflag = CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    cfsetispeed(&options, B38400);
    cfsetospeed(&options, B38400);
    tcflush(frame,TCIFLUSH);
    tcsetattr(frame, TCSANOW, &options);
    return true;
}

void read_callback(uint8_t r_cmd[])
{
    // read data from motor
    printf("Data is: ");
    for(int i = 0; i < 10; i++)
    {
        printf("%02X ",r_cmd[i]);
    }
    printf("\n");
}

void send_cmd(int frame_id, uint8_t cmd[])
{
    // send out write request
    uint8_t cmd_internal[10];
    uint8_t cmd_received[10];
    for(int i = 0; i < 9; i++)
    {
        cmd_internal[i] = cmd[i];
    }
    cmd_internal[9] = (0xFFFF + (-(cmd[0] + cmd[1] + cmd[2] + cmd[3] + cmd[4] + cmd[5] + cmd[6] + cmd[7] + cmd[8]) + 1))& 0xFF;
    ssize_t write_size        = write(frame_id, &cmd_internal, sizeof(cmd_internal));
    usleep(100000);
    ssize_t cmd_received_size = read(frame_id, &cmd_received, sizeof(cmd_received));
//    read_callback(cmd_received);
}

void set_speed(int frame_id, int vel)
{
    // set the speed of the motor, dec = rpm * 512 * 60000/1875
    vel = vel*512*60000/1875;
    uint8_t speed[10];
    uint8_t speed_r[10];
    speed[0] = 0x01;
    speed[1] = 0x23;
    speed[2] = 0xFF;
    speed[3] = 0x60;
    speed[4] = 0x00;
    if(vel >= 0)
    {
        speed[5] = (vel & 0x00FF);
        speed[6] = (vel & 0xFF00) >> 8;
        speed[7] = (vel & 0xFF0000) >> 16;
        speed[8] = (vel & 0xFF000000) >> 24;
    }
    else
    {
        vel = 0xFFFF + (vel + 1);
        speed[5] = (vel & 0x00FF);
        speed[6] = (vel & 0xFF00) >> 8;
        speed[7] = (vel & 0xFF0000) >> 16;
        speed[8] = (vel & 0xFF000000) >> 24;      
    }
    speed[9] = (0xFFFF + (-(speed[0] + speed[1] + speed[2] + speed[3] + speed[4] + speed[5] + speed[6] + speed[7] + speed[8]) + 1))& 0xFF;
    ssize_t speed_size = write(frame_id, &speed, sizeof(speed));
    usleep(10000);
    ssize_t speed_receive_size = read(frame_id, &speed_r, sizeof(speed_r));
//    read_callback(speed_r);
}

void read_data(int frame_id, uint8_t index_data[])
{
    // send out read data request, read the object with given index
    uint8_t request_read[10];
    uint8_t cmd_received[10];

    request_read[0] = 0x01;
    request_read[1] = 0x40;
    request_read[2] = index_data[1];
    request_read[3] = index_data[0];
    request_read[4] = index_data[2];
    request_read[5] = 0x00;
    request_read[6] = 0x00;
    request_read[7] = 0x00;
    request_read[8] = 0x00;
    request_read[9] = (0xFFFF + (-(request_read[0] + request_read[1] + request_read[2] + request_read[3] + request_read[4] + request_read[5] + request_read[6] + request_read[7] + request_read[8]) + 1))& 0xFF;
    ssize_t read_request = write(frame_id, &request_read, sizeof(request_read));
    usleep(10000);
    ssize_t received_size = read(frame_id, &cmd_received, sizeof(cmd_received));
    read_callback(cmd_received);
}

int get_vel(int frame_id)
{
   // get the speed of the motor
   uint8_t speed_read[10];
   uint8_t speed_read_received[10];
   speed_read[0] = 0x01;
   speed_read[1] = 0x40;
   speed_read[2] = 0x6C;
   speed_read[3] = 0x60;
   speed_read[4] = 0x00;
   speed_read[5] = 0x00;
   speed_read[6] = 0x00;
   speed_read[7] = 0x00;
   speed_read[8] = 0x00;
   speed_read[9] = (0xFFFF + (-(speed_read[0] + speed_read[1] + speed_read[2] + speed_read[3] + speed_read[4] + speed_read[5] + speed_read[6] + speed_read[7] + speed_read[8]) + 1))& 0xFF;

   // send out read speed request and read the motor's callback to get vel
   ssize_t write_read_speed = write(frame_id, &speed_read, sizeof(speed_read));
   usleep(10000);
   ssize_t received_read_speed = read(frame_id, &speed_read_received, sizeof(speed_read_received));
   int Vel;

   if (speed_read_received[8] == 0x00)
   {
       Vel = speed_read_received[5] + speed_read_received[6]*256;
   } 
   else if(speed_read_received[8] == 0xFF)
   {   
       Vel = (speed_read_received[5]) | (speed_read_received[6] << 8);
       Vel = Vel - 0xFFFF - 1;
   }
   else
   {
       Vel =0;
   }
   return Vel;
}


int main(int argc, char** argv){

    init();
    ros::init(argc, argv, "kinco_motor_rs232");
    ros::NodeHandle nh;

    ros::Publisher m_velPub = nh.advertise<geometry_msgs::PointStamped>("vel", 1);
    std::cout << "----------------------Program Start----------------------" << std::endl;

    uint8_t receive_data[10];
    // speed mode set
    uint8_t cmd1[9]       = {0x01, 0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};
    // set control bit to 2F: means set on the controller
    uint8_t cmd2[9]       = {0x01, 0x2B, 0x40, 0x60, 0x00, 0x2F, 0x00, 0x00, 0x00};
    // set control bit to 06: means set off the controller
    uint8_t cmd3[9]       = {0x01, 0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
    // set acceleration 
    uint8_t cmd_acc[9]    = {0x01, 0x23, 0x83, 0x60, 0x00, 0x00, 0x10, 0x00, 0x00};
    // set deacceleration
    uint8_t cmd_deacc[9]  = {0x01, 0x23, 0x84, 0x60, 0x00, 0x00, 0x10, 0x00, 0x00};

    ssize_t receiver_size;

    // initialise motor set speed mode and set on controller
    send_cmd(frame, cmd1);
    usleep(100000);
    send_cmd(frame, cmd_acc);
    usleep(100000);
    send_cmd(frame, cmd_deacc);
    usleep(100000);
    send_cmd(frame, cmd2);
    usleep(100000);

    // set vel to be zero
    set_speed(frame, 100);
    usleep(100000);

    int count = 1000;
    double period;
    ros::Time start = ros::Time::now();
    geometry_msgs::PointStamped vel;

    while(count--)
    {
        // set vel to be 150
        if (count == 500)
        {
            set_speed(frame, 150);
        }
        // set vel to be 50
        if (count == 250)
        {
            set_speed(frame, 50);
        }
        vel.point.x = float(get_vel(frame));
        vel.point.y = 0.0;
        vel.point.y = 0.0;

        vel.header.stamp = ros::Time::now();
        m_velPub.publish(vel);
    }

    period = (ros::Time::now() - start).toSec();

    printf("Time usage is %f\n", period);
    // set speed to be zero
    set_speed(frame, 0);
    sleep(1);
    // set off controller
    send_cmd(frame, cmd3);
    sleep(1);
    close(frame);
}

//I am here to test git workflow, so I change this file in laptop local repo
//I change the cpp file in devel branch
