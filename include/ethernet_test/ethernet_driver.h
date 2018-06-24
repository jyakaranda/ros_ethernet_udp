#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>
#include <cmath>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <ethernet_test/user_protocol.h>

namespace ethernet_driver{
    class EthernetDriver{
        public:
            EthernetDriver(ros::NodeHandle& n, ros::NodeHandle& np);
            ~EthernetDriver();
            bool initialize();
            bool polling();

            typedef boost::shared_ptr<EthernetDriver> EthernetDriverPtr;
            typedef boost::shared_ptr<const EthernetDriver> EthernetDriverConstPtr;

        private:
            bool DEBUG;

            ros::NodeHandle n_;
            ros::NodeHandle np_;
            bool loadParams();
            bool createRosIO();
            bool openUDPPort();
            int getPacket(PacketData& packet, double& stamp);
            bool pubMsg(PacketData packet, double stamp);
            void cmdCB(const geometry_msgs::TwistConstPtr& msg);

            // Ethernet relate variables
            std::string device_ip_string;
            in_addr device_ip;
            int UDP_PORT_NUMBER;
            int socket_fd;
            struct sockaddr_in mcu_addr;
            socklen_t mcu_addr_len;
            std::string MCU_ip_string;
            in_addr MCU_ip;
            int MCU_UDP_PORT_NUMBER;
            // int MCU_socket_fd;

            // ROS related variables
            ros::NodeHandle nh;
            ros::NodeHandle pnh;

            std::string frame_id;
            ros::Publisher packet_pub;
            ros::Publisher pub_imu;
            ros::Publisher pub_pose;
            geometry_msgs::TwistStamped pub_msg_pose;
            sensor_msgs::Imu pub_msg_imu;
            ros::Subscriber sub_vel;

    };

}