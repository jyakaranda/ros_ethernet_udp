#include <ethernet_test/ethernet_driver.h>

namespace ethernet_driver{
    EthernetDriver::EthernetDriver(ros::NodeHandle& n, ros::NodeHandle& np):n_(n), np_(np)
    , socket_fd(-1){
        return;
    }

    EthernetDriver::~EthernetDriver(){
        (void) close(socket_fd);
        return;
    }

    bool EthernetDriver::initialize(){
        if (!loadParams()) {
            ROS_ERROR("Cannot load all required ROS parameters...");
            return false;
        }

        if (!createRosIO()) {
            ROS_ERROR("Cannot create all ROS IO...");
            return false;
        }

        if (!openUDPPort()) {
            ROS_ERROR("Cannot open UDP port...");
            return false;
        }
        ROS_INFO("Initialised ethernet_node without error");
        return true;
    }

    bool EthernetDriver::loadParams(){
        np_.param("device_ip", device_ip_string, std::string("127.0.0.1"));
        np_.param("device_port", UDP_PORT_NUMBER, 8888);
        np_.param("debug", DEBUG, false);
        np_.param("mcu_ip", MCU_ip_string, std::string("127.0.0.1"));
        np_.param("mcu_port", MCU_UDP_PORT_NUMBER, 8889);
        np_.param("poll_timeout", POLL_TIMEOUT, 2000);

        inet_aton(device_ip_string.c_str(), &device_ip);
        inet_aton(MCU_ip_string.c_str(), &MCU_ip);
        socket_fd = -1;
        ROS_INFO_STREAM("Opening UDP socket: address " << device_ip_string);
        ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
        return true;
    }

    bool EthernetDriver::createRosIO(){

        // Output
        pub_imu = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 50);
        pub_pose = nh.advertise<geometry_msgs::TwistStamped>("/raw_vel", 50);
        sub_vel = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 50, boost::bind(&EthernetDriver::cmdCB, this, _1));

        return true;

    }

    bool EthernetDriver::openUDPPort(){
        socket_fd = socket(PF_INET, SOCK_DGRAM, 0);
        if (socket_fd == -1) {
            perror("socket");
            return false;
        }
        memset(&mcu_addr, 0, sizeof(mcu_addr));
        mcu_addr.sin_family = AF_INET;
        mcu_addr.sin_addr.s_addr = inet_addr(MCU_ip_string.c_str());
        mcu_addr.sin_port = htons(MCU_UDP_PORT_NUMBER);
        mcu_addr_len = sizeof(mcu_addr);

        sockaddr_in my_addr;                     // my address information
        memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
        my_addr.sin_family = AF_INET;            // host byte order
        my_addr.sin_port = htons(UDP_PORT_NUMBER);      // short, in network byte order
        ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
        my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP

        if (bind(socket_fd, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
            perror("bind");                 // TODO: ROS_ERROR errno
            return false;
        }

        if (fcntl(socket_fd, F_SETFL, O_NONBLOCK|FASYNC) < 0) {
            perror("non-block");
            return false;
        }

        return true;

    }

    int EthernetDriver::getPacket(PacketData& packet, double& stamp){
        double time1 = ros::Time::now().toSec();

        struct pollfd fds[1];
        fds[0].fd = socket_fd;
        fds[0].events = POLLIN;
        // static const int POLL_TIMEOUT = 2000; // one second (in msec)

        sockaddr_in sender_address;
        socklen_t sender_address_len = sizeof(sender_address);

        while (true)
        {
            // Unfortunately, the Linux kernel recvfrom() implementation
            // uses a non-interruptible sleep() when waiting for data,
            // which would cause this method to hang if the device is not
            // providing data.  We poll() the device first to make sure
            // the recvfrom() will not block.
            //
            // Note, however, that there is a known Linux kernel bug:
            //
            //   Under Linux, select() may report a socket file descriptor
            //   as "ready for reading", while nevertheless a subsequent
            //   read blocks.  This could for example happen when data has
            //   arrived but upon examination has wrong checksum and is
            //   discarded.  There may be other circumstances in which a
            //   file descriptor is spuriously reported as ready.  Thus it
            //   may be safer to use O_NONBLOCK on sockets that should not
            //   block.

            // poll() until input available
            do {
                int retval = poll(fds, 1, POLL_TIMEOUT);
                if (retval < 0)             // poll() error?
                {
                    if (errno != EINTR)
                        ROS_ERROR("poll() error: %s", strerror(errno));
                    return 1;
                }
                if (retval == 0)            // poll() timeout?
                {
                    ROS_WARN("mcu poll() timeout");
                    return 1;
                }
                if ((fds[0].revents & POLLERR)
                        || (fds[0].revents & POLLHUP)
                        || (fds[0].revents & POLLNVAL)) // device error?
                {
                    ROS_ERROR("poll() reports mcu error");
                    return 1;
                }
            } while ((fds[0].revents & POLLIN) == 0);

            // Receive packets that should now be available from the
            // socket using a blocking read.
            char buf[PACKET_SIZE + 1];
            ssize_t nbytes = recvfrom(socket_fd, &buf, PACKET_SIZE,  0,
                    (sockaddr*) &sender_address, &sender_address_len);
            // mcu_addr = sender_address;  // TEST: socket sendto in cmdCB
            //        ROS_DEBUG_STREAM("incomplete lslidar packet read: "
            //                         << nbytes << " bytes");
            PacketData* tmp = (PacketData *)(&(buf[0]));
            packet = *tmp;
            if (nbytes < 0)
            {
                if (errno != EWOULDBLOCK)
                {
                    perror("recvfail");
                    ROS_INFO("recvfail");
                    return 1;
                }
            }
            else if ((size_t) nbytes == PACKET_SIZE)
            {
                // read successful,
                // if packet is not from the lidar scanner we selected by IP,
                // continue otherwise we are done
                ROS_INFO("sender_address: %s, %f", inet_ntoa(sender_address.sin_addr), packet.dat.vel.liner[1]);
                if( MCU_ip_string != "" && sender_address.sin_addr.s_addr != MCU_ip.s_addr )
                    continue;
                else{
                    // ssize_t bytes = sendto(socket_fd, (uint8_t*)&packet, PACKET_SIZE, 0, (sockaddr*)&sender_address, sizeof(sender_address_len));
                    // ROS_INFO("send %d bytes to mcu: %s", (size_t)bytes, inet_ntoa(sender_address.sin_addr));
                    break; //done  
                }
                    
            }
        }
        // Average the times at which we begin and end reading.  Use that to
        // estimate when the scan occurred.
        double time2 = ros::Time::now().toSec();
        stamp = ros::Time((time2 + time1) / 2.0).toSec();

        return 0;
    }

    bool EthernetDriver::pubMsg(PacketData data, double stamp){
        if(data.type == VAL_VEL)
		{
			pub_msg_pose.header.stamp = ros::Time(stamp);
			pub_msg_pose.twist.linear.x = data.dat.vel.liner[0];
			pub_msg_pose.twist.linear.y = data.dat.vel.liner[1];
			pub_msg_pose.twist.linear.z = 0;
			pub_pose.publish(pub_msg_pose);
			if(DEBUG)
			{
				ROS_INFO("VAL_POINT[%.1f,%.1f,%.1f]", pub_msg_pose.twist.linear.x
				, pub_msg_pose.twist.linear.y
				, pub_msg_pose.twist.linear.z);
			}

		}

		if(data.type == VAL_IMU)
		{
			pub_msg_imu.header.stamp = ros::Time(stamp);
			pub_msg_imu.header.frame_id = "imu_link";
			
			pub_msg_imu.angular_velocity.x = data.dat.vel.angular[0];
			pub_msg_imu.angular_velocity.y = data.dat.vel.angular[1];
			pub_msg_imu.angular_velocity.z = data.dat.vel.angular[2];
			pub_imu.publish(pub_msg_imu);
			if(DEBUG)
			{
				ROS_INFO("IMU RPY[%f, %f, %f]XYZ", data.dat.vel.angular[0], data.dat.vel.angular[1], data.dat.vel.angular[2]);
			}
		}
    }

    bool EthernetDriver::polling(){

        PacketData packet;
        double stamp;
        while(true){
            int rc = getPacket(packet, stamp);
            ROS_INFO("rc: %d", rc);
            if(rc > 0)  return true;
            if(rc == 0)  break;
            if(rc < 0)  return false;
        }

        ROS_DEBUG("receive a packet");

        if(stamp > 0){
            pubMsg(packet, stamp);
        }

        return true;
    }

    void EthernetDriver::cmdCB(const geometry_msgs::TwistConstPtr& msg){
        if(socket_fd < 0){
            ROS_ERROR("Socket didnot successfully bind!");
            return;
        }
        static PacketData packet_pub;
        packet_pub.type = VAL_VEL;
        packet_pub.dat.vel.liner[0] = msg->linear.x;
        packet_pub.dat.vel.liner[1] = msg->linear.y;
        packet_pub.dat.vel.liner[2] = msg->linear.z;
        packet_pub.dat.vel.angular[0] = msg->angular.x;
        packet_pub.dat.vel.angular[1] = msg->angular.y;
        packet_pub.dat.vel.angular[2] = msg->angular.z;
        packet_pub.syn_CR = '\r';
        packet_pub.syn_LF = '\n';

        ROS_INFO("send packet to mcu");
        ssize_t nbytes = sendto(socket_fd, (uint8_t*)&packet_pub, PACKET_SIZE, 0, (struct sockaddr*)&mcu_addr, mcu_addr_len);
        if(nbytes <= 0){
            ROS_ERROR("cannot send cmd_vel to MCU!");
        }
    }

}