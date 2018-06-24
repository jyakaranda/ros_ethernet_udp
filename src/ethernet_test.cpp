#include <ros/ros.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <ethernet_test/user_protocol.h>

#define SERVER_PORT 8888
#define SERVER_IP "127.0.0.1"

void udp_msg_sender(int sock_fd, const struct sockaddr* dst){
	socklen_t len;
	PacketData packet;
	ros::Duration duration(1);

	while(1){
		static char buf[PACKET_SIZE + 1];
		len = sizeof(*dst);
		packet.type = VAL_VEL;
		packet.dat.vel.liner[0] = 1;
		packet.dat.vel.liner[1] = 1;
		packet.dat.vel.liner[2] = 1;
		packet.dat.vel.angular[0] = 2;
		packet.dat.vel.angular[1] = 2;
		packet.dat.vel.angular[2] = 2;
		packet.dat.vel.angular[3] = 2;
		packet.syn_CR = '\r';
		packet.syn_LF = '\n';

		ssize_t nbytes = sendto(sock_fd, (uint8_t*)&packet, PACKET_SIZE, 0, dst, len);

		if(nbytes > 0){
			ROS_INFO("send a udp packet to dst %f", ros::Time::now().toSec());
			memset(buf, 0, sizeof(buf));
			struct sockaddr_in src;

			struct pollfd fds[1];
			fds[0].fd = sock_fd;
			fds[0].events = POLLIN;
			int flag = 1;

			ssize_t rbytes = recvfrom(sock_fd, buf, PACKET_SIZE, 0, (struct sockaddr*)&src, &len);
			if((size_t)rbytes == PACKET_SIZE){
				ROS_INFO("sender_address: %s", inet_ntoa(src.sin_addr));
				PacketData packet = *((PacketData*)(&(buf[0])));
				// if(inet_ntoa(src.sin_addr) == SERVER_IP){
					ROS_INFO("linear.x: %f", packet.dat.vel.liner[0]);
				// }
			} else{
				ROS_INFO("recv %d bytes from %s", (size_t)rbytes, inet_ntoa(src.sin_addr));
			}

			// duration.sleep();
		}
	}
}

int main(int argc, char** argv){
    
	ros::init(argc, argv, "ethernet_test");
	ros::NodeHandle n;
	ROS_INFO("ethernet_test");
	int client_fd;
	struct sockaddr_in ser_addr;
	client_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if(client_fd < 0){
		ROS_ERROR("cannot create socket!");
		return -1;
	}
	memset(&ser_addr, 0, sizeof(ser_addr));
	ser_addr.sin_family = AF_INET;
	ser_addr.sin_port = htons(SERVER_PORT);
	// ser_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	ser_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

	udp_msg_sender(client_fd, (struct sockaddr*)&ser_addr);
	close(client_fd);

    return 0;
}