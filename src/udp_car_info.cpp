#include "udp_log.h"
class CarInfoServer {
public:
    CarInfoServer() : sockfd(-1) {}


    bool initialize() {
        ros::NodeHandle n;
        // Create UDP socket
        if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
            std::cerr << "Failed to create socket!" << std::endl;
            return false;
        }

        // Set server address information
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(SERVER_PORT);
        if (inet_pton(AF_INET, SERVER_IP, &(serverAddr.sin_addr)) <= 0) {
            std::cerr << "Failed to set server address!" << std::endl;
            return false;
        }

        // Bind socket to local address and port
        if (bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
            std::cerr << "Failed to bind socket!" << std::endl;
            return false;
        }

        std::cout << "UDP server is listening on " << SERVER_IP << ":" << SERVER_PORT << std::endl;

        pub_carSpeed = n.advertise<turn_on_wheeltec_robot::Speed>("/fixposition/speed", 10);

        return true;
    }


    void run() {
        ssize_t numBytesReceived;
        while (ros::ok()) {
            numBytesReceived = recvfrom(sockfd, chassisParser.receivedData, sizeof(chassisParser.receivedData), 0,
                                        (struct sockaddr*)&clientAddr, &clientAddrLen);
            if (numBytesReceived == -1) {
                std::cerr << "Failed to receive data!" << std::endl;
                break;
            } else if (numBytesReceived == 0) {
                std::cout << "Connection closed by the client." << std::endl;
                break;
            } else {
                std::cout << "Received " << numBytesReceived << " bytes of data from "
                        << inet_ntoa(clientAddr.sin_addr) << ":" << ntohs(clientAddr.sin_port) << std::endl;

                // Process received data here
                // You can perform parsing, validation, or any other operations as needed
                // For example, print the received data content
                std::cout << "Received data: ";
                for (size_t i = 0; i < numBytesReceived; ++i) {
                    std::cout << std::hex << static_cast<int>(chassisParser.receivedData[i]) << " ";
                }
                chassisParser.parseChassisData();
                chassisParser.logChassisData();
                // chassisParser.closeLog();
                std::vector<int32_t> speedData;
                speedData.push_back(chassisParser.chassisData.carSpeed);  // Convert to mm/s

                speed.speeds = speedData;
                pub_carSpeed.publish(speed);

                std::cout << std::endl;
            }
        }
        shutdown(sockfd, SHUT_RDWR);
        ros::shutdown();
        close(sockfd);
    }
private:
    int sockfd;
    struct sockaddr_in serverAddr, clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);
    ChassisParser chassisParser;
    turn_on_wheeltec_robot::Speed speed;
    ros::Publisher pub_carSpeed;
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "car_info");
    CarInfoServer server;
    if (server.initialize()) {
        server.run();
    }
    ros::spin();
    return 0;
}