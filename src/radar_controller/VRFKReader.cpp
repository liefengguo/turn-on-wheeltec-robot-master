#include "../../include/radar_controller/VRFKReader.h"

VRFKReader::VRFKReader() {
    subscriber = nh.subscribe("fixposition_driver/VRTK", 1, &VRFKReader::vrTkCallback, this);
}

void VRFKReader::vrTkCallback(const VRTK::ConstPtr& msg) {
    vrTkData = *msg;
}

std_msgs::Header VRFKReader::getHeader() const {
    return vrTkData.header;
}

std::string VRFKReader::getPoseFrame() const {
    return vrTkData.pose_frame;
}

std::string VRFKReader::getKinFrame() const {
    return vrTkData.kin_frame;
}

geometry_msgs::PoseWithCovariance VRFKReader::getPose() const {
    return vrTkData.pose;
}

geometry_msgs::TwistWithCovariance VRFKReader::getVelocity() const {
    return vrTkData.velocity;
}

geometry_msgs::Vector3 VRFKReader::getAcceleration() const {
    return vrTkData.acceleration;
}

int16_t VRFKReader::getFusionStatus() const {
    return vrTkData.fusion_status;
}

int16_t VRFKReader::getIMUBiasStatus() const {
    return vrTkData.imu_bias_status;
}

int16_t VRFKReader::getGNSSStatus() const {
    return vrTkData.gnss_status;
}

int16_t VRFKReader::getWheelspeedStatus() const {
    return vrTkData.wheelspeed_status;
}

std::string VRFKReader::getVersion() const {
    return vrTkData.version;
}
