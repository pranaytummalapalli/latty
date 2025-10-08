#pragma once

#include <cmath>
#include <Eigen/Dense>

namespace geometry {
namespace state {

struct RPY
{
    Eigen::Vector3d data;
    double& r() {return data.x();}
    double& p() {return data.y();}
    double& y() {return data.z();}

    const double& r() const {return data.x();}
    const double& p() const {return data.y();}
    const double& y() const {return data.z();}

    void setZero() {data.setZero();}
};

struct StateRPY
{
    double timestamp = 0.0;

    struct pose
    {
        Eigen::Vector3d position;
        RPY q_rpy; //orientation in rpy
    } pose;

    struct twist
    {
        Eigen::Vector3d linear;
        Eigen::Vector3d angular;
    } twist;

    void reset()
    {
        timestamp = 0.0;
        pose.position.setZero();
        pose.q_rpy.setZero();
        twist.linear.setZero();
        twist.angular.setZero();
    }

    // Operators (Helpers for ROS Nodes)

    double distance_to(const StateRPY& other)
    {
        double dx = pose.position.x() - other.pose.position.x();
        double dy = pose.position.y() - other.pose.position.y();
        double dz = pose.position.z() - other.pose.position.z();

        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    /*
    * rotation matrix from StateRPY to target state. rotation required to go from current q to target q
    */
    Eigen::Matrix3d rotation_to(const StateRPY& other)
    {
        Eigen::Quaterniond q;
        Eigen::Quaterniond q_other;

        q = SetQuaternion(pose.q_rpy.r(), pose.q_rpy.p(), pose.q_rpy.y());
        q_other = SetQuaternion(other.pose.q_rpy.r(), other.pose.q_rpy.p(), other.pose.q_rpy.y());

        Eigen::Quaterniond q_rel = q.inverse() * q_other;

        Eigen::Matrix3d R_rel(q_rel);

        return R_rel;
    }

    Eigen::Vector3d angle_to(const StateRPY& other)
    {
        Eigen::Quaterniond q_self   = SetQuaternion(pose.q_rpy.r(), pose.q_rpy.p(), pose.q_rpy.y());
        Eigen::Quaterniond q_target = SetQuaternion(other.pose.q_rpy.r(), other.pose.q_rpy.p(), other.pose.q_rpy.y());

        Eigen::Quaterniond q_rel = q_self.inverse() * q_target;
        q_rel.normalize();

        Eigen::AngleAxisd aa(q_rel);
        return aa.axis() * aa.angle();
    }

};


// Helper functions for Eigen Quaternion orientations

Eigen::Quaterniond SetQuaternion(double roll, double pitch, double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    q.normalize();

    return q;
}

// Helper functions for Eigen Rotation matrix 

Eigen::Vector3d getRPYFromRotation(const Eigen::Matrix3d& R)
{
    Eigen::Vector3d rpy;
    rpy.x() = std::atan2(R(2,1), R(2,2)); //roll
    rpy.y() = std::asin(-R(2,0));
    rpy.z() = std::atan2(R(1,0), R(0,0));

    return rpy;
}


    
}; //geometry::state
}; //geometry