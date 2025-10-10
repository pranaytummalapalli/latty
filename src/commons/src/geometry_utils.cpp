#include "commons/geometry_utils.hpp"

namespace Geometry {

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

namespace State {
// StateEuler3D member functions

StateEuler3D::StateEuler3D() {reset();}

void StateEuler3D::reset()
{
    this->timestamp = 0.0;
    this->pose.position.setZero();
    this->pose.q_rpy.setZero();
    this->twist.linear.setZero();
    this->twist.angular.setZero();
}

double StateEuler3D::distance_to(const StateEuler3D& other) const
{
    double dx = this->pose.position.x() - other.pose.position.x();
    double dy = this->pose.position.y() - other.pose.position.y();
    double dz = this->pose.position.z() - other.pose.position.z();
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Eigen::Matrix3d StateEuler3D::rotation_to(const StateEuler3D& other) const
{
    Eigen::Quaterniond q = SetQuaternion(this->pose.q_rpy.r(), 
                                         this->pose.q_rpy.p(), 
                                         this->pose.q_rpy.y());

    Eigen::Quaterniond q_other = SetQuaternion(other.pose.q_rpy.r(), 
                                               other.pose.q_rpy.p(), 
                                               other.pose.q_rpy.y());

    Eigen::Quaterniond q_rel = q.inverse() * q_other;
    q_rel.normalize();

    Eigen::Matrix3d R_rel = q_rel.toRotationMatrix();

    return R_rel;
}

Eigen::Vector3d StateEuler3D::angle_to(const StateEuler3D& other) const
{
    Eigen::Quaterniond q_self   = SetQuaternion(this->pose.q_rpy.r(), 
                                                this->pose.q_rpy.p(),  
                                                this->pose.q_rpy.y());

    Eigen::Quaterniond q_target = SetQuaternion(other.pose.q_rpy.r(), 
                                                other.pose.q_rpy.p(), 
                                                other.pose.q_rpy.y());

    Eigen::Quaterniond q_rel = q_self.inverse() * q_target;
    q_rel.normalize();

    Eigen::AngleAxisd aa(q_rel);
    return aa.axis() * aa.angle();
}

// StateQ member functions
StateQ3D::StateQ3D() {reset();}

void StateQ3D::reset()
{
    this->timestamp = 0.0;
    this->pose.position.setZero();
    this->pose.orientationq.setIdentity();
    this->twist.linear.setZero();
    this->twist.angular.setZero();
}

double StateQ3D::distance_to(const StateQ3D& other) const
{
    double dx = this->pose.position.x() - other.pose.position.x();
    double dy = this->pose.position.y() - other.pose.position.y();
    double dz = this->pose.position.z() - other.pose.position.z();
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Eigen::Matrix3d StateQ3D::rotation_to(const StateQ3D& other) const
{
    Eigen::Quaterniond q_rel = 
            this->pose.orientationq.inverse() * other.pose.orientationq;
    q_rel.normalize();

    Eigen::Matrix3d R_rel = q_rel.toRotationMatrix();
    return R_rel;
}

Eigen::Vector3d StateQ3D::angle_to(const StateQ3D& other) const
{
    Eigen::Quaterniond q_rel = 
            this->pose.orientationq.inverse() * other.pose.orientationq;
    q_rel.normalize();

    Eigen::AngleAxisd aa(q_rel);
    return aa.axis() * aa.angle();
}

//StateEuler2D member functions
StateEuler2D::StateEuler2D() {reset();}

void StateEuler2D::reset()
{
    this->timestamp = 0.0;
    this->pose.position.setZero();
    this->pose.yaw = 0.0;
    this->twist.linear.setZero();
    this->twist.yaw_rate = 0.0;
}

double StateEuler2D::distance_to(const StateEuler2D& other) const 
{
    double dx = this->pose.position.x() - other.pose.position.x();
    double dy = this->pose.position.y() - other.pose.position.y();
    return std::sqrt(dx * dx + dy * dy);
}

Eigen::Matrix3d StateEuler2D::rotation_to(const StateEuler2D& other) const
{
    double dtheta = this->pose.yaw - other.pose.yaw;
    Eigen::Matrix3d R;
    R << std::cos(dtheta), -std::sin(dtheta), 0,
         std::sin(dtheta),  std::cos(dtheta), 0,
         0               ,  0               , 1;
    return R;
}

double StateEuler2D::angle_to(const StateEuler2D& other) const
{
    return (this->pose.yaw - other.pose.yaw);
}

double StateEuler2D::bearing_to(const StateEuler2D& other) const
{
    return std::atan2(other.pose.position.y() - this->pose.position.y(),
                      other.pose.position.x() - this->pose.position.x());
}

//StateQ2D member functions
StateQ2D::StateQ2D() {reset();}

void StateQ2D::reset()
{
    this->timestamp = 0.0;
    this->pose.position.setZero();
    this->pose.orientationq.setIdentity();
    this->twist.linear.setZero();
    this->twist.yaw_rate = 0.0;
}

double StateQ2D::distance_to(const StateQ2D& other) const
{
    double dx = this->pose.position.x() - other.pose.position.x();
    double dy = this->pose.position.y() - other.pose.position.y();
    return std::sqrt(dx * dx + dy * dy);
}

Eigen::Matrix3d StateQ2D::rotation_to(const StateQ2D& other) const
{
    Eigen::Quaterniond q_rel = 
            this->pose.orientationq.inverse() * other.pose.orientationq;
    q_rel.normalize();

    Eigen::Matrix3d R_rel = q_rel.toRotationMatrix();
    return R_rel;
}

double StateQ2D::angle_to(const StateQ2D& other) const
{
    Eigen::Quaterniond q_rel = 
            this->pose.orientationq.inverse() * other.pose.orientationq;
    q_rel.normalize();

    Eigen::AngleAxisd aa(q_rel);
    return aa.angle() * aa.axis().z();
}
} //Geometry::State

namespace Point {
Point::Point(){reset();}

Point::Point(double x, double y) : x(x), y(y) {}

void Point::reset() {x = 0.0; y = 0.0;}

double Point::distance_to(const Point& other) const
{
    double dx = this->x - other.x;
    double dy = this->y - other.y;
    return std::sqrt(dx * dx + dy * dy);
}

}
} //geometry