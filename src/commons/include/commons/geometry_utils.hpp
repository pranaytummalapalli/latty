#pragma once 

#include <cmath>
#include <Eigen/Dense>

namespace Geometry {

// Helper functions for Eigen Quaternion orientations
Eigen::Quaterniond SetQuaternion(double roll, double pitch, double yaw);

// Helper functions for Eigen Rotation matrix 
Eigen::Vector3d getRPYFromRotation(const Eigen::Matrix3d& R);

namespace State {
//3D State
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

struct StateEuler3D
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

    StateEuler3D();

    void reset();

    // Operators (Helpers for ROS Nodes)
    double distance_to(const StateEuler3D& other) const;

    //rotation matrix from StateEuler3D to target state. 
    //rotation required to go from current q to target q    
    Eigen::Matrix3d rotation_to(const StateEuler3D& other) const;
    Eigen::Vector3d angle_to(const StateEuler3D& other) const;

};

struct StateQ3D
{
    double timestamp = 0.0;

    struct pose
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientationq; //orientation in quaternion
    } pose;

    struct twist
    {
        Eigen::Vector3d linear;
        Eigen::Vector3d angular;
    } twist;

    StateQ3D();
    void reset();

    // Operators (Helpers for ROS Nodes)
    double distance_to(const StateQ3D& other) const;
    
    //rotation matrix from StateQ3D to target state. 
    //rotation required to go from current q to target q
    Eigen::Matrix3d rotation_to(const StateQ3D& other) const;
    Eigen::Vector3d angle_to(const StateQ3D& other) const;
};

//2D State
struct StateEuler2D
{
    double timestamp = 0.0;

    struct pose
    {
        Eigen::Vector2d position;
        double yaw; //orientation in rpy
    } pose;

    struct twist
    {
        Eigen::Vector2d linear;
        double yaw_rate;
    } twist;

    StateEuler2D();

    void reset();

    // Operators (Helpers for ROS Nodes)
    double distance_to(const StateEuler2D& other) const;

    //rotation matrix from StateEuler3D to target state. 
    //rotation required to go from current q to target q    
    Eigen::Matrix3d rotation_to(const StateEuler2D& other) const;
    double angle_to(const StateEuler2D& other) const;
    double bearing_to(const StateEuler2D& other) const;
};

struct StateQ2D
{
    double timestamp = 0.0;

    struct pose
    {
        Eigen::Vector2d position;
        Eigen::Quaterniond orientationq; //orientation in quaternion
    } pose;

    struct twist
    {
        Eigen::Vector2d linear;
        double yaw_rate;
    } twist;

    StateQ2D();
    void reset();

    // Operators (Helpers for ROS Nodes)
    double distance_to(const StateQ2D& other) const;
    
    //rotation matrix from StateQ3D to target state. 
    //rotation required to go from current q to target q
    Eigen::Matrix3d rotation_to(const StateQ2D& other) const;
    double angle_to(const StateQ2D& other) const;
};
} //Geometry::State

namespace Point{
struct Point 
{
    double x;
    double y;

    Point();
    Point(const double x, const double y);
    void reset();
    double distance_to(const Point& other) const;
};
} //Geometry::Point


} //Geometry