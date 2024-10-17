#ifndef CAR_TRAJ_CTRL_H_
#define CAR_TRAJ_CTRL_H_

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <bicycle_kin_fblin.h>

#define NAME_OF_THIS_NODE "car_traj_ctrl"

class car_traj_ctrl
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber vehicleState_subscriber;
    ros::Publisher vehicleCommand_publisher, controllerState_publisher;

    /* Parameters from ROS parameter server */
    double P_dist, l, Kpx, Kpy, Tix, Tiy, Ts, a, T, pi;

    /* ROS topic callbacks */
    void vehicleState_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    /* Node periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    bicycle_kin_fblin* controller;
    double xref, yref, dxref, dyref, ddxref, ddyref, thref, dthref;
    double xP, yP, xPref, yPref, dxPref, dyPref;
    double vPx, vPy, v, phi;
    double xPintegral = 0.0;
    double yPintegral = 0.0;

  public:
    float RunPeriod;

    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);

};

#endif /* CAR_TRAJ_CTRL_H_ */
