#include "car_traj_ctrl/car_traj_ctrl.h"

#include <unistd.h>


void car_traj_ctrl::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    // run_period
    FullParamName = ros::this_node::getName()+"/run_period";
    if (false == Handle.getParam(FullParamName, RunPeriod))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Controller parameters
    FullParamName = ros::this_node::getName()+"/P_dist";
    if (false == Handle.getParam(FullParamName, P_dist))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/l";
    if (false == Handle.getParam(FullParamName, l))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Kpx";
    if (false == Handle.getParam(FullParamName, Kpx))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Kpy";
    if (false == Handle.getParam(FullParamName, Kpy))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/Tix";
    if (false == Handle.getParam(FullParamName, Tix))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/Tiy";
    if (false == Handle.getParam(FullParamName, Tiy))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/Ts";
    if (false == Handle.getParam(FullParamName, Ts))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
  
    FullParamName = ros::this_node::getName() + "/a";
    if (false == Handle.getParam(FullParamName, a))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/T";
    if (false == Handle.getParam(FullParamName, T))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/pi";
    if (false == Handle.getParam(FullParamName, pi))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    vehicleState_subscriber = Handle.subscribe("/car_state", 1, &car_traj_ctrl::vehicleState_MessageCallback, this);
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/car_input", 1);
    controllerState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/controller_state", 1);

    /* Create controller class */
    controller = new bicycle_kin_fblin(P_dist);

    // Initialize controller parameters
    controller->set_bicycleParam(l);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void car_traj_ctrl::RunPeriodically(float Period)
{
    ros::Rate LoopRate(1.0/Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

    while (ros::ok())
    {
        PeriodicTask();

        ros::spinOnce();

        LoopRate.sleep();
    }
}

void car_traj_ctrl::Shutdown(void)
{
    // Delete controller object
    delete controller;

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void car_traj_ctrl::vehicleState_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    /* Set current vehicle state */
    // Current state: t, msg->data[0]; x, msg->data[1]; y, msg->data[2]; theta, msg->data[3];
    controller->set_bicycleState(msg->data.at(1), msg->data.at(2), msg->data.at(3));
}

void car_traj_ctrl::PeriodicTask(void)
{ 

    /* 8-shaped trajectory generation */
    // Trajectory parameters
    const double w = (2*pi)/T;

    // Trajectory computation
    xref = a * std::sin(w * ros::Time::now().toSec());
    dxref = w * a * std::cos(w * ros::Time::now().toSec());
    ddxref = -std::pow(w, 2.0) * a * std::sin(w * ros::Time::now().toSec());
    yref = a * std::sin(w * ros::Time::now().toSec()) * std::cos(w * ros::Time::now().toSec());
    dyref = w * a * (std::pow(std::cos(w * ros::Time::now().toSec()), 2.0) - std::pow(std::sin(w * ros::Time::now().toSec()), 2.0));
    ddyref = -4.0 * std::pow(w, 2.0) * a * std::sin(w * ros::Time::now().toSec()) * std::cos(w * ros::Time::now().toSec());
    dthref = (dxref * ddyref - dyref * ddxref) / (std::pow(dxref, 2.0) + std::pow(dyref, 2.0));

    /*  Compute the control action */
    // Transform trajectory to point P
    controller->reference_transformation_position(xref, yref, xPref, yPref);
    controller->reference_transformation_velocity(dxref, dyref, dthref, dxPref, dyPref);
    controller->output_transformation(xP, yP);

    // Trajectory tracking law ----------------------------------------------------------------------

    // PI (forward Euler) + FF (reference velocity of point P)

    vPx = dxPref + Kpx * (xPref - xP) + (Kpx / Tix) * (xPintegral);
    vPy = dyPref + Kpy * (yPref - yP) + (Kpy / Tiy) * (yPintegral);

    xPintegral += Ts * (xPref - xP);
    yPintegral += Ts * (yPref - yP);

    // ----------------------------------------------------------------------------------------------

    // Linearization law
    controller->control_transformation(vPx, vPy, v, phi);

    /*  Publish vehicle commands */
    std_msgs::Float64MultiArray vehicleCommandMsg;
    vehicleCommandMsg.data.push_back(ros::Time::now().toSec());
    vehicleCommandMsg.data.push_back(v);
    vehicleCommandMsg.data.push_back(phi);
    vehicleCommand_publisher.publish(vehicleCommandMsg);

    /*  Publish controller state */
    std_msgs::Float64MultiArray controllerStateMsg;
    controllerStateMsg.data.push_back(ros::Time::now().toSec());
    controllerStateMsg.data.push_back(xref);
    controllerStateMsg.data.push_back(yref);
    controllerStateMsg.data.push_back(xPref);
    controllerStateMsg.data.push_back(yPref);
    controllerStateMsg.data.push_back(xP);
    controllerStateMsg.data.push_back(yP);
    controllerStateMsg.data.push_back(vPx);
    controllerStateMsg.data.push_back(vPy);
    controllerStateMsg.data.push_back(v);
    controllerStateMsg.data.push_back(phi);
    controllerState_publisher.publish(controllerStateMsg);
}
