import sys
import rosbag
import matplotlib.pyplot as plt

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

# State published by the simulator
vehicleState_time = []
vehicleState_x = []
vehicleState_y = []
vehicleState_theta = []
vehicleState_velocity = []
vehicleState_steer = []

# State published by the controller
controllerState_time = []
controllerState_xref = []
controllerState_yref = []
controllerState_xPref = []
controllerState_yPref = []
controllerState_xP = []
controllerState_yP = []
controllerState_vPx = []
controllerState_vPy = []
controllerState_velocity = []
controllerState_steer = []
controllerState_xPerr = []
controllerState_yPerr = []


for topic, msg, t in bag.read_messages():
    if topic == "/car_state":
        vehicleState_time.append(msg.data[0])
        vehicleState_x.append(msg.data[1])
        vehicleState_y.append(msg.data[2])
        vehicleState_theta.append(msg.data[3])
        vehicleState_velocity.append(msg.data[4])
        vehicleState_steer.append(msg.data[5])

    if topic == "/controller_state":
        controllerState_time.append(msg.data[0])
        controllerState_xref.append(msg.data[1])
        controllerState_yref.append(msg.data[2])
        controllerState_xPref.append(msg.data[3])
        controllerState_yPref.append(msg.data[4])
        controllerState_xP.append(msg.data[5])
        controllerState_yP.append(msg.data[6])
        controllerState_vPx.append(msg.data[7])
        controllerState_vPy.append(msg.data[8])
        controllerState_velocity.append(msg.data[9])
        controllerState_steer.append(msg.data[10])
        controllerState_xPerr.append(msg.data[3]-msg.data[5])
        controllerState_yPerr.append(msg.data[4]-msg.data[6])

# Compute max values for Position Error
maxXP = max(controllerState_xPerr, key=abs)
maxYP = max(controllerState_yPerr, key=abs)
timeXP = controllerState_time[controllerState_xPerr.index(maxXP)]
timeYP = controllerState_time[controllerState_yPerr.index(maxYP)]


bag.close()


# Plot data
plt.figure(1)
plt.plot(vehicleState_x,vehicleState_y, label="actual XY trajectory")
plt.plot(controllerState_xref,controllerState_yref,'g', label="reference XY trajectory")
plt.plot(vehicleState_x[0],vehicleState_y[0],'ro', label="initial pos")
plt.plot(vehicleState_x[len(vehicleState_x)-1],vehicleState_y[len(vehicleState_x)-1],'rx', label="final pos")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend(loc='best')


plt.figure(2)
plt.subplot(211)
plt.plot(vehicleState_time,vehicleState_velocity, label="long vel input")
plt.xlabel("Time [s]")
plt.ylabel("Longitudinal velocity [m/s]")
plt.legend(loc='best')
plt.subplot(212)
plt.plot(vehicleState_time,vehicleState_steer, label="steer input")
plt.xlabel("Time [s]")
plt.ylabel("Steer position [rad]")
plt.legend(loc='best')


plt.figure(3)
plt.subplot(311)
plt.plot(vehicleState_time,vehicleState_x, label="x act")
plt.plot(controllerState_time,controllerState_xref, 'r--', label="x ref")
plt.xlabel("Time [s]")
plt.ylabel("x [m]")
plt.legend(loc='best')
plt.subplot(312)
plt.plot(vehicleState_time,vehicleState_y, label="y act")
plt.plot(controllerState_time,controllerState_yref, 'r--', label="y ref")
plt.xlabel("Time [s]")
plt.ylabel("y [m]")
plt.legend(loc='best')
plt.subplot(313)
plt.plot(vehicleState_time,vehicleState_theta, label="heading")
plt.xlabel("Time [s]")
plt.ylabel("theta [rad]")
plt.legend(loc='best')


plt.figure(4)
plt.subplot(211)
plt.plot(controllerState_time,controllerState_xPerr, label="xP error")
plt.plot(timeXP,maxXP,'ro', label="MAX xP error = "+str(round(maxXP,3))+" [m]")
plt.xlabel("Time [s]")
plt.ylabel("xP position error [m]")
plt.legend(loc='best')
plt.subplot(212)
plt.plot(controllerState_time,controllerState_yPerr, label="yP error")
plt.plot(timeYP,maxYP,'ro', label="MAX yP error = "+str(round(maxYP,3))+" [m]")
plt.xlabel("Time [s]")
plt.ylabel("yP position error [m]")
plt.legend(loc='best')


plt.show()

