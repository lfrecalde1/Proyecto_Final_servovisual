#!/usr/bin/python3
import os
import sys
import cv2
import time
import numpy as np

# Path of external libraries
sys.path.insert(0, "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/")
sys.path.insert(0, "/usr/local/webots/lib/controller/python36")
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from controller import *
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from calibration_store import load_coefficients

# Set env Variable name of the robot
os.environ['WEBOTS_ROBOT_NAME'] = 'Pioneer 3-DX'

# Global variables for for matrices of each camera left camera
path_l = '/home/fer/Control_servo_visual/Code/Visual_Servoing_Drone/Webots_controllers/left_parameters.yml'
mtx_l, dist_l = load_coefficients(path_l)

# Global variables for for matrices of each camera left camera
path_r = '/home/fer/Control_servo_visual/Code/Visual_Servoing_Drone/Webots_controllers/right_parameters.yml'
mtx_r, dist_r = load_coefficients(path_r)

# Velocities obtain through Ros
# Linear velocities
vxd = 0
vyd = 0
vzd = 0

# Angular velocities
wxd = 0
wyd = 0
wzd = 0


def velocity_call_back(velocity_message):
    global vxd, vyd, vzd, wxd, wyd, wzd
    # Read desired linear velocities from node
    vxd = velocity_message.linear.x
    vyd = velocity_message.linear.y
    vzd = velocity_message.linear.z

    # Read desired angular velocities from node
    wxd = velocity_message.angular.x
    wyd = velocity_message.angular.y
    wzd = velocity_message.angular.z
    return None


def config_robot(robot):
    # Function to init the system
    front_left_motor = robot.getMotor('left wheel')
    front_right_motor = robot.getMotor('right wheel')

    front_left_motor.setPosition(float('inf'))
    front_right_motor.setPosition(float('inf'))

    return front_left_motor, front_right_motor


def send_motor(motor, set_point):
    motor.setVelocity(set_point)
    return None


def gps_config(name, time_sensor):
    # Configuration Gps
    gps = GPS(name)
    gps.enable(time_sensor)
    return gps


def imu_config(name, time_sensor):
    # Configuration Imu
    imu = InertialUnit(name)
    imu.enable(time_sensor)
    return imu


def gyro_config(name, time_sensor):
    # Configuration Gyro
    gyro = Gyro(name)
    gyro.enable(time_sensor)
    return gyro


def system_pos(gps):
    # Values gps
    h = gps.getValues()
    h = np.array(h).reshape(3, 1)

    # Creation transformation matrix
    r = Rotation.from_euler('x', [90], degrees=True)
    rot = r.as_matrix().reshape(3, 3)

    # Projection gps to general system
    real_pos = rot @ h
    return real_pos


def system_ori(imu):
    # Read Roll pitch yaw sensor
    ori = imu.getRollPitchYaw()
    ori_f = np.array([[ori[0]], [-ori[1]], [ori[2]]])
    return ori_f


def system_angular_velocities(gyro):
    angular = gyro.getValues()
    angular_f = np.array([[angular[0]], [angular[1]], [angular[2]]])
    return angular_f


def system_linear_velocities(pos, pos_1, delta_time, i, rpy):
    # Declaration of the actual states
    x = pos[0, 0]
    y = pos[1, 0]
    z = pos[2, 0]

    # Declaration of the previous state
    x_1 = pos_1[0, 0]
    y_1 = pos_1[1, 0]
    z_1 = pos_1[2, 0]

    if i > 1:
        vx = (x - x_1) / delta_time
        vy = (y - y_1) / delta_time
        vz = (z - z_1) / delta_time
    else:
        vx = 0
        vy = 0
        vz = 0

    # Matrix transformation
    rot = rot_matrix(rpy)

    # Velocities vector
    vel = np.array([[vx], [vy], [vz]])

    vel_f = np.linalg.inv(rot) @ vel

    pos_1 = pos

    return vel_f, pos_1


def rot_matrix(ori):
    # Calc of the rotational matrix
    roll = ori[0, 0]
    pitch = ori[1, 0]
    yaw = ori[2, 0]

    # Rotational matrices
    rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    rt = rz @ ry @ rx
    return rt


def pid(sp, real, memories, kp, ki, kd, t_sample):
    error = np.tanh(sp - real)
    error_1 = memories[1, 0]
    error_2 = memories[2, 0]
    u_1 = memories[0, 0]
    p = kp * (error - error_1)
    i = ki * error * t_sample
    d = kd * (error - 2 * error_1 + error_2) / t_sample
    u = u_1 + p + i + d

    # Update memories
    memories[0, 0] = u
    memories[2, 0] = error_1
    memories[1, 0] = error
    return u, memories


def robot_controller(vd, wd, motors, parameters):
    # Robot parameters
    r = parameters[0]
    d = parameters[1]

    # Definition of linear and angular velocities
    u = vd[0, 0]
    w = wd[2, 0]
    v = np.array([[u], [w]])
    trans = np.array([[r/2, r/2], [r/d, -r/d]])
    wheels = np.linalg.inv(trans)@v
    send_motor(motors[0], wheels[1, 0])
    send_motor(motors[1],  wheels[0, 0])
    return None


def send_odometry(h, rpy, vel, otw, odom_plu, odom_mess):
    # Creation of Rotation matrix and transformation to quaternions
    rot = rot_matrix(rpy)
    rotation = Rotation.from_matrix(rot)
    quaternion = rotation.as_quat()

    # Formation of odometry  position and orientation message
    odom_mess.pose.pose.position.x = h[0, 0]
    odom_mess.pose.pose.position.y = h[1, 0]
    odom_mess.pose.pose.position.z = h[2, 0]

    odom_mess.pose.pose.orientation.x = quaternion[0]
    odom_mess.pose.pose.orientation.y = quaternion[1]
    odom_mess.pose.pose.orientation.z = quaternion[2]
    odom_mess.pose.pose.orientation.w = quaternion[3]

    # Formation od odometry message linear and angular velocities
    odom_mess.twist.twist.linear.x = vel[0, 0]
    odom_mess.twist.twist.linear.y = vel[1, 0]
    odom_mess.twist.twist.linear.z = vel[2, 0]

    odom_mess.twist.twist.angular.x = otw[0, 0]
    odom_mess.twist.twist.angular.y = otw[1, 0]
    odom_mess.twist.twist.angular.z = otw[2, 0]

    # Publication Message
    odom_plu.publish(odom_mess)
    return None


def camera_system(robot, name, timestep):
    # System configuration for camera
    camera = robot.getCamera(name)
    camera.enable(timestep)
    return camera


def get_image(camera):
    # Adquisition of camera information
    data = camera.getImage()

    # Decoding image more faster that others metods
    img = np.frombuffer(data, np.uint8)

    # Resize the image to the respective dimesions
    aux = img.reshape(camera.getHeight(), camera.getWidth(), 4)

    # Convert image to the respective type of open cv
    frame = cv2.cvtColor(aux, cv2.COLOR_BGRA2BGR)

    return frame


def send_image(bridge, imglr_pub, imgr, imgl):
    # Concatenate images L and R
    final = np.hstack((imgl, imgr))
    # Decode Image left
    msglr = bridge.cv2_to_imgmsg(final, 'bgr8')
    imglr_pub.publish(msglr)

    return None


def main(robot, odom_plu, odom_mess, im_lr_plu):
    # Minimal time for simulation
    time_step = int(robot.getBasicTimeStep())

    # Sample time
    sample_time = 0.01
    
    # Definition bridge used in convertion
    bridge = CvBridge()

    # Camera system sensor
    camera_r = camera_system(robot, "camera_r", time_step)
    camera_l = camera_system(robot, "camera_l", time_step)

    # Robot Configuration for motor
    motor_1, motor_2 = config_robot(robot)
    motors = [motor_1, motor_2]

    # Set motor velocities to zero
    send_motor(motor_1, 0)
    send_motor(motor_2, 0)

    # Robot parameters radius and length
    r = 0.160/2
    length = 0.321
    param = [r, length]

    # Robot configuration GPS
    gps_sensor = gps_config("gps", time_step)

    # Robot configuration Imu
    imu_sensor = imu_config("inertial unit", time_step)

    # Robot configuration Gyro
    gyro_sensor = gyro_config("gyro", time_step)

    # Definition  of previous position states for estimation of the velocity
    h_1 = np.array([[0.0], [0.0], [0.0]])

    # Aux value For interactions
    k = 0

    # Reference velocities Drone
    vd = np.array([[0.0], [0.0], [0.0]])
    wd = np.array([[0.0], [0.0], [0.0]])

    # definition of time
    t = 0

    loop_rate = rospy.Rate(100)

    while robot.step(time_step) != -1:
        tic = time.time()
        if t > 1:
            # Update desired linear velocities
            vd[0, 0] = vxd
            vd[1, 0] = vyd
            vd[2, 0] = vzd

            # Update desired angular velocities from node
            wd[0, 0] = wxd
            wd[1, 0] = wyd
            wd[2, 0] = wzd

        # Position of the mobile robot
        h = system_pos(gps_sensor)

        # Roll pitch yaw Mobile robot
        rpy = system_ori(imu_sensor)

        # System angular velocities
        otw = system_angular_velocities(gyro_sensor)

        # Linear velocities of the mobile robot
        vel, h_1 = system_linear_velocities(h, h_1, sample_time, k, rpy)

        # Adquisition of image throught Webots
        img_r = get_image(camera_r)
        img_l = get_image(camera_l)

        # send Odometry of the robot to node
        send_odometry(h, rpy, vel, otw, odom_plu, odom_mess)
        send_image(bridge, im_lr_plu, img_r, img_l)

        # Robot controller
        robot_controller(vd, wd, motors, param)

        loop_rate.sleep()
        k = k + 1
        delta = time.time() - tic
        t = t + delta

    return None


if __name__ == '__main__':
    try:
        rospy.init_node('Pioner_3dx_Webots', anonymous=False)

        # Topic definition of velocities commands
        velocity_topic = "Pioner_3dx/cmd_vel"
        velocity_subscriber = rospy.Subscriber(velocity_topic, Twist, velocity_call_back)

        # Topic definition of Odometry
        odometry_topic = "Pioner_3dx/odom"
        odometry_message = Odometry()
        odometry_publisher = rospy.Publisher(odometry_topic, Odometry, queue_size=10)

        image_lr_topic = "Pioner_3dx/camera_lr"
        image_lr_publisher = rospy.Publisher(image_lr_topic, Image, queue_size=1)

        # Definition of the object robot part of Simulator
        robot1 = Robot()
        main(robot1, odometry_publisher, odometry_message, image_lr_publisher)

    except KeyboardInterrupt:
        print("Pres Ctrl-c to end the statement")
