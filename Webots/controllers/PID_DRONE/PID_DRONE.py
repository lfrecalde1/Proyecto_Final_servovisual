from controller import *
import cv2 
import time
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# Declaracion de las variables de comuinicacion
vx_c=0
vy_c=0
vz_c=0
wx_c=0
wy_c=0
wz_c=0
def velocityCallback(velocity_message):
    global vx_c
    global vy_c
    global vz_c
    global wx_c
    global wy_c
    global wz_c

    vx_c=velocity_message.linear.x
    vy_c=velocity_message.linear.y
    vz_c=velocity_message.linear.z
    wx_c=velocity_message.angular.x
    wy_c=velocity_message.angular.y
    wz_c=velocity_message.angular.z

def inicializar(robot):
    frontLeftMotor = robot.getMotor('front left propeller')
    frontRightMotor = robot.getMotor('front right propeller')
    backLeftMotor = robot.getMotor('rear left propeller')
    backRightMotor = robot.getMotor('rear right propeller') 
    return [frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor]

def config(robot):
    [frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor]=inicializar(robot)
    frontLeftMotor.setPosition(float('inf'))
    frontRightMotor.setPosition(float('inf'))
    backLeftMotor.setPosition(float('inf'))
    backRightMotor.setPosition(float('inf'))
    return None

def envio(robot,w1,w2,w3,w4):
    [frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor]=inicializar(robot)
    frontLeftMotor.setVelocity(w1)
    frontRightMotor.setVelocity(w2)
    backLeftMotor.setVelocity(w3)
    backRightMotor.setVelocity(w4)
    return None

def pid(sp,real,err_1,err_2,u_1,kp,ki,kd,ts):
    err=np.tanh(sp-real)
    proporcional=kp*(err-err_1)
    integral=ki*err*ts
    derivativo=kd*(err-2*err_1+err_2)/ts
    u=u_1+proporcional+integral+derivativo
    return u,err

def pid_v(sp,real,err_1,err_2,u_1,kp,ki,kd,ts):
    err=np.tanh(sp-real)
    proporcional=kp*(err-err_1)
    integral=ki*err*ts
    derivativo=kd*(err-2*err_1+err_2)/ts
    u=u_1+proporcional+integral+derivativo
    return u,err

def bucle(robot,odometry_publisher,odometry_message):
    timestep = int(robot.getBasicTimeStep())
    t_sample = 0.01

    # CONFIGURACION DE LOS MOTORES PARA QUE MUEVAN DE MANERA CONTINUA
    config(robot)
    envio(robot, 0, 0, 0, 0)

    # DEFINICION DE LOS SENSORES QUE DIPONE EL ROBOT


    # DEFINCION DEL SENSOR GPS
    gps = GPS("gps")
    gps.enable(timestep)

    # DFINCION DE LA IMU
    imu = InertialUnit("inertial unit")
    imu.enable(timestep)

    # DEFINICION DELC OMPAS
    compass = Compass("compass")
    # DEFINCION DEL GYRO

    compass.enable(timestep)
    gyro = Gyro("gyro")
    gyro.enable(timestep)

    # Valor equlibri0
    vertical_thrust = 60
    envio(robot, vertical_thrust, -vertical_thrust, -vertical_thrust, vertical_thrust)
    # Declaracion de las contantes para el controlador de elevacion
    uz_1 = 0
    errz_1 = 0
    errz_2 = 0
    z_1 = 0

    # delcaracion de las contantes para el control latera
    ur_1 = 0
    errr_1 = 0
    errr_2 = 0
    y_1 = 0

    # declaracion de las constantes para el control frontal
    up_1 = 0
    errp_1 = 0
    errp_2 = 0
    x_1 = 0

    # declaracion de las contantes del control de orientacion
    uy_1 = 0
    erry_1 = 0
    erry_2 = 0

    # Contantes del sitema
    M_PI = 3.1415926535897932384626433
    sp_z = 1
    sp_roll = 0
    sp_pitch = 0
    sp_yaw = 0

    # velocidades deseadasd el sistema
    vxd = 0
    vyd = 0
    vzd = 0.5
    vpsid = 0

    # declaracion de los valores del control para la velocidad forntal
    err_vx2 = 0
    err_vx1 = 0
    sp_pitch_1 = 0

    # declaracion de las variables para la velocidad frontal
    err_vy2 = 0
    err_vy1 = 0
    sp_roll_1 = 0

    t = 0
    k = 0

    loop_rate = rospy.Rate(100)
    while robot.step(timestep) != -1:
        if (t > 7):
            vxd = vx_c
            vyd = vy_c
            vzd = vz_c
            sp_yaw = wz_c

        tiempo = time.time()
        # OBTENCION DE LAS POSICIONES DEL ROBOT
        posicion = gps.getValues()
        y = -posicion[2]
        x = posicion[0]
        z = posicion[1]

        if k > 1:
            vz = (z - z_1) / t_sample
            vx = (x - x_1) / t_sample
            vy = (y - y_1) / t_sample
        else:
            vz = 0
            vx = 0
            vy = 0

        roll = -imu.getRollPitchYaw()[0] - M_PI / 2.0
        pitch = -imu.getRollPitchYaw()[1]
        yaw = imu.getRollPitchYaw()[2]

        roll_acceleration = gyro.getValues()[0]
        pitch_acceleration = gyro.getValues()[1]
        w = gyro.getValues()[2]
        # Declaracion de las matrices de transformacion para las velocidades del robot
        Tz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        Vg = np.array([[vx], [vy], [vz]])
        Vi = np.linalg.inv(Tz) @ Vg

        #Envio datos por ros de la odometria
        odometry_message.pose.pose.position.x = x
        odometry_message.pose.pose.position.y = y
        odometry_message.pose.pose.position.z = z

        odometry_message.pose.pose.orientation.x = roll
        odometry_message.pose.pose.orientation.y = pitch
        odometry_message.pose.pose.orientation.z = yaw

        odometry_message.twist.twist.linear.x = Vi[0, 0]
        odometry_message.twist.twist.linear.y = Vi[1, 0]
        odometry_message.twist.twist.linear.z = Vi[2, 0]

        odometry_message.twist.twist.angular.x = roll_acceleration
        odometry_message.twist.twist.angular.y = pitch_acceleration
        odometry_message.twist.twist.angular.z = w

        rospy.loginfo("Enviando Odometria..")
        odometry_publisher.publish(odometry_message)



        # Declaracion de los pid d velocidades del sistema
        sp_pitch, err_vx = pid_v(vxd, Vi[0, 0], err_vx1, err_vx2, sp_pitch_1, 0.6, 0.2, 0.001, t_sample)

        sp_roll, err_vy = pid_v(vyd, Vi[1, 0], err_vy1, err_vy2, sp_roll_1, 0.6, 0.2, 0.001, t_sample)
        # declarcion del pid de altitud lateral y frontal
        vertical_input, errz = pid(vzd, Vi[2, 0], errz_1, errz_2, uz_1, 50, 5, 100, t_sample)

        roll_input, errroll = pid(sp_roll, roll, errr_1, errr_2, ur_1, 50, 1, 100, t_sample)

        pitch_input, errpitch = pid(sp_pitch, pitch, errp_1, errp_2, up_1, 50, 1, 100, t_sample)

        yaw_input, erryaw = pid(sp_yaw, w, erry_1, erry_2, uy_1, 10, 0.1, 20, t_sample)

        # declaracion para cada motor
        front_left_motor_input = vertical_thrust + vertical_input - roll_input - pitch_input - yaw_input
        front_right_motor_input = vertical_thrust + vertical_input + roll_input - pitch_input + yaw_input
        rear_left_motor_input = vertical_thrust + vertical_input - roll_input + pitch_input + yaw_input
        rear_right_motor_input = vertical_thrust + vertical_input + roll_input + pitch_input - yaw_input

        # envio valores al drone
        envio(robot, front_left_motor_input, -front_right_motor_input, -rear_left_motor_input, rear_right_motor_input)

        #print(vxd, Vi[0, 0],"....", vyd, Vi[1, 0],"....", vzd, Vi[2, 0],"....", sp_yaw, w)
        print("ENVIANDO DATOS A LOS NODOS..")
        # atualizacion de datos de los controladores

        z_1 = z
        errz_2 = errz_1
        errz_1 = errz
        uz_1 = vertical_input

        y_1 = y
        errr_2 = errr_1
        errr_1 = errroll
        ur_1 = roll_input

        x_1 = x
        errp_2 = errp_1
        errp_1 = errpitch
        up_1 = pitch_input

        erry_2 = erry_1
        erry_1 = erryaw
        uy_1 = yaw_input

        err_vx2 = err_vx1
        err_vx1 = err_vx
        sp_pitch_1 = sp_pitch

        err_vy2 = err_vy1
        err_vy1 = err_vy
        sp_roll_1 = sp_roll
        # Dibujo del drone
        # parte importante par aobtener la derivada correcta del robot
        k = k + 1
        loop_rate.sleep()
        delta = time.time() - tiempo
        ## tiempo de simulacion
        t = t + delta

if __name__=='__main__':
    try:
        # Inicializacion del nodo
        rospy.init_node('Escuchar_velocidades_ESCRIBIR_ODOMETRIA', anonymous=True)

        # Topico para escuchar los comandos de las velocidades
        velocidad_topic = '/Mavic_pro/cmd_vel'
        pose_subscriber = rospy.Subscriber(velocidad_topic, Twist, velocityCallback)

        #Topico para publicar la odometria del robot
        odometria_topic='/Mavic_pro/odom'
        odometry_message=Odometry()
        odometry_publisher = rospy.Publisher(odometria_topic, Odometry, queue_size=10)
        robot=Robot()
        bucle(robot,odometry_publisher,odometry_message)

    except KeyboardInterrupt:
        print("Press Ctrl-C to terminate while statement")
        pass


