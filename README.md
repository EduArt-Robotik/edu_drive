# edu_drive
This package comprises a ROS interface for EduArt's generic drive concept. It covers several kinematic concepts: Differential drive, Mecanum steering and Skid steering. All three can be used in dependency of the mounted wheels and the configuration of YAML parameters.

## Launching the Robot
In order to run the robot, you need to launch the appropriate launch file. In the launch folder, there is a prepared template.
Configure the correct kinematic concept and motor parameters. A description of the YAML file can be found below.
```console
roslaunch edu_drive edu_drive.launch
```
When everthing is initialized well, one should see the following output:
```console
# Listener start
CAN Interface: can0
[ INFO] [1651663592.994224328]: Instanciated robot with vMax: 0.366519 m/s and omegaMax: 0.733038 rad/s
...
```
After startup, the drive system is in the deactivated state.

Please notice also, that the ROS variables ROS_MASTER_URI and ROS_IP should be set properly. If you have a changing IP-address of the robot, you might consider to use the following bash code in your ~/.bashrc:

```console
MYIP=`hostname -I | awk '{print $1}'`
export ROS_MASTER_URI=http://${MYIP}:11311
export ROS_IP=${MYIP}
``` 

## YAML file parameters

| tag    | description      |
| ------ |:--|
| canInterface   | SocketCAN interface, e.g. can0       |
| frequencyScale | Divider of PWM frequency, Base frequency: 500kHz (Divider=1)     |
| inputWeight    | Low pass filtering, setpoint = inputWeight*setpoint_new + (1-inputWeight)*setpoint_old  |
| maxPulseWidth  | Limitation of pulse width, meaningful values [0; 100]      |
| timeout        | Dead time monitoring of communication (in milliseconds) |
| kp, ki, kd     | Controller coefficients    |
| antiWindup     | Enable anti windup monitoring of closed-loop controller     |
| invertEnc      | Invert encoder signal |
| responseMode   | Activate transmission of RPM or position measurements of motor controller |
| controllers    | Number of motor controllers used |
| canID          | Motorcontroller ID (set by DIP switch on motor controller PCB) |
| gearRatio      | Gear ratio of connected geared motors |
| encoderRatio   | Encoder pulses per motor revolution (rising and falling edge evaluation) |
| rpmMax         | Maximum revolutions per minute of geared motor pinion |
| channel        | Used channel of motor controller. There are single-channel motorshields and dual-channel motorshields. Meaningful values are 0 or 1. |
| kinematics     | Three-dimensional vector describing the conversion from Twist messages in motor revolutions. See explanation below. |

## Calculation of the kinematic parameters
The kinematic concept uses a conversion matrix for the conversion of twist parameters and wheel speeds.
$$
\left(\begin{array}{cc} 
\omega_0\\ 
\omega_1\\
\omega_2\\
\omega_3
\end{array}\right) = \mathbf{T} \cdot 
\left(\begin{array}{cc} 
v_x\\ 
v_y\\
\omega
\end{array}\right)
$$
, where $\omega_i$ are the wheel's angular velocities and $v_x$, $v_y$ and $\omega$ are Twist values. The matrix $\mathbf{T}$ can be calculated as follows:
$$
\mathbf{T} = \left(\begin{array}{ccc} 
kx_0 & ky_0 & komega_0\\ 
kx_1 & ky_1 & komega_1\\ 
kx_2 & ky_2 & komega_2\\ 
kx_3 & ky_3 & komega_3\\ 
\end{array}\right)
$$ for a four-wheeled robot.

 <script type="text/javascript" async src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-MML-AM_CHTML">
