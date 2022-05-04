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

<img src="https://latex.codecogs.com/svg.image?\begin{pmatrix}&space;\omega_0&space;\\&space;\omega_1&space;\\&space;\omega_2&space;\\&space;\omega_3\end{pmatrix}&space;=&space;\mathbf{T}\begin{pmatrix}v_x&space;\\v_y&space;\\\omega\end{pmatrix}&space;" title="https://latex.codecogs.com/svg.image?\begin{pmatrix} \omega_0 \\ \omega_1 \\ \omega_2 \\ \omega_3\end{pmatrix} = \mathbf{T}\begin{pmatrix}v_x \\v_y \\\omega\end{pmatrix} " />

, where &omega;<sub>i</sub> are the wheel's angular velocities and v<sub>x</sub>, v<sub>y</sub> and &omega; are Twist values. The matrix **T** can be calculated as follows:
<img src="https://latex.codecogs.com/svg.image?\mathbf{T}&space;=&space;\begin{pmatrix}&space;kx_0&space;&&space;ky_0&space;&&space;k\omega{}_0\\&space;kx_1&space;&&space;ky_1&space;&&space;k\omega{}_1\\&space;kx_2&space;&&space;ky_2&space;&&space;k\omega{}_2\\&space;kx_3&space;&&space;ky_3&space;&&space;k\omega{}_3\end{pmatrix}" title="https://latex.codecogs.com/svg.image?\mathbf{T} = \begin{pmatrix} kx_0 & ky_0 & k\omega{}_0\\ kx_1 & ky_1 & k\omega{}_1\\ kx_2 & ky_2 & k\omega{}_2\\ kx_3 & ky_3 & k\omega{}_3\end{pmatrix}" />

for a four-wheeled robot. kx<sub>i</sub>, ky<sub>i</sub> and k&omega;<sub>i</sub> are the translation parameters from one space to the other. These parameters include the wheel radius r as well as the robot length l<sub>x</sub> and robot width l<sub>y</sub>.

### Example for a Differential drive
<img src="https://latex.codecogs.com/svg.image?\mathbf{T}&space;=&space;\begin{pmatrix}&space;\frac{1}{r}&space;&&space;0&space;&&space;-\frac{l_y}{2&space;\cdot&space;r}\\&space;-\frac{1}{r}&space;&&space;0&space;&&space;-\frac{l_y}{2&space;\cdot&space;r}\\\end{pmatrix}" title="https://latex.codecogs.com/svg.image?\mathbf{T} = \begin{pmatrix} \frac{1}{r} & 0 & -\frac{l_y}{2 \cdot r}\\ -\frac{1}{r} & 0 & -\frac{l_y}{2 \cdot r}\\\end{pmatrix}" />

### Example for a Mecanum drive
<img src="https://latex.codecogs.com/svg.image?\mathbf{T}&space;=&space;\frac{1}{r}\begin{pmatrix}&space;&space;1&space;&&space;-1&space;&&space;-\frac{l_x&plus;l_y}{2}\\&space;-1&space;&&space;-1&space;&&space;-\frac{l_x&plus;l_y}{2}\\&space;&space;1&space;&&space;&space;1&space;&&space;-\frac{l_x&plus;l_y}{2}\\&space;-1&space;&&space;&space;1&space;&&space;-\frac{l_x&plus;l_y}{2}\end{pmatrix}" title="https://latex.codecogs.com/svg.image?\mathbf{T} = \frac{1}{r}\begin{pmatrix} 1 & -1 & -\frac{l_x+l_y}{2}\\ -1 & -1 & -\frac{l_x+l_y}{2}\\ 1 & 1 & -\frac{l_x+l_y}{2}\\ -1 & 1 & -\frac{l_x+l_y}{2}\end{pmatrix}" />
