CC=g++
CFLAGS=-Wall -std=c++11 -pthread -g
INC=..
INC_CAN=../../interface/can

SRC_CAN=../../interface/can/SocketCAN.cpp ../../interface/can/SocketCANObserver.cpp ../MotorControllerCAN.cpp
HDR_CAN=../../interface/can/SocketCAN.h ../../interface/can/SocketCANObserver.h

SRC_CAN_MOTOR_TEST=can_motor_test.cpp
HDR_CAN_MOTOR=../MotorControllerCAN.h

EXECUTABLE_CAN_MOTOR_TEST=$(SRC_CAN_MOTOR_TEST:.cpp=)

all: $(EXECUTABLE_CAN_MOTOR_TEST)

$(EXECUTABLE_CAN_MOTOR_TEST): $(SRC_CAN_MOTOR_TEST) $(SRC_CAN) $(HDR_CAN) $(HDR_CAN_MOTOR)
	$(CC) $(CFLAGS) -I $(INC) -I $(INC_CAN) $(SRC_CAN_MOTOR_TEST) $(SRC_CAN) -o $@

clean:
	rm $(EXECUTABLE_CAN_MOTOR_TEST)
