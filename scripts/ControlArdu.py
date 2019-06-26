#!/usr/bin/env python
import rospy
import serial
import constants

#import pygame
import os
import time
from std_msgs.msg import String, Int16, Float32, Int16MultiArray
#from ppsk.msg import Control
class order:
    def __init__(self):
        self.order = []
        self.OrderProperties=[]
        self.SerialMessage = b""
        self.Change = 0
    
    def ChangeOrder(self,newOrder):
        self.order.append(newOrder)
        self.Change += 1 

    def ChangeOrderProperties(self,newProperties):
        self.OrderProperties = newProperties
        self.Change += 1
    
    def ChangeOrderAndProperties(self,newOrder,newProperties):
        self.OrderProperties.append(newProperties)
        self.order.append(newProperties) = newOrder
        self.Change += 1

    def ReadOrder(self):
        self.Change -= 1
        return (self.order[0],self.OrderProperties[0])

    def CheckChange(self):
        return self.Change 

    def ReadSerialMessage(self):
        return self.SerialMessage

    def DecideIfItIsARequest(self):
        if self.order[0] == (constants.CONST_READ_STATE or constants.CONST_READ_DISTANCE_FRONT or constants.CONST_READ_DISTANCE_REAR or constants.CONST_DIRECTION or constants.CONST_READ_FLOOR or constants.CONST_READ_LIMIT_SWITCHES):
            pass

    def GenerateSerialMessage(self):
        self.Change -= 1 
        if self.order.pop() == constants.CONST_START:
            rospy.loginfo("Generuje start")
            self.SerialMessage = bytearray([constants.CONST_SERIAL_RPI_START]) 
            self.OrderProperties.pop()
            return True
        elif self.order.pop() == constants.CONST_DIRECTION:
            self.SerialMessage = bytearray([constants.CONST_SERIAL_RPI_DIRECTION, self.OrderProperties.pop()])
            return True
        elif self.order.pop() == constants.CONST_SPEED:
            self.SerialMessage = bytearray([constants.CONST_SERIAL_RPI_SPEED, self.OrderProperties.pop()])
            return True
        elif self.order.pop() == constants.CONST_INITIALIZE:
            self.SerialMessage = bytearray([constants.CONST_SERIAL_RPI_INITIALIZED])
            self.OrderProperties.pop()
            return True
        elif self.order.pop() == constants.CONST_STOP:
            rospy.loginfo("Generuje stop")
            self.SerialMessage = bytearray([constants.CONST_SERIAL_RPI_STOP])
            self.OrderProperties.pop()
            return True
        else:
            return False     
class state:
    def __init__(self):
        self.limitSwitches = 15
        self.frontDistance = 0
        self.rearDistance = 0
        self.state = constants.CONST_STATE_MOVEMENT
        self.position = 0
        self.floorSensors = 0
    
    def ChangeState(self,newState):
        self.state = newState
    
    def ChangeFrontDistance(self, newDistance):
        self.frontDistance = newDistance
    
    def ChangeRearDistance(self,newDistance):
        self.rearDistance = newDistance
    
    def ChangeLimitSwitches(self,newLimitSwitches):
        self.limitSwitches = newLimitSwitches

    def ChangeFloorSensors(self,newSensors):
        self.floorSensors = newSensors
    
    def GetLimitSwitches(self):
        return self.limitSwitches
    
ser = None
orderObject = None
def receiveOrders(data):
    global ser
    global orderObject

    recieved = data.data
    orderNumber = recieved//1000
    orderProperty = recieved - (orderNumber*1000)
    orderObject.ChangeOrderAndProperties(orderNumber,orderProperty)
    rospy.loginfo("Dostalem-callback")
    rospy.loginfo(recieved)
    # orderObject.ChangeOrder(data.Order)
    # orderObject.ChangeOrderProperties(data.OrderProperty)

    #if data.data =  constants.   

def talker():
    global ser
    global orderObject

    orderObject = order()
    robotState = state()
    rospy.loginfo("Otwieramy port!")
    ser = serial.Serial()
    ser.port = '/dev/ttyACM0'
    ser.baudrate = 115200
    ser.setDTR(0)
    ser.timeout = 1
    #bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=0.5,xonxoff=0,rtscts=0,dsrdtr=0)
    if (ser.isOpen() == False):
        ser.open()
    ser.flushInput()
    #ser.baudrate = 115200
    #ser.port = "/dev/ttyACM0"
    #ser.setDTR(0)
    #ser.open()

    ser.reset_input_buffer()
    ser.reset_output_buffer()
    #ser = serial.Serial('/dev/ttyACM0',115200)
    #ser.flushInput()
    rospy.loginfo("Otwarlismy port!")
    pub = rospy.Publisher('RaspberryControlWriter',String,queue_size=10)
    rospy.init_node('talker',anonymous=True)
    rate = rospy.Rate(20) #10Hz
    #pub.publish("JEDZIEMY!")
    rospy.loginfo("Jedziemy!")
    rospy.Subscriber("RaspberryControlReader", Int16, receiveOrders)
    ser.write(bytearray([constants.CONST_SERIAL_RPI_INITIALIZED]))
    time.sleep(1)
    bufferSerial = b""
    while not rospy.is_shutdown():
        try:
            if orderObject.CheckChange() == True:
                if orderObject.GenerateSerialMessage() == True:
                    rospy.loginfo("Wrzucam na serial")
                    ser.write(orderObject.ReadSerialMessage())

            wait = ser.in_waiting
            if wait > 0:
                bufferTemp = ser.read(wait)
                bufferSerial = bufferSerial + bufferTemp 
            while (len(bufferSerial) > 0):
                temp=bufferSerial[0]
                bufferSerial = bufferSerial[1:]
                rospy.loginfo(ord(temp))
                if not temp:
                    function = 0
                else:            
                    function = ord(temp) 
                #rospy.loginfo(function)
                if function == constants.CONST_STATE_MOVEMENT:
                    pub.publish("STATE MOVEMENT")
                    rospy.loginfo("MOV")
                    robotState.ChangeState(constants.CONST_STATE_MOVEMENT)
                    os.system('mpg321 /home/pi/Jaguar.mp3 &')
                elif function == constants.CONST_STATE_OBSTACLE:
                    pub.publish("STATE OBSTACLE")
                    rospy.loginfo("OBST")
                    robotState.ChangeState(constants.CONST_STATE_OBSTACLE)
                    if(robotState.GetLimitSwitches() > 0):
                        os.system('mpg321 /home/pi/Torture.mp3 &')
                elif function == constants.CONST_STATE_STAIRS:
                    pub.publish("STATE STAIRS")
                    rospy.loginfo("STAIRS")
                    robotState.ChangeState(constants.CONST_STATE_STAIRS)
                    os.system('mpg321 /home/pi/Fall.mp3 &')
                elif function == constants.CONST_STATE_STOP:
                    pub.publish("STATE STOP")
                    rospy.loginfo("STOP")
                    robotState.ChangeState(constants.CONST_STATE_STOP)
                elif function == constants.CONST_FLOOR:
                    while len(bufferSerial) == 0:
                        bufferTemp = ser.read(ser.in_waiting)
                        bufferSerial = bufferSerial + bufferTemp
                    floorSensors = ord(bufferSerial[0])
                    bufferSerial = bufferSerial[1:]
                    robotState.ChangeFloorSensors(floorSensors)
                    rospy.loginfo(floorSensors)
                elif function == constants.CONST_OBSTACLE_SONAR_BACK:
                    while len(bufferSerial) == 0:
                        bufferTemp = ser.read(ser.in_waiting)
                        bufferSerial = bufferSerial + bufferTemp
                    backSonar = ord(bufferSerial[0])
                    bufferSerial = bufferSerial[1:]
                    robotState.ChangeRearDistance(backSonar)
                    rospy.loginfo(backSonar)
                elif function == constants.CONST_OBSTACLE_SONAR_FRONT:
                    while len(bufferSerial) == 0:
                        bufferTemp = ser.read(ser.in_waiting)
                        bufferSerial = bufferSerial + bufferTemp
                    frontSonar = ord(bufferSerial[0])
                    bufferSerial = bufferSerial[1:]
                    robotState.ChangeFrontDistance(frontSonar)
                    rospy.loginfo(frontSonar)
                elif function == constants.CONST_OBSTACLE_SWITCHES:
                    while len(bufferSerial) == 0:
                        bufferTemp = ser.read(ser.in_waiting)
                        bufferSerial = bufferSerial + bufferTemp
                    limitSwitches = ord(bufferSerial[0])
                    bufferSerial = bufferSerial[1:]
                    robotState.ChangeLimitSwitches(limitSwitches)
                    rospy.loginfo(limitSwitches)
                rate.sleep()
        except rospy.ROSInterruptException:
            ser.close()
            break
    ser.close() 


if __name__ == '__main__':
    #try:
        talker()
    #except rospy.ROSInterruptException:
    #    pass