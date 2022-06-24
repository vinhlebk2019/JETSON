import sys
import time
import serial
import struct
import logging
import argparse
from socket import socket, AF_INET, SOCK_DGRAM
import threading
import json

from support_class import TimerThread
from support_class import ServerThread
from support_class import ParseClass

import rospy
from std_msgs.msg import String
from aev_pkg.msg import ecu_feedback_msg
from aev_pkg.msg import gui_msg
from aev_pkg.msg import radar_msg
from aev_pkg.msg import driving_mode_msg

# This is old version, Function to create CAN message to transmit
def _setTransmitMsg(id, rtr, ext, len, buf):
    sendData = [0xAA, 0xAA, 0x78, 0x56, 0x34, 0x12, 0x11, 0x22, 0x33, 0x44, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x3F, 0x55, 0x55, 0xF0]
    idStr = "{:08x}".format(id)
    logging.debug("idStr={}".format(idStr))
    if (ext == 0):
        sendData[2] = int(idStr[6:8],16)
        sendData[3] = int(idStr[4:6],16) & 0x7
        sendData[4] = 0
        sendData[5] = 0
        logging.debug("id={:02x}{:02x}".format(sendData[2], sendData[3]))
    else:
        sendData[2] = int(idStr[6:8],16)
        sendData[3] = int(idStr[4:6],16)
        sendData[4] = int(idStr[2:4],16)
        sendData[5] = int(idStr[0:2],16) & 0x1F
        logging.debug("id={:02x}{:02x}".format(sendData[2], sendData[3]))
    for x in range(len):
        sendData[x+6] = buf[x]
    sendData[14] = len # Frame Data Length
    sendData[16] = ext # Standard/Extended frame
    sendData[17] = rtr # Data/Request frame
    sendData[18] = sum(sendData[2:18]) & 0xff
    #print("old:crc={:2x}".format(sendData[18]))
    #print("old:sendData={}".format(sendData))
    return sendData

USART_FRAMECTRL = 0xA5                                                  
USART_FRAMEHEAD = 0xAA
USART_FRAMETAIL = 0x55

def insertCtrl(buffer, ch):
    result = buffer
    #print("insertCtrl ch={:02x}".format(ch))
    if (ch == USART_FRAMECTRL or ch == USART_FRAMEHEAD or ch == USART_FRAMETAIL):
        result.append(USART_FRAMECTRL)
    return result

# Function to set the filter message
def setFilterMsg(filterIndex, filterId, filterMask, frameType, filterStatus):
    #sendData = [0xAA, 0xAA, 0xEx, 0xFE, 0xFF, 0x01, 0x11, 0x22, 0x33, 0x44, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x3F, 0x55, 0x55, 0xF0]
    #print("sendData={}".format(sendData))

    if (filterIndex > 15):
        logging.waring("filterIndex {} is too large.".format(filterIndex))
        return []

    if (frameType.lower() != "std" and frameType.lower() != "ext"):
        logging.waring("frameType {} is invalid.".format(frameType))
        return []

    if (filterStatus.lower() != "enable" and filterStatus.lower() != "disable"):
        logging.waring("filterStatus {} is invalid.".format(filterStatus))
        return []

    sendData = [0xAA, 0xAA]
    id = 0xE0 + filterIndex
    sendData.append(id)
    crc = id
    id = 0xFE
    crc = crc + id
    sendData.append(id)
    id = 0xFF
    crc = crc + id
    sendData.append(id)
    id = 0x01
    crc = crc + id
    sendData.append(id)

    idStr = "{:08x}".format(filterId)
    if (frameType.lower() == "std"):
        id = int(idStr[6:8],16)
        sendData = insertCtrl(sendData, id)
        sendData.append(id)
        crc = crc + id
        id = int(idStr[4:6],16) & 0x7
        sendData = insertCtrl(sendData, id)
        sendData.append(id)
        crc = crc + id
        sendData.append(0)
        id = 0 # Disable
        if (filterStatus.lower() == "enable"): id = 0x80 # Enable
        sendData.append(id)
        crc = crc + id
        logging.debug("id={:02x}{:02x}".format(sendData[2], sendData[3]))
    else:
        id = int(idStr[6:8],16)
        sendData = insertCtrl(sendData, id)
        sendData.append(id)
        crc = crc + id
        id = int(idStr[4:6],16)
        sendData = insertCtrl(sendData, id)
        sendData.append(id)
        crc = crc + id
        id = int(idStr[2:4],16)
        sendData = insertCtrl(sendData, id)
        sendData.append(id)
        crc = crc + id
        id = int(idStr[0:2],16) & 0x1F
        if (filterStatus.lower() == "enable"): id = 0x80 # Enable
        sendData = insertCtrl(sendData, id)
        sendData.append(id)
        crc = crc + id
        logging.debug("id={:02x}{:02x}".format(sendData[2], sendData[3]))

    filterMaskStr = "{:08x}".format(filterMask)
    mask = int(filterMaskStr[6:8],16)
    sendData = insertCtrl(sendData, mask)
    sendData.append(mask)
    crc = crc + mask
    mask = int(filterMaskStr[4:6],16)
    sendData = insertCtrl(sendData, mask)
    sendData.append(mask)
    crc = crc + mask
    mask = int(filterMaskStr[2:4],16)
    sendData = insertCtrl(sendData, mask)
    sendData.append(mask)
    crc = crc + mask
    mask = int(filterMaskStr[0:2],16) & 0x1F
    if (frameType.lower() == "ext"):
        mask = mask + 0x40 # Extended
    sendData = insertCtrl(sendData, mask)
    sendData.append(mask)
    crc = crc + mask

    len = 8
    sendData.append(len) # Frame Data Length
    crc = crc + len
    req = 0xFF
    sendData.append(req)
    crc = crc + req
    ext = 1
    sendData.append(ext) # Standard/Extended frame
    crc = crc + ext
    rtr = 0 # Set
    sendData.append(rtr) # Set/Read
    crc = crc + rtr
    crc = crc & 0xff
    logging.debug("crc={:2x}".format(crc))
    sendData = insertCtrl(sendData, crc)
    sendData.append(crc)
    sendData.append(0x55)
    sendData.append(0x55)
    logging.debug("sendData={}".format(sendData))
    return sendData

# Function to create CAN message to transmit
def setTransmitMsg(id, rtr, ext, len, buf):
    #sendData = [0xAA, 0xAA, 0x78, 0x56, 0x34, 0x12, 0x11, 0x22, 0x33, 0x44, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x3F, 0x55, 0x55, 0xF0]
    #print("sendData={}".format(sendData))

    sendData = [0xAA, 0xAA]
    idStr = "{:08x}".format(id)
    logging.debug("idStr={}".format(idStr))
    crc = 0

    if (ext == 0):
        id = int(idStr[6:8],16)
        sendData = insertCtrl(sendData, id)
        sendData.append(id)
        crc = crc + id
        id = int(idStr[4:6],16) & 0x7
        sendData = insertCtrl(sendData, id)
        sendData.append(id)
        crc = crc + id
        sendData.append(0)
        sendData.append(0)
        logging.debug("id={:02x}{:02x}".format(sendData[2], sendData[3]))
    else:
        id = int(idStr[6:8],16)
        sendData = insertCtrl(sendData, id)
        sendData.append(id)
        crc = crc + id
        id = int(idStr[4:6],16)
        sendData = insertCtrl(sendData, id)
        sendData.append(id)
        crc = crc + id
        id = int(idStr[2:4],16)
        sendData = insertCtrl(sendData, id)
        sendData.append(id)
        crc = crc + id
        id = int(idStr[0:2],16) & 0x1F
        sendData = insertCtrl(sendData, id)
        sendData.append(id)
        crc = crc + id
        logging.debug("id={:02x}{:02x}".format(sendData[2], sendData[3]))

    for x in range(len):
        sendData = insertCtrl(sendData, buf[x])
        sendData.append(buf[x])
        crc = crc + buf[x]
    if (len < 8):
        for x in range(8-len):
            sendData.append(0)

    sendData.append(len) # Frame Data Length
    crc = crc + len
    sendData.append(0)
    sendData.append(ext) # Standard/Extended frame
    crc = crc + ext
    sendData.append(rtr) # Data/Request frame
    crc = crc + rtr
    crc = crc & 0xff
    logging.debug("crc={:2x}".format(crc))
    sendData = insertCtrl(sendData, crc)
    sendData.append(crc)
    sendData.append(0x55)
    sendData.append(0x55)
    #sendData.append(0xF0)
    logging.debug("sendData={}".format(sendData))
    return sendData


# Function to set the transmit message to CAN Module
# https://qiita.com/mml/items/ccc66ecc46d8299b3346
def sendMsg( buf ):
      while True:
            if ser.out_waiting == 0:
                  break
      for b in buf:
            a = struct.pack( "B", b )
            ser.write(a)
      ser.flush()

# Function to init the id of the CAN Module         
def initId():
    data = [0xAA, 0xAA, 0xFF, 0xFE, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x08, 0xFF, 0x01, 0x00, 0x66, 0x55, 0x55]

    data[18]=sum(data[2:18]) & 0xFF
    #print("data[18]={:02x}".format(data[18]))
    sendMsg(data)


# Function to request the Info of module CAN
def readInfo(id):
    data = [0xAA, 0xAA, 0xE0, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x01, 0x01, 0x66, 0x55, 0x55]
    idStr = "{:08x}".format(id)
    #print("idStr={}".format(idStr))
    data[2] = int(idStr[6:8],16)
    data[3] = int(idStr[4:6],16)
    data[4] = int(idStr[2:4],16)
    data[5] = int(idStr[0:2],16)
    data[18]=sum(data[2:18]) & 0xFF
    #print("data={:02x}{:02x}{:02x}{:02x}".format(data[2], data[3], data[4], data[5]))
    sendMsg(data)


# Function to request the filter from module CAN
def readFilter(index):
     data = [0xAA, 0xAA, 0xE0, 0xFE, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x01, 0x01, 0x66, 0x55, 0x55]

     #print("index={}".format(index))
     data[2] = 0xE0 + index
     data[18]=sum(data[2:18]) & 0xFF
     #print("data4[18]={:2x}".format(data4[18]))
     sendMsg(data)


# Function to set baudrate of the CAN Module
def setSpeed(speed):
     logging.info("speed={}".format(speed))
     speed1000 = [0xAA, 0xAA, 0xD0, 0xFE, 0xFF, 0x01, 0x40, 0x42, 0x0F, 0x00, 0x01, 0x02, 0x00, 0x00, 0x04, 0xFF, 0x01, 0x00, 0x66, 0x55, 0x55]

     speed800 = [0xAA, 0xAA, 0xD0, 0xFE, 0xFF, 0x01, 0x00, 0x35, 0x0C, 0x00, 0x01, 0x02, 0x00, 0x00, 0x04, 0xFF, 0x01, 0x00, 0x16, 0x55, 0x55]

     speed500 = [0xAA, 0xAA, 0xD0, 0xFE, 0xFF, 0x01, 0x20, 0xA1, 0x07, 0x00, 0x01, 0x02, 0x00, 0x00, 0x04, 0xFF, 0x01, 0x00, 0x9D, 0x55, 0x55]

     speed400 = [0xAA, 0xAA, 0xD0, 0xFE, 0xFF, 0x01, 0x80, 0x1A, 0x06, 0x00, 0x01, 0x02, 0x00, 0x00, 0x04, 0xFF, 0x01, 0x00, 0x75, 0x55, 0x55]

     speed250 = [0xAA, 0xAA, 0xD0, 0xFE, 0xFF, 0x01, 0x90, 0xD0, 0x03, 0x00, 0x01, 0x02, 0x00, 0x00, 0x04, 0xFF, 0x01, 0x00, 0x38, 0x55, 0x55]

     speed125 = [0xAA, 0xAA, 0xD0, 0xFE, 0xFF, 0x01, 0x48, 0xE8, 0x01, 0x00, 0x01, 0x02, 0x00, 0x00, 0x04, 0xFF, 0x01, 0x00, 0x06, 0x55, 0x55]

     speed100 = [0xAA, 0xAA, 0xD0, 0xFE, 0xFF, 0x01, 0xA0, 0x86, 0x01, 0x00, 0x01, 0x02, 0x00, 0x00, 0x04, 0xFF, 0x01, 0x00, 0xFC, 0x55, 0x55]


     if (speed == 1000000):
         sendMsg(speed1000)
         return True
     elif (speed == 800000):
         sendMsg(speed800)
         return True
     elif (speed == 500000):
         sendMsg(speed500)
         return True
     elif (speed == 400000):
         sendMsg(speed400)
         return True
     elif (speed == 250000):
         sendMsg(speed250)
         return True
     elif (speed == 125000):
         sendMsg(speed125)
         return True
     elif (speed == 100000):
         sendMsg(speed100)
         return True
     else:
         return False


# FUnction loggingFrame
def loggingFrame(header, buffer):
     message = header
     message = message + "["
     for x in range(len(buffer)):
        message = message + "{:02x}".format(buffer[x]).upper()
        if (x != len(buffer)-1): message = message + " "
     message = message + "]"
     logging.info(message)


#################################################################################################################################
# Function to print the received frame
def printFrame(buffer):
     logging.debug("buffer={}".format(buffer))
     if (buffer[16] == 0): # Standard frame
         receiveId = (buffer[3] << 8) + buffer[2]
     else: # Extended frame
         receiveId = (buffer[5] << 24) + (buffer[4] << 16) + (buffer[3] << 8) + buffer[2]
     logging.debug("receiveId=0x{:X}".format(receiveId))

     if (buffer[15] == 0xFF):
         if (receiveId == 0x01fffed0):
             message = "BAUDRATE ID: 0x"
         elif (receiveId == 0x01fffff0):
             message = "CPUINFO0 ID: 0x"
         elif (receiveId == 0x01fffff1):
             message = "CPUINFO1 ID: 0x"
         elif (receiveId == 0x01ffffe0):
             message = "VERSION  ID: 0x"
         elif (receiveId == 0x01fffeff):
             message = "INIT     ID: 0x"
         elif (receiveId == 0x01fffeb0):
             message = "ABOM     ID: 0x"
         elif (receiveId == 0x01fffea0):
             message = "ART      ID: 0x"
         elif ((receiveId & 0x01fffff0) == 0x01fffee0):
             index = receiveId & 0xf
             message = "FILTER{:02d} ID: 0x".format(index)
         message = message + "{:02x}".format(buffer[5]).upper()
         message = message + "{:02x}".format(buffer[4]).upper()
         message = message + "{:02x}".format(buffer[3]).upper()
         message = message + "{:02x}".format(buffer[2]).upper()
         message = message + "  DLC: "
         message = message + "{:01n}".format(buffer[14])
         message = message + "  Data:"
         for x in range(buffer[14]):
             #print ("x={}".format(x))
             message = message + " 0x"
             message = message + "{:02x}".format(buffer[x+6]).upper()
     else:
         if (buffer[16] == 0):
             message = "Standard ID: 0x"
             message = message + "{:01x}".format(buffer[3]).upper()
             message = message + "{:02x}".format(buffer[2]).upper()
             message = message + "       DLC: "
         else:
             message = "Extended ID: 0x"
             message = message + "{:02x}".format(buffer[5]).upper()
             message = message + "{:02x}".format(buffer[4]).upper()
             message = message + "{:02x}".format(buffer[3]).upper()
             message = message + "{:02x}".format(buffer[2]).upper()
             message = message + "  DLC: "
         message = message + "{:01n}".format(buffer[14])
         message = message + "  Data:"
         if (buffer[17] == 0):
             for x in range(buffer[14]):
                 #print ("x={}".format(x))
                 message = message + " 0x"
                 message = message + "{:02x}".format(buffer[x+6]).upper()
         else:
             message = message + " REMOTE REQUEST FRAME"
     print(message)
     return receiveId

###################################################################################    
def unsigned8Tosigned8(val):
    if val > 127:
        return (256-val) * (-1)
    else:
        return val

def signed8ToUsigned8(val):
    if (val < 0):
        return (256+val)
    else:
        return val

control_targetSpeed = 0
control_drivingMode = 1
control_turnSignal = 0
control_horn = 0
control_frontLight = 0

def callback_gui_msg(data):
    global control_targetSpeed
    global control_turnSignal
    global control_horn
    #rospy.loginfo("Received Gui data %d" % data.msg_counter)
    #print("Comment: Received speedSetpoint: {}".format(data.speedSetpoint))
    control_targetSpeed = data.speedSetpoint
    control_turnSignal = data.turnSignal
    control_horn = data.horn

def callback_driving_mode_msg(data):
    global control_drivingMode
    #rospy.loginfo("Received Driving Mode data %d" % data.msg_counter)
    control_drivingMode = data.drivingMode;
    #print("Comment: Received Driving Mode data: {}".format(control_drivingMode))

def callback_radar_msg(data):
    global control_targetSpeed
    pass
    #control_targetSpeed = int(data.ttcSpeed)
    #print (control_targetSpeed)

ecu_feedback_data = ecu_feedback_msg()
def ecu_feedback_msg_pub():
    if not rospy.is_shutdown():
        #rospy.loginfo("Publish ecu_feedback %d" % ecu_feedback_data.msg_counter)
        #print("Comment: Publish ecu_feedback {}".format(ecu_feedback_data.msg_counter))
        pub_ecu_feedback.publish(ecu_feedback_data)

# Function to handle the received frame
def handleFrame(buffer):
    received_id = 0;
    logging.debug("buffer={}".format(buffer))
    if (buffer[16] == 0): # Standard frame
        receiveId = (buffer[3] << 8) + buffer[2]
    else: # Extended frame
        receiveId = (buffer[5] << 24) + (buffer[4] << 16) + (buffer[3] << 8) + buffer[2]
    logging.debug("receiveId=0x{:X}".format(receiveId))

    if (buffer[15] == 0xFF):
        # nothing to do
        message = ""
    else:
        if (buffer[16] == 0):
            received_id = (buffer[3] << 8) | buffer[2]
        else:
            received_id = (buffer[5] << 24) | (buffer[4] << 16) | (buffer[3] << 8) | buffer[2]

        if ((received_id == 0x0202) and (buffer[17] == 0) and (buffer[14] == 8)):
            ecu_feedback_data.msg_counter = ecu_feedback_data.msg_counter + 1
            ecu_feedback_data.feedbackSpeed_b1 = buffer[6]
            ecu_feedback_data.feedbackSpeed_b2 = buffer[7]
            ecu_feedback_data.feedbackSpeed_b3 = buffer[8]
            ecu_feedback_data.feedbackSpeed_b4 = buffer[9]
            ecu_feedback_data.acceleratorLevel = buffer[10]
            ecu_feedback_data.acceleratorSwitch = (buffer[11] >> 0) & 0x01
            ecu_feedback_data.brakeSwitch = (buffer[11] >> 1) & 0x01
            ecu_feedback_data.movingDirection = (buffer[11] >> 2) & 0x01
            ecu_feedback_data.turnSignal = (buffer[11] >> 3) & 0x03
            ecu_feedback_data.horn = (buffer[11] >> 5) & 0x01
            ecu_feedback_data.frontLight = (buffer[11] >> 6) & 0x01

            #print("Comment: Publish feedbackSpeed")

    #print("message counter %d" % ecu_feedback_data.msg_counter)
    ecu_feedback_msg_pub()
    return receiveId

# In ROS, nodes are uniquely named. If two nodes with the same
# name are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'ooo' node so that multiple listeners can
# run simultaneously.
rospy.init_node('CAN_Node', anonymous=True)
pub_ecu_feedback = rospy.Publisher('ECU_Feedback_Data', ecu_feedback_msg, queue_size=10)

rospy.Subscriber("GUI_Data", gui_msg, callback_gui_msg)
rospy.Subscriber("DrivingMode_Data", driving_mode_msg, callback_driving_mode_msg)
rospy.Subscriber("Radar_Data", radar_msg, callback_radar_msg)

####################################################################################


format="%(asctime)s [%(filename)s:%(lineno)d] %(levelname)-8s %(message)s"
#logging.basicConfig(level=logging.DEBUG, format=format)
#logging.basicConfig(level=logging.WARNING, format=format)

parser = argparse.ArgumentParser()
parser.add_argument("-p", "--port", help="open port")
parser.add_argument("-s", "--speed", help="can bit rate", type=int)
parser.add_argument("-u", "--udp", help="UDP receive port", type=int)
parser.add_argument("-l", "--log", dest="logLevel", choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'], help="Set the logging level")

args = parser.parse_args()
device = "/dev/ttyUSB0"
speed = 1000000
udpPort = 8200

if args.port:
    print("args.port={}".format(args.port))
    device = args.port
if args.speed:
    print("args.speed={}".format(args.speed))
    speed = args.speed
if args.udp:
    print("args.udp={}".format(args.udp))
    udpPort = args.udp
if args.logLevel:
    level=getattr(logging, args.logLevel)
    print("logLevel set to {}".format(level))
    #logging.basicConfig(level=getattr(logging, args.logLevel))
    logging.basicConfig(level=getattr(logging, args.logLevel), format=format)
else:
    print("logLevel set to {}".format(logging.WARNING))
    logging.basicConfig(level=logging.WARNING, format=format)

if (speed != 1000000 and speed != 500000 and speed != 400000 and speed != 250000 and speed != 125000 and speed != 100000):
    logging.error("This speed {} is not supported.".format(speed))
    sys.exit()

ser = serial.Serial(
      port = device,
      baudrate = 115200,
      #parity = serial.PARITY_NONE,
      #bytesize = serial.EIGHTBITS,
      #stopbits = serial.STOPBITS_ONE,
      #timeout = None,
      #xonxoff = 0,
      #rtscts = 0,
      )


## Start Timer thread (Timer is not use)
timer = TimerThread(ACTIVE=True, INTERVAL=0.2)
timer.setDaemon(True)
timer.start()

## Start UDP Receive hread
udp = ServerThread(PORT=udpPort)
udp.setDaemon(True)
udp.start()

buffer = []
parse = ParseClass()
receiveId = 0x01FFFFFF

while True:
    if (receiveId == 0x01FFFFFF):
        initId()
        receiveId = 0
    elif (receiveId == 0x01FFFEFF):
        setSpeed(speed)
        receiveId = 0
    elif (receiveId == 0x01FFFED0):
        readInfo(0x01FFFFE0)
        receiveId = 0
    elif (receiveId == 0x01FFFFE0):
        readInfo(0x01FFFFF0)
        receiveId = 0
    elif (receiveId == 0x01FFFFF0):
        readInfo(0x01FFFFF1)
        receiveId = 0
    elif (receiveId == 0x01FFFFF1):
        readInfo(0x01FFFEB0)
        receiveId = 0
    elif (receiveId == 0x01FFFEB0):
        readInfo(0x01FFFEA0)
        receiveId = 0
    elif (receiveId == 0x01FFFEA0):
        readFilter(0)
        receiveId = 0
    elif ((receiveId & 0x01FFFFF0) == 0x01FFFEE0):
        index = receiveId & 0xf
        #print("index={}".format(index))
        if (index != 15):
            readFilter(index+1)
            receiveId = 0
        else:
            receiveId = 0xFFFFFFFF
        
    if timer.timerFlag is True:
        global control_targetSpeed
        global control_turnSignal
        global control_drivingMode
        global control_frontLight
        global control_horn
        timer.timerFlag = False
        frameId = 0x201
        frameRequest = 0
        frameType = 0 # Standard frame
        frameLength = 8
        # Check valid targetSpeed before send request
        if (control_targetSpeed < -25):
            control_targetSpeed = -25
        if (control_targetSpeed > 25):
            control_targetSpeed = 25

        #print("Prepare: send control data to ECU: {}".format(control_drivingMode))
        # Check valid targetSpeed before send request
        if (control_drivingMode < 1):
            control_drivingMode = 1
        if (control_drivingMode > 5):
            control_drivingMode = 1

        # Check valid turnSignal before send request
        if (control_turnSignal < 0):
            control_turnSignal = 0
        if (control_turnSignal > 2):
            control_turnSignal = 0

        # Check valid horn signal before send request
        if (control_horn < 0):
            control_horn = 0
        if (control_horn > 1):
            control_horn = 0

        # Check valid frontLight signal before send request
        if (control_frontLight < 0):
            control_frontLight = 0
        if (control_frontLight > 1):
            control_frontLight = 0

        byte_2 = ((control_frontLight << 3) & 0x08) | ((control_horn << 2) & 0x04) | ((control_turnSignal << 0) & 0x03)

        frameData = [signed8ToUsigned8(control_targetSpeed), control_drivingMode, byte_2, 
                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
        sendData = _setTransmitMsg(frameId, frameRequest, frameType, frameLength, frameData)
        sendMsg(sendData)
        #print("Comment: send control data to ECU: {}".format(control_drivingMode))
        

    if udp.interrupt is True:
        logging.info("udp interrupt request={}".format(udp.request))
        if (udp.request == "transmit"):
            logging.debug("id={} type={}".format(udp.id, udp.type))
            frameId = int(udp.id, 16)
            logging.debug("frameId={:x}".format(frameId))
            if (udp.type == "stddata"):
                frameRequest = 0
                frameType = 0
                frameLength = len(udp.data)
            if (udp.type == "extdata"):
                frameRequest = 0
                frameType = 1
                frameLength = len(udp.data)
            if (udp.type == "stdremote"):
                frameRequest = 1
                frameType = 0
                frameLength = 0
            if (udp.type == "extremote"):
                frameRequest = 1
                frameType = 1
                frameLength = 0
            frameData = []
            for x in range(frameLength):
                logging.debug("udp.data={}".format(udp.data[x]))
                frameData.append(udp.data[x])
            logging.debug("frameData={}".format(frameData))
            sendData = setTransmitMsg(frameId, frameRequest, frameType, frameLength, frameData)
            sendMsg(sendData)
            logging.info("Transmit={}".format(sendData))
            loggingFrame("Transmit=", sendData)
            udp.interrupt = False

        if (udp.request == "filter"):
            logging.info("index={} id={} mask={} type={} status={}".format(udp.index, udp.id, udp.mask, udp.type, udp.status))
            filterIndex = int(udp.index)
            filterId = int(udp.id,16)
            filterMask = int(udp.mask,16)
            frameType = udp.type
            filterStatus = udp.status
            logging.debug("filterIndex={} filterId=0x{:x} filterMask=0x{:x} frameType={} filterStatus={}".format(filterIndex, filterId, filterMask, frameType, filterStatus))
            sendData = setFilterMsg(filterIndex, filterId, filterMask, frameType, filterStatus)
            if (len(sendData) > 0):
                sendMsg(sendData)
                logging.info("Mask={}".format(sendData))
                loggingFrame("Mask=", sendData)
            udp.interrupt = False

    if ser.in_waiting > 0:
        recv_data = ser.read(1)
        a = struct.unpack_from("B",recv_data ,0)
        b = a[0]

        buffer = parse.parseData(b)
        if (len(buffer) > 0):
            #logging.info("Receive={}".format(buffer))
            #loggingFrame("Receive=", buffer)
            #print("receiveId={:x}".format(receiveId))
            if (receiveId == 0xFFFFFFFF):
                handleFrame(buffer)
            else:
                receiveId = handleFrame(buffer)

