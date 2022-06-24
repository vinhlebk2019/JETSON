import time
import threading
import logging
from socket import socket, AF_INET, SOCK_DGRAM

# Timer Thread
class TimerThread(threading.Thread):
    def __init__(self, ACTIVE, INTERVAL):
        threading.Thread.__init__(self)
        self.timerFlag = False
        self.interval = INTERVAL
        self.active = ACTIVE

    def run(self):
        while True:
            #print("Timer={} active={}".format(time.time(), self.active))
            if self.active == True: self.timerFlag = True
            time.sleep(self.interval)

# UDP Receive Thread
class ServerThread(threading.Thread):
    def __init__(self, PORT):
        threading.Thread.__init__(self)
        # line information
        self.HOST = ""
        self.PORT = PORT
        self.BUFSIZE = 1024
        self.ADDR = ("", self.PORT)

        # bind
        self.udpServSock = socket(AF_INET, SOCK_DGRAM)
        self.udpServSock.bind(self.ADDR)      
        self.interrupt = False

    def run(self):
        while True:
            try:
                #packet, self.addr = self.udpServSock.recvfrom(self.BUFSIZE) # Receive Data
                packet, addr = self.udpServSock.recvfrom(self.BUFSIZE) # Receive Data
                logging.debug("addr={}".format(addr))
                logging.debug("recvfrom packet={}".format(packet))
                self.packet = packet.decode()                
                json_dict = json.loads(self.packet) # If parsing fails, go to exception
                logging.debug("json_dict={}".format(json_dict))
                self.addrFrom = addr[0]
                self.portFrom = addr[1]
                logging.debug("self.addrFrom={} self.portFrom={}".format(self.addrFrom, self.portFrom))
                self.request = json_dict["request"].lower()
                if (self.request == "transmit"):
                    self.id = json_dict["id"]
                    self.type = json_dict["type"].lower()
                    logging.debug("self.request={} self.id={} self.type={}".format(self.request, self.id, self.type))
                    if (self.type == "stddata"):
                        self.data = json_dict["data"]
                        logging.debug("self.data={} len={}".format(self.data, len(self.data)), self.type)
                        self.interrupt = True
                    if (self.type == "extdata"):
                        self.data = json_dict["data"]
                        logging.debug("self.data={} len={}".format(self.data, len(self.data)))
                        self.interrupt = True
                    if (self.type == "stdremote"): self.interrupt = True
                    if (self.type == "extremote"): self.interrupt = True
                if (self.request == "filter"):
                    self.index = json_dict["index"]
                    self.id = json_dict["id"]
                    self.mask = json_dict["mask"]
                    self.type = json_dict["type"]
                    self.status = json_dict["status"]
                    logging.info("request={} index={} id={} mask={} type={} status{}".format(self.request, self.index, self.id, self.mask, self.type, self.status))
                    self.interrupt = True
            except:
                logging.error("json parse fail {}".format(self.packet))
                pass

class ParseClass(object):
    def __init__(self):
        #print("__init__")
        self.buffer = []
        self.status = 0

    def parseData(self, ch):
        logging.debug("ch=0x{:02x} {} status={} len={}".format(ch, ch, self.status, len(self.buffer)))
        if (self.status == 0):
            #print("self.status == 0")
            if (ch == 0xAA):
                self.status = 1
                self.buffer = []
                self.crc = 0
                self.buffer.append(ch)
                #print("self.buffer={}".format(self.buffer))
            return []

        elif (self.status == 1):
            #print("self.status == 1")
            if (ch == 0xAA):
                self.status = 2
                self.buffer.append(ch)
                #print("self.buffer={}".format(self.buffer))
            else:
                self.buffer = []
                self.status = 0
            return []

        elif (self.status == 2):
            #print("self.status == 2")
            #print("self.buffer={}".format(self.buffer))
            if (len(self.buffer) == 18):
                if (ch == 0xA5): # FrameCtrl,Next character is true CRC.
                    self.status = 3
                else:
                    self.crc = self.crc & 0xff
                    #print("self.crc={:x} ch={:x}".format(self.crc,ch))
                    if (self.crc != ch):
                        logging.warning("Invalid CRC(status=2) {:02x} {:02x} {}".format(self.crc, ch, self.buffer))
                        self.status = 0
                    else:
                        self.status = 8
                        self.buffer.append(ch)
            else:
                if (ch == 0xA5): # FrameCtrl,Skip this character
                    self.status = 4
                else:
                    self.crc = self.crc + ch
                    self.buffer.append(ch)
            return []

        elif (self.status == 3):
            #print("self.status == 3")
            self.crc = self.crc & 0xff
            #print("self.crc={:x} ch={:x}".format(self.crc,ch))
            if (self.crc != ch):
                logging.warning("Invalid CRC(status=3) {:02x} {:02x} {}".format(self.crc, ch, self.buffer))
                self.status = 0
            else:
                self.status = 8
                self.buffer.append(ch)
            return []

        elif (self.status == 4):
            #print("self.status == 4")
            self.crc = self.crc + ch
            self.buffer.append(ch)
            self.status = 2
            return []

        elif (self.status == 8):
            #print("self.status == 8")
            if (ch == 0x55):
                self.buffer.append(ch)
                self.status = 9
            else:
                logging.warning("invalid Packet (status=4)")
                self.status = 0
            return []

        elif (self.status == 9):
            #print("self.status == 9")
            if (ch == 0x55):
                self.buffer.append(ch)
                self.status = 9
                if (len(self.buffer) == 21):
                    logging.debug("self.buffer={}".format(self.buffer))
                    self.status = 0
                    return self.buffer
                else:
                    logging.warning("invalid Packet (status=9)")
                    self.status = 0
            else:
                logging.warning("invalid Packet (status=3)")
                self.status = 0
            return []
           