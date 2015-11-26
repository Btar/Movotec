#!/usr/bin/python
import sys, struct, serial, binascii, time

def wait_for_ack():
   ddata = ""
   spam = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
      spam += ddata
	  
   print "received: ", binascii.hexlify(spam)
   return

if len(sys.argv) < 2:
   print "no device specified"
   print "You need to specify the serial port of the device you wish to connect to"
   print "example:"
   print "   aAccel5Hz.py Com12"
   print "or"
   print "   aAccel5Hz.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 460800)
   ser.flushInput()
   time.sleep(2)   
   print "port opening, done."
   ser.write(struct.pack('B', 0x08))
   wait_for_ack()
   print "reset to default command sending, done."
   ser.write(struct.pack('BB', 0x12, 0x03))
   wait_for_ack()
   print "set sg data structure, done."
   ser.write(struct.pack('BB', 0x0c, 0x02))
   wait_for_ack()
   print "set accel data structure, done."
   ser.write(struct.pack('BB', 0x0f, 0x02))
   wait_for_ack()
   print "set accel data structure, done."
   ser.write(struct.pack('B', 0x27))
   wait_for_ack()
   print "start command sending, done."

# read incoming data
   ddata = ""
   numbytes = 0
   framesize = 27 # 6byte emg, 12byte accel, 6byte gyro, 8byte strain gauge, 3 byte status+ts
   found0 = 0                
   #print "Packet Type,Timestamp,Analog Accel X,Analog Accel Y,Analog Accel Z"
   counter = 0
   id_last = 0
   id_current = 0
   missing_total = 0
   rx_total = 0
   rx_cnt = 0
   rx_max = 491520
   try:
      while True:
         while numbytes < framesize:
            temp = ser.read(1)
            (temp1) = struct.unpack('B',temp[0:1])
            #print "temp1 = ", temp1[0]
            if int(temp1[0]) != 0:
               found0 = 0       
            else:           
               found0 = 1           
               ddata += temp
               ddata += ser.read(framesize-1)
               numbytes = len(ddata)
         if not found0:
            continue          
         
         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)         
         
         (id0, id1) = struct.unpack('BB', data[1:3])
         id_current = id0 + id1*256
         missing_current = (id_current - id_last - 1) % 16384
         missing_total += missing_current
         id_last = id_current
         rx_total +=1
         missing_rate =  float(missing_total)/float(rx_total)
         print binascii.hexlify(data[0:9]),binascii.hexlify(data[15:framesize]), missing_current, missing_total, rx_total, ("%.4f"% missing_rate)
      
   except KeyboardInterrupt:
#send stop streaming command
      ser.flushInput()
      ser.write(struct.pack('B', 0x28))
      print
      print "stop command sent, waiting for ACK_COMMAND"
      wait_for_ack()
      print "ACK_COMMAND received."
#close serial port
      ser.close()
      print "All done"
