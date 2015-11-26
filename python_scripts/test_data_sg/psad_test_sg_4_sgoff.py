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
   ser.write(struct.pack('BB', 0x12, 0x04))
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
   sg1_l = sg1_h = sg2_l = sg2_h = sg3_l = sg3_h = sg4_l = sg4_h = 0
   #print "Packet Type,Timestamp,Analog Accel X,Analog Accel Y,Analog Accel Z"
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

         #(packettype) = struct.unpack('B', data[0:1])
         #(timestamp, analogaccelx, analogaccely, analogaccelz) = struct.unpack('HHHH', data[1:framesize])
         #print "0x%02x,%5d,\t%4d,%4d,%4d" % (packettype[0], timestamp, analogaccelx, analogaccely, analogaccelz)
         (data0, ts0,ts1) = struct.unpack('BBB', data[0:3])
         sw1 = ts1 >> 7
         (emg1_l, emg1_m, emg1_h) = struct.unpack('BBB', data[3:6])
         (emg2_l, emg2_m, emg2_h) = struct.unpack('BBB', data[6:9])
         emg2 = emg1_l+emg1_m*256+emg1_h*65536                 
         emg1 = emg2_l+emg2_m*256+emg2_h*65536  
         (accel1xl, accel1xh, accel1yl, accel1yh, accel1zl, accel1zh) = struct.unpack('BBBBBB', data[9:15])
         #(accel2xl, accel2xh, accel2yl, accel2yh, accel2zl, accel2zh) = struct.unpack('BBBBBB', data[15:21])
         (gyroxl, gyroxh, gyroyl, gyroyh, gyrozl, gyrozh) = struct.unpack('BBBBBB', data[21:27])     
         
         gyrox = gyroxl+gyroxh*256
         gyroy = gyroyl+gyroyh*256
         gyroz = gyrozl+gyrozh*256    
         if gyrox > 0x7FFF:
             gyrox -= 0x10000
         if gyroy > 0x7FFF:
             gyroy -= 0x10000
         if gyroz > 0x7FFF:
             gyroz -= 0x10000  
         #(sg_1,sg_2) = struct.unpack('BB', data[27:29])
         #(sg1_l,sg1_h,sg2_l,sg2_h,sg3_l,sg3_h,sg4_l,sg4_h) = struct.unpack('BBBBBBBB', data[27:35])
         
         
         
         print "0x%02x,%02x,%02x|%06x,%06x|%5d,%5d,%5d" % (data0,ts0, ts1,emg1, emg2, gyrox, gyroy, gyroz)
         
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
