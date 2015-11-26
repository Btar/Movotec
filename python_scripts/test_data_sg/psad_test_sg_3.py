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
   print "set accel data structure, done."
   ser.write(struct.pack('B', 0x27))
   wait_for_ack()
   print "start command sending, done."

# read incoming data
   ddata = ""
   numbytes = 0
   framesize = 39 # 6byte emg, 12byte accel, 6byte gyro, 8byte strain gauge, 3 byte status+ts
   found0 = 0                
   sg1_l = sg1_h = sg2_l = sg2_h = sg3_l = sg3_h = sg4_l = sg4_h = 0
   #print "Packet Type,Timestamp,Analog Accel X,Analog Accel Y,Analog Accel Z"
   counter = 0
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
         
         #counter+=1
         #if counter == 2048:
         #   ser.write(struct.pack('B', 0x2a))
         #   counter = 0
         #   print binascii.hexlify(data)

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
         (sg1_l,sg1_m,sg1_h,sg2_l,sg2_m,sg2_h,sg3_l,sg3_m,sg3_h,sg4_l,sg4_m,sg4_h) = struct.unpack('BBBBBBBBBBBB', data[27:39])
         sg1 = sg1_l+sg1_m*256+sg1_h*65536
         sg2 = sg2_l+sg2_m*256+sg2_h*65536
         sg3 = sg3_l+sg3_m*256+sg3_h*65536
         sg4 = sg4_l+sg4_m*256+sg4_h*65536
         if sg1 > 0x7FFFFF:
             sg1 -= 0x1000000    
         if sg2 > 0x7FFFFF:
             sg2 -= 0x1000000    
         if sg3 > 0x7FFFFF:
             sg3 -= 0x1000000    
         if sg4 > 0x7FFFFF:
             sg4 -= 0x1000000    
         #(sg1,sg2,sg3,sg4) = struct.unpack('hhhh', data[27:35])
         #print "0x%02x,%02x,%02x|%5d,%5d,%5d|%5d,%5d,%5d|%5d,%5d,%5d|%4x,%4x,%4x,%4x" % (data0,ts0, ts1, accel1x, accel1y, accel1z,accel2x, accel2y, accel2z, gyrox, gyroy, gyroz, sg1,sg2,sg3,sg4)
         
         buf_len = ser.inWaiting()
         print "0x%02x,%02x,%02x|%06x,%06x|%04x,%04x,%04x,%04x" % (data0,ts0, ts1,emg1, emg2, sg1,sg2,sg3,sg4) , buf_len
         
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
