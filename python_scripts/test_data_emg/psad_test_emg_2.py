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
   ser.write(struct.pack('BB', 0x09, 0x02))# change here to configure EMG
   wait_for_ack()
   print "reset to default command sending, done."
   ser.write(struct.pack('B', 0x27))
   wait_for_ack()
   print "start command sending, done."

# read incoming data
   ddata = ""
   numbytes = 0
   framesize = 31 # 6byte emg, 12byte accel, 6byte gyro, 8byte strain gauge, 3 byte status+ts
   found0 = 0       
   emg1 = emg1_l = emg1_m = emg1_h = 0
   emg2 = emg2_l = emg2_m = emg2_h = 0

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
         #(emg1_l, emg1_m,emg1_h) = struct.unpack('BBB', data[3:6])
         #(emg2_l, emg2_m,emg2_h) = struct.unpack('BBB', data[6:9])
         (emg_1, emg_2) = struct.unpack('BB', data[3:5])
         if (ts0%4)==0:#0      
            emg1_l = emg_1
            emg1_m = emg_2
         elif (ts0%4)==1:#0  
            emg1_h = emg_1
            emg2_l = emg_2    
         elif (ts0%4)==2:#0  
            emg2_m = emg_1
            emg2_h = emg_2    
         else:#0                     
            emg1 = emg1_l+emg1_m*256+emg1_h*65536    
            emg2 = emg2_l+emg2_m*256+emg2_h*65536  
            
         (accel1x, accel1y, accel1z,accel2x, accel2y, accel2z) = struct.unpack('>hhhhhh', data[5:17])
         (gyrox_l,gyrox_h, gyroy_l,gyroy_h, gyroz_l, gyroz_h) = struct.unpack('BBBBBB', data[17:23])
         gyrox = gyrox_l+gyrox_h*256
         gyroy = gyroy_l+gyroy_h*256
         gyroz = gyroz_l+gyroz_h*256
         (sg1_l,sg1_h,sg2_l,sg2_h,sg3_l,sg3_h,sg4_l,sg4_h) = struct.unpack('BBBBBBBB', data[23:31])
         sg1 = sg1_l+sg1_h*256
         sg2 = sg2_l+sg2_h*256
         sg3 = sg3_l+sg3_h*256
         sg4 = sg4_l+sg4_h*256
         #(sg1,sg2,sg3,sg4) = struct.unpack('hhhh', data[27:35])
         #print "0x%02x,%02x,%02x|%5d,%5d,%5d|%5d,%5d,%5d|%5d,%5d,%5d|%4x,%4x,%4x,%4x" % (data0,ts0, ts1, accel1x, accel1y, accel1z,accel2x, accel2y, accel2z, gyrox, gyroy, gyroz, sg1,sg2,sg3,sg4)
         print "0x%02x,%02x,%02x|%06x,%06x|%04x,%04x,%04x,%04x" % (data0,ts0, ts1,emg1, emg2, sg1,sg2,sg3,sg4)
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
