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
# send the set sensors command
   print "changing settings 1"
   ser.write(struct.pack('BB', 0x09, 0))
   wait_for_ack()   
   print "changing settings 2"
   ser.write(struct.pack('BB', 0x0c, 0))
   wait_for_ack()   
   print "changing settings 3"
   ser.write(struct.pack('BB', 0x0f, 0))
   wait_for_ack()   
   print "changing settings 4"
   ser.write(struct.pack('BB', 0x12, 0))
   wait_for_ack()   
   print "changing settings 5"
   ser.write(struct.pack('BB', 0x15, 3))
   wait_for_ack()   
   print "changing settings 6"
   ser.write(struct.pack('BB', 0x18, 3))
   wait_for_ack()   
   print "changing settings 7"
   ser.write(struct.pack('BB', 0x2e, 0))
   wait_for_ack()   
# send start streaming command
   
   print "start command sending, done."
   ser.write(struct.pack('B', 0x27))
   wait_for_ack()

# read incoming data
   ddata = ""
   numbytes = 0
   framesize = 35 # 6byte emg, 12byte accel, 6byte gyro, 8byte strain gauge, 3 byte status+ts

   #print "Packet Type,Timestamp,Analog Accel X,Analog Accel Y,Analog Accel Z"
   try:
      while True:
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)
         
         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)

         #(packettype) = struct.unpack('B', data[0:1])
         #(timestamp, analogaccelx, analogaccely, analogaccelz) = struct.unpack('HHHH', data[1:framesize])
         #print "0x%02x,%5d,\t%4d,%4d,%4d" % (packettype[0], timestamp, analogaccelx, analogaccely, analogaccelz)
         (data0, ts0,ts1) = struct.unpack('BBB', data[0:3])
         (emg1) = struct.unpack('>i', (data[3:6] + '\0'))[0] >> 8
         (emg2) = struct.unpack('>i', (data[6:9] + '\0'))[0] >> 8
         (accel1x, accel1y, accel1z,accel2x, accel2y, accel2z) = struct.unpack('HHHHHH', data[9:21])
         (gyrox, gyroy, gyroz) = struct.unpack('HHH', data[21:27])
         (sg1,sg2,sg3,sg4) = struct.unpack('HHHH', data[27:35])
         print "0x%02x,%02x,%02x|%4x,%4x,%4x|%4x,%4x,%4x|%4x,%4x,%4x|%4x,%4x,%4x,%4x" % (data0,ts0, ts1, accel1x, accel1y, accel1z,accel2x, accel2y, accel2z, gyrox, gyroy, gyroz, sg1,sg2,sg3,sg4)
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
