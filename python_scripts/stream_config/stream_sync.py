#!/usr/bin/python
import sys, struct, serial, binascii, time, array

syncBuffExpected1 = [0xff, 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0xff]
syncBuffExpected = array.array('B',syncBuffExpected1).tostring()  
syncBuff = "123456789012"

def find_dummy(inChar):
   global syncBuff
   syncBuff = syncBuff[1:]
   syncBuff += inChar
   if syncBuff == syncBuffExpected:
      print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
      print "!!!!!!!!!!!!!!!!!!!!!!    sync    !!!!!!!!!!!!!!!!!!!!!!!!!!"
      print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
      ser.flushInput()
      time.sleep(0.1) 
      ser.flushInput()
      time.sleep(0.1) 
      ser.flushInput()
   return
   
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
   ser.write(struct.pack('B', 0x27))
   wait_for_ack()
   print "start command sending, done."

# read incoming data
   ddata = ""
   numbytes = 0
   framesize = 35 # 6byte emg, 12byte accel, 6byte gyro, 8byte strain gauge, 3 byte status+ts
   found0 = 0      
   stream_cnt = 0   

   #print "Packet Type,Timestamp,Analog Accel X,Analog Accel Y,Analog Accel Z"
   try:
      while 1:
         #if numbytes < framesize:
         #   continue
            
         while numbytes < framesize:
            temp = ser.read(1)
            find_dummy(temp)
            (temp1) = struct.unpack('B',temp[0:1])
            if int(temp1[0]) != 0:
               #print 0
               found0 = 0   
               #continue            
            else:           
               #print 1
               found0 = 1           
               ddata += temp       
               for i in range (1,framesize):    # this reads 1 to framesize-1
                  #ddata += ser.read(framesize-1)
                  temp = ser.read(1)
                  ddata += temp       
                  find_dummy(temp)            
               numbytes = len(ddata)
               
         if not found0:
            continue
            
         #print "hehe, ", syncBuff
         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)

         #(packettype) = struct.unpack('B', data[0:1])
         #(timestamp, analogaccelx, analogaccely, analogaccelz) = struct.unpack('HHHH', data[1:framesize])
         #print "0x%02x,%5d,\t%4d,%4d,%4d" % (packettype[0], timestamp, analogaccelx, analogaccely, analogaccelz)
         (data0, ts0,ts1) = struct.unpack('BBB', data[0:3])
         sw1 = ts1 >> 7
         sw2 = (ts1 >> 6)&0x01
         (emg1) = struct.unpack('>i', (data[3:6] + '\0'))[0] >> 8
         (emg2) = struct.unpack('>i', (data[6:9] + '\0'))[0] >> 8
         #(accel1xl, accel1xh, accel1yl, accel1yh, accel1zl, accel1zh) = struct.unpack('BBBBBB', data[9:15])
         #(accel2xl, accel2xh, accel2yl, accel2yh, accel2zl, accel2zh) = struct.unpack('<HHHHHH', data[15:21])
         #accel1x, accel1y, accel1z,accel2x, accel2y, accel2z
         #(accel1x, accel1y, accel1z,accel2x, accel2y, accel2z) = struct.unpack('>hhhhhh', data[9:21])
         #(gyrox, gyroy, gyroz) = struct.unpack('>hhh', data[21:27])         
         (accel1xl, accel1xh, accel1yl, accel1yh, accel1zl, accel1zh) = struct.unpack('BBBBBB', data[9:15])
         (accel2xl, accel2xh, accel2yl, accel2yh, accel2zl, accel2zh) = struct.unpack('BBBBBB', data[15:21])
         (gyroxl, gyroxh, gyroyl, gyroyh, gyrozl, gyrozh) = struct.unpack('BBBBBB', data[21:27])
         accel1x = accel1xl+accel1xh*256
         accel1y = accel1yl+accel1yh*256
         accel1z = accel1zl+accel1zh*256
         accel2x = accel2xl+accel2xh*256
         accel2y = accel2yl+accel2yh*256
         accel2z = accel2zl+accel2zh*256
         if accel1x > 0x7FFF:
             accel1x -= 0x10000
         if accel1y > 0x7FFF:
             accel1y -= 0x10000
         if accel1z > 0x7FFF:
             accel1z -= 0x10000
         if accel2x > 0x7FFF:
             accel2x -= 0x10000
         if accel2y > 0x7FFF:
             accel2y -= 0x10000
         if accel2z > 0x7FFF:
             accel2z -= 0x10000
         gyrox = gyroxl+gyroxh*256
         gyroy = gyroyl+gyroyh*256
         gyroz = gyrozl+gyrozh*256
         if gyrox > 0x7FFF:
             gyrox -= 0x10000
         if gyroy > 0x7FFF:
             gyroy -= 0x10000
         if gyroz > 0x7FFF:
             gyroz -= 0x10000
         (sg1,sg2,sg3,sg4) = struct.unpack('>hhhh', data[27:35])
         print "0x%02x,%02x,%02x,%1x,%1x|%5d,%5d,%5d|%5d,%5d,%5d|%5d,%5d,%5d| %4d" % (data0,ts0, ts1, sw1, sw2, accel1x, accel1y, accel1z,accel2x, accel2y, accel2z, gyrox, gyroy, gyroz, (stream_cnt % 2048))
         #print "0x%02x,%02x,%02x|%5d,%5d|%5d,%5d,%5d,%5d" % (data0,ts0, ts1,emg1, emg2, sg1,sg2,sg3,sg4)
         stream_cnt += 1
         if (stream_cnt % 2048)==0:            
            ser.write(struct.pack('B', 0x2a))
            #wait_for_ack()
            
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
