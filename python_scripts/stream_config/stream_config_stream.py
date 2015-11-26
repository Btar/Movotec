#!/usr/bin/python
import sys, struct, serial, binascii, time, array, random, msvcrt

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

   #print "Packet Type,Timestamp,Analog Accel X,Analog Accel Y,Analog Accel Z"
   stream_cnt = 0
   try:
      while stream_cnt<10240:
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
         (emg1_l, emg1_m,emg1_h) = struct.unpack('BBB', data[3:6])
         (emg2_l, emg2_m,emg2_h) = struct.unpack('BBB', data[6:9])
         emg1 = emg1_l+emg1_m*256+emg1_h*65536
         emg2 = emg2_l+emg2_m*256+emg2_h*65536
         #(emg1) = struct.unpack('>i', (data[3:6] + '\0'))[0] >> 8
         #(emg2) = struct.unpack('>i', (data[6:9] + '\0'))[0] >> 8
         #(accel1xl, accel1xh, accel1yl, accel1yh, accel1zl, accel1zh) = struct.unpack('BBBBBB', data[9:15])
         #(accel2xl, accel2xh, accel2yl, accel2yh, accel2zl, accel2zh) = struct.unpack('<HHHHHH', data[15:21])
         #accel1x, accel1y, accel1z,accel2x, accel2y, accel2z
         (accel1x, accel1y, accel1z,accel2x, accel2y, accel2z) = struct.unpack('>hhhhhh', data[9:21])
         (gyrox_l,gyrox_h, gyroy_l,gyroy_h, gyroz_l, gyroz_h) = struct.unpack('BBBBBB', data[21:27])
         gyrox = gyrox_l+gyrox_h*256
         gyroy = gyroy_l+gyroy_h*256
         gyroz = gyroz_l+gyroz_h*256
         (sg1_l,sg1_h,sg2_l,sg2_h,sg3_l,sg3_h,sg4_l,sg4_h) = struct.unpack('BBBBBBBB', data[27:35])
         sg1 = sg1_l+sg1_h*256
         sg2 = sg2_l+sg2_h*256
         sg3 = sg3_l+sg3_h*256
         sg4 = sg4_l+sg4_h*256
         #(sg1,sg2,sg3,sg4) = struct.unpack('hhhh', data[27:35])
         #print "0x%02x,%02x,%02x|%5d,%5d,%5d|%5d,%5d,%5d|%5d,%5d,%5d|%4x,%4x,%4x,%4x" % (data0,ts0, ts1, accel1x, accel1y, accel1z,accel2x, accel2y, accel2z, gyrox, gyroy, gyroz, sg1,sg2,sg3,sg4)
         print "0x%02x,%02x,%02x|%06x,%06x|%04x,%04x,%04x,%04x" % (data0,ts0, ts1,emg1, emg2, sg1,sg2,sg3,sg4)
         stream_cnt += 1
   except KeyboardInterrupt:
#send stop streaming command
      ser.flushInput()
      ser.write(struct.pack('B', 0x28))
      print
      print "stop command sent, waiting for ACK_COMMAND"
      wait_for_ack()
      print "ACK_COMMAND received."
      
   ser.flushInput()
   buf_len = ser.inWaiting()
   data = ser.read(buf_len)   
   ser.write(struct.pack('B', 0x28))
   print
   print "stop command sent, waiting for ACK_COMMAND"
   wait_for_ack()
   print "ACK_COMMAND received."
   
   print "PRESS ANY KEY TO CONTINUE."
   choice = msvcrt.getch()
   
   ser.flushInput()
   buf_len = ser.inWaiting()
   data = ser.read(buf_len)   
   outArg_head = [0x29, 0x80, 0x00, 0x00] #  24?  29?
   num_send = 0
   
   try:
      while num_send<10:      
         # set Infomem
         #print "-------------------------------------- infoMem D Settinge" 
         outArg_body = []
         for i in range (128):   
            outArg_body.append(random.randint(0,255))   
         outArg = outArg_head + outArg_body
         outArg_string = array.array('B',outArg).tostring()   
         ser.write(outArg)
         wait_for_ack()
         #print "-------------------------------------- infoMem D Getting: " 
         # get Infomem
         ser.write(struct.pack('BBBB', 0x25, 0x80, 0x00, 0x00))
         wait_for_ack()
         cnt=0
         inArg = ""
         
         while 1:
            buf_len = ser.inWaiting()
            if buf_len>=130:
               data = ser.read(2)   
               buf_len -= 2
               #print "rx:  ", binascii.hexlify(data), data
               while buf_len>0:
                  if buf_len >16:
                     data = ser.read(16)   
                     buf_len -=16
                  else:
                     data = ser.read(buf_len)   
                     buf_len =0               
                  #print "rx:%02X"% (cnt),  binascii.hexlify(data), data
                  #print "rx:%02X"% (cnt),  binascii.hexlify(data)
                  inArg += data
                  cnt+=1
                  #print cnt, buf_len
                  
               #print binascii.hexlify(outArg_string[18:132])
               #print binascii.hexlify(inArg[14:128])
               print "try number %d\t  compare out and in: "%num_send, outArg_string[17:132] == inArg[13:128]
               num_send+=1
               time.sleep(0.5)
               break
            
   except KeyboardInterrupt:
#close serial port
      ser.close()
      print
      print "All done"
   
      
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

   #print "Packet Type,Timestamp,Analog Accel X,Analog Accel Y,Analog Accel Z"
   stream_cnt = 0
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
         (emg1_l, emg1_m,emg1_h) = struct.unpack('BBB', data[3:6])
         (emg2_l, emg2_m,emg2_h) = struct.unpack('BBB', data[6:9])
         emg1 = emg1_l+emg1_m*256+emg1_h*65536
         emg2 = emg2_l+emg2_m*256+emg2_h*65536
         #(emg1) = struct.unpack('>i', (data[3:6] + '\0'))[0] >> 8
         #(emg2) = struct.unpack('>i', (data[6:9] + '\0'))[0] >> 8
         #(accel1xl, accel1xh, accel1yl, accel1yh, accel1zl, accel1zh) = struct.unpack('BBBBBB', data[9:15])
         #(accel2xl, accel2xh, accel2yl, accel2yh, accel2zl, accel2zh) = struct.unpack('<HHHHHH', data[15:21])
         #accel1x, accel1y, accel1z,accel2x, accel2y, accel2z
         (accel1x, accel1y, accel1z,accel2x, accel2y, accel2z) = struct.unpack('>hhhhhh', data[9:21])
         (gyrox_l,gyrox_h, gyroy_l,gyroy_h, gyroz_l, gyroz_h) = struct.unpack('BBBBBB', data[21:27])
         gyrox = gyrox_l+gyrox_h*256
         gyroy = gyroy_l+gyroy_h*256
         gyroz = gyroz_l+gyroz_h*256
         (sg1_l,sg1_h,sg2_l,sg2_h,sg3_l,sg3_h,sg4_l,sg4_h) = struct.unpack('BBBBBBBB', data[27:35])
         sg1 = sg1_l+sg1_h*256
         sg2 = sg2_l+sg2_h*256
         sg3 = sg3_l+sg3_h*256
         sg4 = sg4_l+sg4_h*256
         #(sg1,sg2,sg3,sg4) = struct.unpack('hhhh', data[27:35])
         #print "0x%02x,%02x,%02x|%5d,%5d,%5d|%5d,%5d,%5d|%5d,%5d,%5d|%4x,%4x,%4x,%4x" % (data0,ts0, ts1, accel1x, accel1y, accel1z,accel2x, accel2y, accel2z, gyrox, gyroy, gyroz, sg1,sg2,sg3,sg4)
         print "0x%02x,%02x,%02x|%06x,%06x|%04x,%04x,%04x,%04x" % (data0,ts0, ts1,emg1, emg2, sg1,sg2,sg3,sg4)
         stream_cnt += 1
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
