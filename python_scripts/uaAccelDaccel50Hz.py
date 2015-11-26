#!/usr/bin/python
import sys, struct, serial, binascii

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
   return

if len(sys.argv) < 2:
   print "no device specified"
   print "You need to specify the serial port of the device you wish to connect to"
   print "example:"
   print "   analogaccelDaccel10Hz.py Com12"
   print "or"
   print "   analogaccelDaccel10Hz.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser_uart = serial.Serial("com6", 921600)
   ser.flushInput()
# send the set sensors command
   ser.write(struct.pack('BBBB', 0x08, 0x80, 0x10, 0x00))  #analog accel, digi accel
   wait_for_ack()
# send the set sampling rate command
   ser.write(struct.pack('BBB', 0x05, 0x80, 0x02)) #51.2Hz (640 (0x280)). Has to be done like this for alignment reasons
   wait_for_ack()
# send start streaming command
   ser.write(struct.pack('B', 0x07))
   wait_for_ack()

# read incoming data
   ddata = ""
   numbytes = 0
   framesize = 15 # 1byte packet type + 2byte timestamp + 3x2byte Analog Accel + 3x2byte Accel

   print "Packet Type,Timestamp,Analog Accel X,Analog Accel Y,Analog Accel ZDigital Accel X,Digital Accel Y,Digital Accel Z"
   try:         
      #buf_len = ser_uart.inWaiting()
      #print buf_len
      while True:
         while numbytes < framesize:
            ddata += ser_uart.read(framesize)
            numbytes = len(ddata)
         
         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)
         
         buf_len = ser_uart.inWaiting()
         ser_uart.read(buf_len)

         (packettype) = struct.unpack('B', data[0:1])
         (timestamp, analogaccelx, analogaccely, analogaccelz, digiaccelx, digiaccely, digiaccelz) = struct.unpack('HHHHhhh', data[1:framesize])
         #print "0x%02x,%5d,\t%4d,%4d,%4d,\t\t%4d,%4d,%4d " % (packettype[0], timestamp, analogaccelx, analogaccely, analogaccelz, digiaccelx, digiaccely, digiaccelz)
         print binascii.hexlify(data), buf_len

   except KeyboardInterrupt:
#send stop streaming command
      ser.write(struct.pack('B', 0x20))
      wait_for_ack()
#close serial port
      ser.close()
      ser_uart.close()
      print
      print "All done"
