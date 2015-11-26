#!/usr/bin/python
import sys, struct, serial, binascii, time, msvcrt

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
   print "pick between 0-7 for strain gauge gain"
   #while 1:
   choice = int(msvcrt.getch())
      
   print "# gain choice = %d, actual gain = 2^%d" % (choice,choice)
   ser.write(struct.pack('BB', 0x33, choice))
   wait_for_ack()
   
   print "read back to confirm"
   ser.write(struct.pack('B', 0x34))
   wait_for_ack()
   
   while 1:
      buf_len = ser.inWaiting()
      if buf_len>=2:
         data = ser.read(2)   
         print "rx:",  binascii.hexlify(data), data
         break
      
   print "read all config bytes back to confirm"
   ser.write(struct.pack('B', 0x06))
   wait_for_ack()
   while 1:
      buf_len = ser.inWaiting()
      if buf_len>=4:
         data = ser.read(4)   
         print "rx:",  binascii.hexlify(data), data
         break
      

   ser.close()
   print
   print "All done"