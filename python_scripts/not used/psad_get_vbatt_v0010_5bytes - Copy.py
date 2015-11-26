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
   print "press any key to get status byte"
   
   while 1:
      char_in = msvcrt.getch()
      ser.write(struct.pack('B', 0x03))
      wait_for_ack()
      while 1:
         buf_len = ser.inWaiting()
         if buf_len>=7:
            data = ser.read(7)   
            print "rx:",  binascii.hexlify(data), data
            break

   ser.close()
   print
   print "All done"