#!/usr/bin/python
import sys, struct, serial, binascii, time, msvcrt

def wait_for_ack():
   ddata = ""
   spam = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
      spam += ddata
	  
   #print "received: ", binascii.hexlify(spam)
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
   print "0: led toggle"
   print "1: led on"
   print "2: led off"
   while 1:
      choice = int(msvcrt.getch())
      if choice == 0:
         print "led toggle"
         ser.write(struct.pack('B', 0x30))
         wait_for_ack()
      elif choice == 1:
         print "led on"
         ser.write(struct.pack('B', 0x31))
         wait_for_ack()
      elif choice == 2:
         print "led off"
         ser.write(struct.pack('B', 0x32))
         wait_for_ack()
      

   ser.close()
   print
   print "All done"