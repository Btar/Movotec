#!/usr/bin/python
import sys, struct, serial, binascii, time, msvcrt

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
   return
   
def setget(in_char):
   choice = int(in_char)
   set_cmd = 0
   get_cmd = 0
   if choice == 1: #emg
      set_cmd = 0x09
      get_cmd = 0x0a
   elif choice == 2: #accel
      set_cmd = 0x0c
      get_cmd = 0x0d
   elif choice == 3: #gyro
      set_cmd = 0x0f
      get_cmd = 0x10
   elif choice == 4: #strain gauge
      set_cmd = 0x12
      get_cmd = 0x13
   elif choice == 5: #accel gange
      set_cmd = 0x15
      get_cmd = 0x16
   elif choice == 6: #gyro range
      set_cmd = 0x18
      get_cmd = 0x19
   elif choice == 7: # Auto Power Off enable
      set_cmd = 0x1b
      get_cmd = 0x1c
   elif choice == 8: # Auto sleep enable
      set_cmd = 0x1e
      get_cmd = 0x1f
   elif choice == 9: # SG bits to shift
      set_cmd = 0x2d
      get_cmd = 0x2e
   elif choice == 0: # SG gain
      set_cmd = 0x33
      get_cmd = 0x34
      
   print "input value: "
   val = msvcrt.getch()         
   #set value
   print "writing value:      %s"% val
   ser.write(struct.pack('BB', set_cmd, int(val)))
   wait_for_ack()
   #get value
   ser.write(struct.pack('B', get_cmd))
   wait_for_ack()
   buf_len = ser.inWaiting()
   while buf_len<1:
      buf_len = ser.inWaiting()
   data = ser.read(buf_len) 
   print "getting value back:",  binascii.hexlify(data), data
   #get whole configure value
   ser.write(struct.pack('B', 0x06))
   wait_for_ack()
   while buf_len<4:
      buf_len = ser.inWaiting()
   buf_len = ser.inWaiting()
   data = ser.read(buf_len) 
   print "all config value:  ",  binascii.hexlify(data), data

if len(sys.argv) < 2:
   print "no device specified"
   print "You need to specify the serial port of the device you wish to connect to"
   print "example:"
   print "   setgetRWC.py Com12"
   print "or"
   print "   setgetRWC.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 460800)
   ser.flushInput()
# get Infomem

   print "1: data struct of EMG"
   print "2: data struct of Accels"
   print "3: data struct of Gyro"
   print "4: data struct of Strain Gauge"
   print "5: Accels range"
   print "6: Gyro range"
   print "7: Auto Power Off option"
   print "8: Auto Sleep option"
   print "9: SG bits to shift"
   print "0: SG gain"
   while 1:
      print "choose between 0 to 9 to config: "
      choice = msvcrt.getch()
      setget(choice)
      
#close serial port
   ser.close()
   print
   print "All done"
