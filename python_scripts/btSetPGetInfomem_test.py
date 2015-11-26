#!/usr/bin/python
import sys, struct, serial, binascii, time, array, random

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
   print "   btSetPGetInfomem_test.py Com12"
   print "or"
   print "   btSetPGetInfomem_test.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 460800)
   ser.flushInput()
   time.sleep(2)   
   outArg_head = [0x29, 0x80, 0x00, 0x00] #  24?  29?
   num_send = 0
   
   try:
      while 1:      
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
                  
               #print outArg_string[18:132]
               #print inArg[14:128]
               print "try number %d\t  compare out and in: "%num_send, outArg_string[17:132] == inArg[13:128]
               num_send+=1
               time.sleep(0.5)
               break
            
   except KeyboardInterrupt:
#close serial port
      ser.close()
      print
      print "All done"

#close serial port
   #ser.close()
   #print
   #print "All done"
