#!/usr/bin/env python
import RPi.GPIO as GPIO
import time

RoAPin = 20    # pin11
RoBPin = 21   # pin12

GPIO.setup(RoAPin, GPIO.IN) # input mode
GPIO.setup(RoBPin, GPIO.IN)
GPIO.setmode(GPIO.BCM)

globalCounter = 0.0
gain=0.97593582887
flag = 0
Last_RoB_Status = 0.0
Current_RoB_Status = 0.0
grados=0.0


 
def rotaryDeal():
 global flag
 global Last_RoB_Status
 global Current_RoB_Status
 global globalCounter
 global gain
 global grados

 Last_RoB_Status = GPIO.input(RoBPin)
 while(not GPIO.input(RoAPin)):
   Current_RoB_Status = GPIO.input(RoBPin)
   flag = 1
 
 if flag == 1:
      flag = 0
      if (Last_RoB_Status == 0) and (Current_RoB_Status == 1):
         globalCounter = globalCounter + 1.0
         #print ('globalCounter =')
         #print ("{0:.3f}".format(globalCounter*gain))
      if (Last_RoB_Status == 1) and (Current_RoB_Status == 0):
         globalCounter = globalCounter - 1.0
         #print ('globalCounter =')
         #print ("{0:.3f}".format(globalCounter*gain))
 grados=globalCounter*gain
 return (grados)
def clear(ev=None):
        globalCounter=0
        print ('globalCounter = %d'%globalCounter)
        time.sleep(1)



def loop():
        global globalCounter
        while True:
               sensor=rotaryDeal()
               print ('globalCounter = %d' % sensor)

def destroy():
        GPIO.cleanup()             # Release resource

if __name__ == '__main__':     # Program start from here
       
        try:
               loop()
        except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
               destroy()
