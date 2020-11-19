#!/usr/bin/env python
#Probando actualización
import RPi.GPIO as GPIO
import time

""" Encoder variables """
RoAPin = 20    # pin11
RoBPin = 21   # pin12
RoSPin = 13    # pin13

globalCounter = 0
gain=0.97593582887
flag = 0
Last_RoB_Status = 0.0
Current_RoB_Status = 0.0
encoder=0.0

""" PID variables """
outMax=100
outMin=-outMax
Set_point=0.0                             # SET RPM value  
feedback=0.0                            
previous_time =0.0  
previous_error=0.0  
Integral=0.0             
Kp=0                                       # Proportional controller Gain (0 to 100)  
Ki=0                                       # Integral controller Gain (0 to 100)  
Kd=0             
""" comienzo de codigo """
def setup():

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RoAPin, GPIO.IN) # input mode
    GPIO.setup(RoBPin, GPIO.IN)
    GPIO.setup(RoSPin,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    rotaryClear()

    """ Setup del motor """
    
    
def rotaryDeal():
 global flag
 global Last_RoB_Status
 global Current_RoB_Status
 global globalCounter
 global gain
 global encoder

 Last_RoB_Status = GPIO.input(RoBPin)
 while(not GPIO.input(RoAPin)):
   Current_RoB_Status = GPIO.input(RoBPin)
   flag = 1

 if flag == 1:
      flag = 0
      if (Last_RoB_Status == 0) and (Current_RoB_Status == 1):
         globalCounter = globalCounter + 1
         print ('globalCounter =')
         encoder= "{0:.3f}".format(globalCounter*gain)
         return(encoder)
      if (Last_RoB_Status == 1) and (Current_RoB_Status == 0):
         globalCounter = globalCounter - 1
         print ('globalCounter =')
         encoder= "{0:.3f}".format(globalCounter*gain)
         return(encoder)

def clear(ev=None):
        globalCounter=0
        print ('globalCounter = %d'%globalCounter)
        time.sleep(1)

def rotaryClear():
        GPIO.add_event_detect(RoSPin, GPIO.FALLING, callback=clear) # wait for fal>

def destroy():
        GPIO.cleanup()             # Release resource
            
""" Funcion de control PID """

def PID_function(Kp,Ki,kd,Set_point,Sensor):  
      
    global previous_time  
    global previous_error  
    global Integral
    
    error = float(Set_point)-Sensor                   # Error entre setpoint y sensor 
      
    if (previous_time== 0):  
         previous_time =time.time()  
           
    current_time = time.time()  
    delta_time = current_time - previous_time  
    delta_error = error - previous_error  
      
    Pout = (Kp/10 * error)                
      
    Integral += (error * delta_time)  
      
      
    if Integral>10:        
        Integral=10  
          
    if Integral<-10:  
        Integral=-10  
      
    Iout=((Ki/10) * Integral)  
      
      
    Derivative = (delta_error/delta_time)         #de/dt  
    previous_time = current_time  
    previous_error = error  
      
    Dout=((Kd/1000 )* Derivative)  
      
    output = Pout + Iout + Dout                  # PID controller output  

    if output>outMax:
        output=outMax
    elif output<outMin:
        output=outMin

    return (output)  




def loop():
       
        while True:
               Valores= Input_data()
               Kp=Valores[0]
               Ki=Valores[1]
               kd=Valores[2]
               Set_point=Valores[3]
               Sensor=rotaryDeal()
               Esfuerzo= PID_function(Kp,Ki,Kd,Set_point,Sensor)
#              
def Input_data():
    Datos_pid=[]
    for v in range(4):
        val=float(input("Inserte Kp, Ki, Kd y Set point en ese orden: "))
        Datos_pid.append(val)
    
    print(Datos_pid)
    return(Datos_pid)

if __name__ == '__main__':     # Program start from here
        setup()
        try:
               loop()
        except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program d>               
               destroy()
