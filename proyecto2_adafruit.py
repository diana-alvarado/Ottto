# import system libraries
import time
import serial
import sys
# import Adafruit IO REST client
from Adafruit_IO import Client, Feed, RequestError
#from serial import Timeout

# Set to your Adafruit IO key.
# Remember, your key is a secret,
# so make sure not to publish it when you publish this code!
ADAFRUIT_IO_KEY = 'aio_Nary47MpptWzTrbCMDDZAhKmwaRl'

# Set to your Adafruit IO username.
# (go to https://accounts.adafruit.com to find your username)
ADAFRUIT_IO_USERNAME = 'diana_alvarado'


prev_feed_1 = 0
prev_feed_2 = 0
prev_feed_3 = 0
prev_feed_4 = 0

# Create an instance of the REST client.
aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

feed_opcion = int
opcion_ = int

ser = serial.Serial(
    port = 'COM2',
    baudrate = 9600,
    timeout = 1
    )

print("connected to:"+ser.portstr)
ser.isOpen()

while True:
    b1 = aio.receive('servouno')
    op1 = int(b1.value)
    if (op1 == 1):
        opcion_ = 1
        ser.write([int(b1.value)])  
        print('POTENCIOMETRO', b1.value, 'seleccionado')

    b2 = aio.receive('servodos')
    op2 = int(b2.value)
    if (op2 == 2):
        opcion_ = 2
        ser.write([int(b2.value)])  
        print('POTENCIOMETRO', b2.value, 'seleccionado')
    #time.sleep(0.5)

    b3 = aio.receive('servotres')
    op3 = int(b3.value)
    if (op3 == 3):
        opcion_ = 3
        ser.write([int(b3.value)])  
        print('POTENCIOMETRO', b3.value, 'seleccionado')
    #time.sleep(0.5)

    b4 = aio.receive('servocuatro')
    op4 = int(b4.value)    
    if (op4 == 4):
        opcion_ = 4
        ser.write([int(b4.value)])  
        print('POTENCIOMETRO', b4.value, 'seleccionado')

    b5 = aio.receive('aceptar')
    op5 = int(b5.value)  
    if (op5 == 5):
        if (opcion_ == 1):
            feed_1 = aio.feeds('servo1')                        #feed 1 --> potenciometro 1
            read_feed_1 = aio.receive(feed_1.key)                       #tomar el valor analogico
            ser.write([int(read_feed_1.value)])                         #manda el valor 
            
            if (read_feed_1.value != prev_feed_1):                      # si el valor actual es ! al anterior
                print('VALOR DE POTENCIOMETRO 1 <- ', read_feed_1.value)  #mostrar valor que se recibio
            prev_feed_1 = read_feed_1.value 

        if (opcion_ == 2):
            feed_2 = aio.feeds('servo2')                        #feed 1 --> potenciometro 1
            read_feed_2 = aio.receive(feed_2.key)                       #tomar el valor analogico
            ser.write([int(read_feed_2.value)])                         #manda el valor 
            
            if (read_feed_2.value != prev_feed_2):                      # si el valor actual es ! al anterior
                print('VALOR DE POTENCIOMETRO 2 <- ', read_feed_2.value)  #mostrar valor que se recibio
            prev_feed_2 = read_feed_2.value  

        if (opcion_ == 3):
            feed_3 = aio.feeds('servo3')                          #feed 1 --> potenciometro 1
            read_feed_3 = aio.receive(feed_3.key)                         #tomar el valor analogico
            ser.write([int(read_feed_3.value)])                           #manda el valor 
            
            if (read_feed_3.value != prev_feed_3):                        # si el valor actual es ! al anterior
                print('VALOR DE POTENCIOMETRO 3 <- ', read_feed_3.value)  #mostrar valor que se recibio
            prev_feed_3 = read_feed_3.value   

        elif (opcion_ == 4):
            feed_4 = aio.feeds('servo4')                        #feed 1 --> potenciometro 1
            read_feed_4 = aio.receive(feed_4.key)                       #tomar el valor analogico
            ser.write([int(read_feed_4.value)])                         #manda el valor 
            
            if (read_feed_4.value != prev_feed_4):                      # si el valor actual es ! al anterior
                print('VALOR DE POTENCIOMETRO 4 <- ', read_feed_4.value)  #mostrar valor que se recibio
            prev_feed_4 = read_feed_4.value 
    time.sleep(0.5)