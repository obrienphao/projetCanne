
import argparse
import cv2 as cv
import numpy as np
import torch 
from time import sleep
from threading import Thread

#implementation differentes parties 
import jetsonref as jetSonML
import uart_sensor_a02yyuw as sensor_distance 
import generateSound as sound_aveugle


OPEN_FUNCTION  = 0

class canne_intelligente(object):

    """docstring for canne_intelligente"""
    global sensor_lidar

    def __init__(self):
        #init sensor distance 
        baud = 9600 
        String_tty_port ="/dev/ttyTHS1"
        self.sensor_lidar = sensor_distance.sensor_A02YYUW_init(baudRate=baud,portCom=String_tty_port)
   
    def threading_camera_under(self):
       jetSonML.detect_image()

    def sensor_lidar_front(self):
        print(" distance objet --------------------------------------:  ")
        while True:
            valueDistance = sensor_distance.sensor_A02YYUW_read(confiSerial=sensor_lidar)
            print("value distance = ", valueDistance)
            if(valueDistance >0.5 and valueDistance < 5):
                #generate sound message
                messageSpeech =  "Obstacle dans " + valueDistance
                sound_aveugle.text_to_speech(messageSpeech)
                #Data trouvée

                if OPEN_FUNCTION : 
                    listeObstacle = jetSonML.generate_name_obstacle()
                    #speech listObject 
                    sound_aveugle.speechListObstcale(list=listeObstacle)
      
            sleep(0.1)

   
if __name__ == "__main__":

    #instance class canne_intelligente
    my_canne = canne_intelligente()
    
    try:
        thread_camera_under = Thread(target=my_canne.threading_camera_under)
        thread_camera_under.start()

    except Exception as e:
        print(e)
        my_canne.sensor_lidar_front()
        
    