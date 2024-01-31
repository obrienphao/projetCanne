
import argparse
import cv2 as cv
import numpy as np
from threading import Thread

#implementation differentes parties 
import jetsonref as jetSonML
import uart_sensor_a02yyuw as sensor_distance 
import generateSound as sound_aveugle



class canne_intelligente(object):

    """docstring for canne_intelligente"""
    #global sensor_lidar

    def __init__(self):
        #init sensor distance 
        baud = 9600 
        String_tty_port ="/dev/ttyTHS1"
        self.sensor_lidar = sensor_distance.sensor_A02YYUW_init(String_tty_port, baud)
        self.thread_son =None
   
    def threading_camera_front(self):
       jetSonML.detect_image()

    def sensor_lidar_front(self):
        print(" distance objet --------------------------------------:  ")
        while True:
            #valueDistance = sensor_distance.sensor_A02YYUW_read(confiSerial=sensor_lidar)
            valueDistance = sensor_distance.calcul_distance(self.sensor_lidar)

            #print("distance obstacle = ",valueDistance , " cm")
            
            if(valueDistance >0.5 and valueDistance < 200):
                #generate sound message
                Obstacle = jetSonML.generate_name_obstacle()
                messageSpeech =  "Obstacle "+ Obstacle +" dans "+ str(valueDistance) + " cm"
                print(messageSpeech)
                #sound_aveugle.text_to_speech(messageSpeech)
                #Data trouvée
                
                #for detection in listeObstacle:
                    #print(messageSpeech, " Obstacle ", net.GetClassDesc(detection.ClassID),"\n")
                    #print(detection)
                #    objects += net.GetClassDesc(detection.ClassID)
                
                #Si self.speech_thread est None, cela signifie que la fonction text_to_speech() n'a pas été appelée auparavant 
                #ou que l'appel 
                #précédent a été terminé. Dans ce cas, un nouveau thread est créé pour appeler la fonction text_to_speech().
                #Si self.speech_thread n'est pas None et que le thread n'est pas actuellement actif, 
                #cela signifie que l'appel précédent à la fonction text_to_spe() a été terminé. Dans ce cas, 
                #un nouveau thread est créé pour appeler à nouveau la fonction text_to_spe().
                
                if self.thread_son is None or not self.thread_son.is_alive():
                    messageSpeech = messageSpeech 
                    self.thread_son = Thread(target=sound_aveugle.text_to_speech, args=(messageSpeech,))
                    self.thread_son.start()
      
            #sleep(0.1)

   
if __name__ == "__main__":

    #instance class canne_intelligente
    my_canne = canne_intelligente()
    
    try:
        
       # my_canne.sensor_lidar_front()
        thread_camera_under = Thread(target=my_canne.sensor_lidar_front)
        thread_camera_under.start()
           
    except Exception as e:
        print(e)

    my_canne.threading_camera_front()