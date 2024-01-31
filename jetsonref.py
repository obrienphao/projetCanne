
from jetson_inference import *
from jetson_utils import *
import cv2
import numpy as np
import sys
import argparse


#--------
# Variables globales pour stocker les détections et activer/désactiver les tests unitaires.
objectDetect  =""
UNIT_TEST   =   0
#----------*

# Définition de l'argument modelTraining pour spécifier le modèle de détection d'objets à utiliser.
parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, 
                                 epilog=detectNet.Usage() + videoSource.Usage() + videoOutput.Usage() + Log.Usage())

#------------------------------------------------------------------------------
try:
	args = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)


#---------- Models -------------------------------------------------------------
# Liste des modèles de détection d'objets disponibles.
listModelTraining =["ssd-mobilenet-v2","ssd-inception-v2" ]
#------------------------------------------

# Sélection du modèle de détection d'objets à utiliser.
modelTraining = listModelTraining[1]

# Création d'une instance de detectNet avec le modèle spécifié et un seuil de détection de 0.5.
net = detectNet(modelTraining, threshold=0.5)

def detect_image():

   # Création d'une instance de videoSource pour capturer des images de la caméra.
    camera = videoSource("csi://0")      # '/dev/video0' for V4L2

    # Création d'une instance de videoOutput pour afficher les images capturées.
    display = videoOutput("display://0")

    
    global objectDetect

    # Boucle infinie pour capturer et traiter des images en continu.
    while display.IsStreaming() :
      
      
         # Capture d'une image de la caméra et vérification qu'elle n'est pas None.
        img = camera.Capture(format ='rgb8') #timeout=1000)
        if img is None:
             continue

        
        # Détection d'objets dans l'image avec le modèle spécifié.
        detections = net.Detect(img ,width=img.width, height=img.height)
    
       # Conversion de l'image capturée en un tableau NumPy et conversion de son format de couleur de RGB à BGR pour qu'il soit compatible avec OpenCV.
        img_cv = cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)

        # Calcul du centre de l'image.
        center_x = img.width // 2
        center_y = img.height // 2

        # Initialisation de variables pour stocker l'objet le plus proche du centre de l'image et la distance entre l'objet et le centre de l'image.
        closest_detection = None
        closest_distance = float('inf')

        # Boucle sur les détections pour trouver l'objet le plus proche du centre de l'image.
        for detection in detections:

            # Calcul de la distance entre l'objet et le centre de l'image.
            distance = ((detection.Left + detection.Right) // 2 - center_x)**2 + ((detection.Top + detection.Bottom) // 2 - center_y)**2

            # Si l'objet est plus proche du centre de l'image que l'objet actuellement stocké, mise à jour de la variable closest_detection et de la variable closest_distance.
            if distance < closest_distance:
                closest_detection = detection
                closest_distance = distance

       # objectDetect = closest_detection.copy()
        # Si un objet a été détecté, dessin d'un rectangle autour de l'objet le plus proche du centre de l'image.
        if closest_detection:
            bbox = BBox(xmin=closest_detection.Left, ymin=closest_detection.Top, xmax=closest_detection.Right, ymax=closest_detection.Bottom)
            xmin, ymin, xmax, ymax = np.rint([closest_detection.Left, closest_detection.Top, closest_detection.Right, closest_detection.Bottom]).astype(int)
            cv2.rectangle(img_cv, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)

        # Affichage du nom de la classe de l'objet détecté le plus proche du centre.
        try:
            objectDetect = net.GetClassDesc(closest_detection.ClassID)
        except Exception as e:
            print(e)
        #cv2.putText(img, class_name, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        #print(class_name)

        # Affichage de l'image traitée avec la fonction display.Render().
        display.Render(img)
        display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))

        # Affichage de l'image traitée avec OpenCV.
        #cv2.imshow("Image", img_cv)
        #cv2.waitKey(1)

    # Arrêt de la caméra et de l'affichage de la vidéo une fois la boucle terminée.
    camera.Stop()
    display.Stop()
    cv2.destroyAllWindows()


	
def generate_name_obstacle():
    return  objectDetect 


def rogneCenterImage(imgInput):
    
    # determine the amount of border pixels (cropping around the center by half)
    crop_factor = 0.5
    crop_border = ((1.0 - crop_factor) * 0.5 * imgInput.width,
                (1.0 - crop_factor) * 0.5 * imgInput.height)

    # compute the ROI as (left, top, right, bottom)
    crop_roi = (crop_border[0], crop_border[1], imgInput.width - crop_border[0], imgInput.height - crop_border[1])

    # allocate the output image, with the cropped size
    imgOutput = jetson_utils.cudaAllocMapped(width=imgInput.width * crop_factor,
                                            height=imgInput.height * crop_factor,
                                            format=imgInput.format)

    # crop the image to the ROI
    jetson_utils.cudaCrop(imgInput, imgOutput, crop_roi)
    #display.Render(imgOutput)
    return imgOutput

class BBox:
    def __init__(self, xmin, ymin, xmax, ymax):
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax

    def __str__(self):
        return f"BBox(xmin={self.xmin}, ymin={self.ymin}, xmax={self.xmax}, ymax={self.ymax})"


def BoundingBox(image, detections):
    # Calculate the center coordinates of the image
    center_x = image.width // 2
    center_y = image.height // 2

    # Find the object at the center of the image
    object_name = None
    for detection in detections:
        if detection.Left == center_x and detection.Top == center_y:
            object_name = detection.GetClass()
            break

    # Create a bounding box that encloses the center point if an object is detected
    bbox = None
    if object_name:
        bbox = BBox(xmin=center_x - 1, ymin=center_y - 1, xmax=center_x + 1, ymax=center_y + 1)

    # Convert the cudaImage to a cupy array
    image_cupy = cuda.CuPyArray(image.to_pitch_blob())

    # Convert the cupy array to a numpy array
    image_numpy = np.asarray(image_cupy)

    # Create a UMat object from the numpy array
    image_opencv = cv2.UMat(image_numpy)

    # Draw the bounding boxes of the detected objects
    for detection in detections:
        if detection.GetClass() == object_name:
            cv2.rectangle(image_opencv, (int(detection.Left), int(detection.Top)), (int(detection.Right), int(detection.Bottom)), (255, 0, 0), 2)

    # Display the image with detected objects
    cv2.imshow("Image", image_opencv)
    cv2.waitKey(0)

    print(f"Center object name: {object_name}")
    return [bbox] if bbox else []

#test
if UNIT_TEST == 1:
    detect_image()