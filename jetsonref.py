
import jetson.inference
import jetson.utils

import jetson_emulator.inference as inference
import jetson_emulator.utils as utils

#--------
objectDetect = []

PERMIT_FUNCTION = 0

def detect_image():
    
    net = jetson.inference.detect("ssd-mobilenet-v2",threshold=0.5)
    camera = jetson.utils.gstCamera(1280,720,"/dev/video0")
    display = jetson.utils.gsDisplay()

    while display.IsOpen():
        img, width,height = camera.CaptureRGB()
        detections = net.Detect(img,width,height)
        display.RenderOnce(img,width,height)
        display.SetTitle("Object detect | NetWork {:0.f} FPS".format(net.GetNetworkFPS()))

        #--------A completer-------
        """
        Pour une détéction Multiple
        groupe liste d'object detecter
        """
        if PERMIT_FUNCTION :
            generateListObject(img)
        #--------------------------


def generate_name_obstacle():
    return objectDetect

def generateListObject(dataObject):
    objectDetect.clear()
    objectDetect = dataObject




#test
detect_image()

