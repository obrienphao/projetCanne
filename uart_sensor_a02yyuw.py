import serial
import time

UNIT_TEST = 0

def sensor_A02YYUW_init(portCom, baudRate):
    """ 
    PortCom : '/dev/ttyTHS1'
    baudRate : 9600bit
    """
    #baud = baudRate
    Config_serial = serial.Serial(port = portCom, baudrate= baudRate,
                                  bytesize=serial.EIGHTBITS,
                                  parity=serial.PARITY_NONE,
                                  stopbits=serial.STOPBITS_ONE
                                  )
    print(Config_serial)

    return Config_serial 

def sensor_A02YYUW_read(confiSerial):
    try:
        valueDistance = confiSerial.read()
        return valueDistance
    except KeyboardInterrupt:
        print("closing error sensor...")

def calcul_distance(peripheral_conf):
    value_read = []
    distance =  0
    summ =0
    value_read.clear()

    for compteur in range(4):
        value_read.append(list(sensor_A02YYUW_read(peripheral_conf)))
        #time.sleep(1)
    #print(value_read[0][0], value_read[1][0], value_read[2][0], value_read[3][0])

    if value_read[0] ==list(b'\xff'):
        #HEADER
        #print("header \n")
        summ = (value_read[0][0] + value_read[1][0] + value_read[2][0]) & 0x00FF 
        #print("sum = ", summ)

        if(summ == value_read[3][0]):
            distance = (value_read[1][0]<<8) + value_read[2][0]

            if(distance > 30):
                distance = distance /10
            else:
                distance = -1
                #print("erreur \n")
    return distance

    #print(distance , " cm \n" )


if UNIT_TEST == 1 :
    peripheral_conf = sensor_A02YYUW_init('/dev/ttyTHS1',9600)
    
    while True:
       distance =  calcul_distance(peripheral_conf)
       print("distance obstacle = ",distance , " cm")
       