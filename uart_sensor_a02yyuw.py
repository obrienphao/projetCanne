import serial

"""documentation : https://jetsonhacks.com/2019/10/10/jetson-nano-uart/"""

def sensor_A02YYUW_init(portCom, baud):
    """ 
    PortCom : '/dev/ttyTHS1'
    baudRate : 9600bit
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
    """
    #baud = baudRate
    Config_serial = serial.Serial(port=portCom,
                                   baudrate = baud, 
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

