import requests
import json
import math
import serial, time


demoqe = serial.Serial('COM7', 9600, timeout=1)
cabecera = 255

class Bolas:
    def __init__(self):
        self.radio = 0
        self.x = 0
        self.y = 0
        self.color = "color"

class Carro:
    def __init__(self):
        self.x1 = 0
        self.y1 = 0
        self.x2 = 0
        self.y2 = 0
        self.centrox = 0
        self.centroy = 0
        self.colorBack = "YELLOW"
        self.colorFront = "GREEN"
        self.frontal = False
        self.back = False 

    def centro(self):
        self.centrox = (self.x1 + self.x2)/2
        self.centroy = (self.y1 + self.y2)/2


def set_pam(data_serv,elementos,robot,lista):
    cont = 0
    for i in range(0,elementos):
        if (data_serv[i][2]==robot.colorFront):
            robot.x1 = data_serv[i][0][0]
            robot.y1 = data_serv[i][0][1]
            robot.frontal = True
        elif (data_serv[i][2] == robot.colorBack):
            robot.x2 = data_serv[i][0][0]
            robot.y2 = data_serv[i][0][1]
            robot.back = True
        else:
            lista.append(Bolas())
            lista[cont].x = data_serv[i][0][0]
            lista[cont].y = data_serv[i][0][1]
            lista[cont].radio = data_serv[i][1]
            lista[cont].color = data_serv[i][2]
            cont += 1

    if (robot.frontal and robot.back):
        robot.centro()
        robot.frontal = False
        robot.back = False
        print("\nHAY UN CARRO EN LA PISTA \n")



class vector:
    def __init__(self, x, y):
        self.magnitud = math.sqrt(x*x + y*y)
        if x == 0:
            x = 0.00001
        if (x > 0):
            if (y >= 0):
                self.angle = math.atan(y/x)
            else:
                self.angle = math.atan(y/x)
        elif( x < 0 ):
            if(y >= 0):
                self.angle = math.pi+math.atan(y/x)
            else:
                self.angle = math.atan(y/x)-math.pi




if __name__ == "__main__":
    
    robot = Carro()
    # Define the IP address and Port of the Server.
    ip_address = "127.0.0.1"
    port = "8000"
    # Make the request


    while True:
        r = requests.get("http://" + ip_address + ":" + port)

        # Move the response to another variable
        response = r.json()
            # Print the response
        num_elementos = len(response)
        bolas = []
        set_pam(response,num_elementos,robot,bolas)

       # print ("tipo de dato: "+ str(type(response)) +'\n')
        '''
        
        data = json.loads(response)
        print(data)
        '''
       # print(response)
        #print(bolas[0].color)
       # print(len(bolas))

        vector_bola = []


        for i in range(0, len(bolas)):
            vector_bola.append(vector(bolas[i].x, bolas[i].y))

        if len(bolas) != 0:

            vector_referencia = vector(bolas[0].x - robot.centrox, bolas[0].y -robot.centroy)
            vector_carro = vector(robot.x1-robot.x2, robot.y1-robot.y2)
           # print(robot.x1, robot.y1, robot.x2, robot.y2)

            #print(bolas[3].x, bolas[3].y)
           # print(vector_referencia.angle*180/math.pi, vector_referencia.magnitud)
           # print(vector_carro.angle*180/math.pi, vector_carro.magnitud)

            angle = vector_referencia.angle*180/math.pi - vector_carro.angle*180/math.pi

            if angle >= 180:
                angle = angle - 180
            if angle <= -180:
                angle = angle + 360
            print('Thetha: ' + str(angle))
            print('Magnitud: ' + str(int(100 * vector_referencia.magnitud)))

            if angle >= 0:
                signo = 0

            else:
                signo = 1
                angle = -1*angle


            rawstring = demoqe.read()

            send = bytearray([cabecera, signo, int(angle), int(100*vector_referencia.magnitud)])
            demoqe.write(send)

        else:

            send = bytearray[(cabecera, 0, 0, 0)]
            demoqe.write(send)

## print(bolas[0].x, bolas[0].y,bolas[1].x, bolas[1].y)
   ## print(vector_bola[0].angle*180/math.pi, vector_bola[1].angle*180/math.pi)
