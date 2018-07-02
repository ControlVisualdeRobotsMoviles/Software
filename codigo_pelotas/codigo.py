import requests
import json
import math


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
        self.colorBack = "vacio"
        self.colorFront = "vacio"
        self.frontal = False
        self.back = False 

    def centro(self):
        self.centrox = (self.x1 + self.x2)/2
        self.centroy = (self.y1 + self.y2)/2


def set_pam(data_serv,elementos,robot,lista):
    cont  = 0
    for i in range(0,elementos):
        if (data_serv[i][2]==robot.colorBack): 
            robot.x1 = data_serv[cont][0][0]
            robot.y1 = data_serv[cont][0][1]
            robot.frontal = True
        elif (data_serv[i][2] == robot.colorFront):
            robot.x2 = data_serv[cont][0][0]
            robot.y2 = data_serv[cont][0][1]
            robot.back = True
        else:
            lista.append(Bolas())
            lista[cont].x = data_serv[cont][0][0]
            lista[cont].y = data_serv[cont][0][1]
            lista[cont].radio = data_serv[cont][1]
            lista[cont].color = data_serv[cont][2]
            cont+=1

    if (robot.frontal and robot.back):
        robot.centro()
        robot.frontal = False
        robot.back = False
        print("\nHAY UN CARRO EN LA PISTA \n")

    

class vector:
    def __init__(self,x, y):
        self.magnitud = math.sqrt(x*x + y*y)
        if (x >= 0):
            if (y >= 0):
                self.angle = math.tan(y/x)
            else:
                self.angle = math.tan(y/x)
        else:
            if(y >= 0):
                self.angle = 180-math.tan(y/x)
            else:
                self.angle = math.tan(y/x)-180


if __name__ == "__main__":
    
    robot = Carro()
    # Define the IP address and Port of the Server.
    ip_address = "127.0.0.1"
    port = "8000"
    # Make the request
    r = requests.get("http://" + ip_address + ":" + port)

    # Move the response to another variable
    response = r.json()
    # Print the response

    num_elementos = len(response)
    bolas = []
    set_pam(response,num_elementos,robot,bolas)

    print ("tipo de dato: "+ str(type(response)) +'\n')
    '''

    data = json.loads(response)
    print(data)

    '''
    print(response)
    print(bolas[0].color)
    print(len(bolas))