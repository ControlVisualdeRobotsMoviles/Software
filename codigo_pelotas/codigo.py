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
        self.c1 = 0
        self.x1 = 0
        self.y1 = 0
        self.c2 = 0
        self.x2 = 0
        self.y2 = 0
        self.centrox = 0
        self.centroy = 0
        self.colorBack = "vacio"
        self.colorFront = "vacio"

    def centro(self):
        self.centrox = (self.x1 + self.x2)/2
        self.centroy = (self.y1 + self.y2)/2

<<<<<<< HEAD

def set_pam(data_serv,elementos,carro,lista):
    cont  = 0
=======
def set_pam(data_serv,elementos,bola,carro):
>>>>>>> 7d301f7fcff1d660019ed3e9b5853745094ddb42
    for i in range(0,elementos):
        if (data_serv[i][2]==carro.colorBack): 
            pass
        elif (data_serv[i][2] == carro.colorFront):
            pass
        else:
            lista.append(Bolas())
            lista[cont].x = data_serv[cont][0][0]
            lista[cont].y = data_serv[cont][0][1]
            lista[cont].radio = data_serv[cont][1]
            lista[cont].color = data_serv[cont][2]
            cont+=1


<<<<<<< HEAD
=======
class vector:
    def __init__(x, y):
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


#lista = [Bola()]
>>>>>>> 7d301f7fcff1d660019ed3e9b5853745094ddb42


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