import requests
import json

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

    def centro(self):
        self.centrox = self.x1 - self.x2
        self.centroy = self.y1 - self.y2
       
def set_pam(data_serv):
    h = 0

    
if __name__ == "__main__":
    # Define the IP address and Port of the Server.
    ip_address = "127.0.0.1"
    port = "8000"
    # Make the request
    r = requests.get("http://" + ip_address + ":" + port)

    # Move the response to another variable
    response = r.json()
    # Print the response

    num_elementos = len(response)

    print ("tipo de dato: "+ str(type(response)) +'\n')
    '''

    data = json.loads(response)
    print(data)

    '''
    print(response)