"""
A* grid based planning
author: Atsushi Sakai(@Atsushi_twi)
"""

import matplotlib.pyplot as plt
import math

show_animation = True

"""
Clase que define cada uno de los nodos de la trayectoria.
x: Posición horizontal del nodo
y: Posición vertical del nodo
cost: Costo asignado al nodo
pind: Identificador del nodo
"""
class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind
        " Agregar atributo del potencial "

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

"""
Función que determina el camino final a partir del conjunto cerrado de elementos.
ngoal: Nodo de destino
closedset: Conjunto de todos los nodos ya evaluados
reso: Resolución del mapa
rx, ry: Definen el conjunto de pares coordenados que identifican los nodos del recorrido final
"""
def calc_fianl_path(ngoal, closedset, reso):
    # generate final course

    # La construcción del camino inicia en el nodo final
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]

    # Indicador que denota los nodos de inicio y llegada
    pind = ngoal.pind

    # Hasta que no se anexe el nodo de origen, se extraen elementos del conjunto
    # El indicador del nodo de origen es siempre -1.
    while pind != -1:

        # Extrae un nodo del conjunto de nodos evaluados
        n = closedset[pind]

        # Anexa el nuevo nodo a la lista de nodos del camino
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry

"""
Planificación de trayectoria con el algoritmo A*

NOTAS: - Lista abierta: Todos los nodos sin evaluar
       - Lista cerrada: Nodos evaluados
       - Nodo padre: Nodo por el cual se llega a tener en cuenta un nodo 
                     dado como posible candidato
       - G: Coste de ir desde el nodo padre al actual
       - H: Distancia mínima y optimista (ignorando los obstáculos) al destino
       - F: G + H 

0. La celda de origen se añade a la lista abierta

1. Se extrae el primer elemento de la lista abierta y se inserta en la lista cerrada 

2. Se escogen los nodos adyacentes al nodo escogido

3. Para cada nodo adyacente:

   a) Si el nodo es el objetivo, se ha construido exitosamente la trayectoria. Se recorre
      la cadena de padres en sentido inverso hasta llegar al origen
   b) Si el nodo es un obstáculo, éste se ignora
   c) Si el nodo está en la lista cerrada (ya fue evaluado), éste se ignora
   d) Si el nodo ya está en la lista abierta: 
      - Si su costo G es mejor que el actual se recalculan F, G y H, 
        y se coloca como padre el nodo extraído.
      - Si su costo G no es mejor que el actual, éste se ignora
   e) Para el resto de los nodos adyacentes, se establece como padre el nodo extraído y se
      recalculan F, G, H. Después se añaden a la lista abierta

4. Se ordena la lista abierta. Ésta es una lista ordenada de manera ascendente en función
   del valor F de las celdas

5. Volver al paso 1
"""
def a_star_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    sx: Posición x del nodo de inicio
    sy: Posición y del nodo de inicio
    gx: Posición x del nodo objetivo
    gy: Posición y del nodo objetivo
    ox: Lista de posiciones x de los obstáculos
    oy: Lista de posiciones x de los obstáculos
    reso: Resolución de la grilla
    rr: Radio del robot
    """

    # Inicialización del nodo de partida
    nstart = Node(round(sx / reso), round(sy / reso), 0.0, -1)

    # Inicialización del nodo de llegada
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1)

    # Se ajustan las posiciones de los obstáculos a la resolución del mapa
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    # Se define la región del mapa sobre la cual existen obstáculos
    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    # Modelo base para G (ver descripción de la función)
    motion = get_motion_model()

    # Inicialización de las listas abierta y cerrada (diccionario vacío)
    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart # El nodo de partida se anexa a la lista abierta (Paso 0)

 
    while 1:
        # Se extrae el primer nodo de la lista abierta y se define como el nodo actual(Paso 1)
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_h(ngoal, openset[o].x, openset[o].y))
        current = openset[c_id]
        #  print("current", current)

        # Mostrar animación (NO ES PARTE DEL ALGORITMO)
        if show_animation:
            plt.plot(current.x * reso, current.y * reso, "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)
         
        """
        De aquí en adelante se siguen los pasos 2 y 3 del algoritmo
        """

        # Paso 3, caso a (Se ha llegado a la meta, finaliza la construcción de la trayectoria)
        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Se extrae el primer elemento de la lista abierta y se inserta en la lista cerrada (Paso 2)
        del openset[c_id]
        closedset[c_id] = current

        # Se asigna al nodo el modelo de movimiento
        for i in range(len(motion)):
            node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            # Paso 3, caso b (el nodo es un obstáculo)
            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            # Paso 3, caso c (el nodo ya fue evaluado)
            if n_id in closedset:
                continue

            # Paso 3, caso d (el nodo ya está en la lisa abierta y se recalculan F,G y H)
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    # Se reconstruye la trayectoria final una vez hallado el nodo de meta
    rx, ry = calc_fianl_path(ngoal, closedset, reso)

    return rx, ry


"""
Función que, dada la coordenada (x,y) de un nodo, calcula la distancia optimista
desde el mismo hasta el nodo de llegada. 

NOTA: La métrica utilizada es la distancia euclideana, no distancia de Manhattan
"""
def calc_h(ngoal, x, y):
    w = 10.0  # weight of heuristic
    d = w * math.sqrt((ngoal.x - x)**2 + (ngoal.y - y)**2)
    return d


"""
Función que verifica si un nodo dado constituye un obstáculo, o si este perte nece a la
región con obstáculos
"""
def verify_node(node, obmap, minx, miny, maxx, maxy):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    if obmap[node.x][node.y]:
        return False

    return True


"""
ox: Lista de posiciones en x de los obstáculos
oy: Lista de posiciones en y de los obstáculos
reso: Resolución del mapa
vr: Radio del robot
"""
def calc_obstacle_map(ox, oy, reso, vr):

    """
    Ubica los menores puntos en el eje x y el eje y para 
    delimitar la región del mapa en la cual hay obstáculos
    """
    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)
    #  print("xwidth:", xwidth)
    #  print("ywidth:", ywidth)

    # obstacle map generation

    
    # Genera un mapa de xwidth x ywidth de ceros
    obmap = [[False for i in range(xwidth)] for i in range(ywidth)]

    
    for ix in range(xwidth):
        x = ix + minx
        for iy in range(ywidth):
            y = iy + miny
            #  print(x, y)
            for iox, ioy in zip(ox, oy):
                # Condición para evitar colisión
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr / reso:
                    obmap[ix][iy] = True
                    break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth

"""
???
"""
def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


"""
Permite asignar la ponderación de cada una de las casillas
adyacentes a una posición dada. Cada fila de la lista motion
representa las ocho celdas adyacentes a la posición actual.
1: dx (derecha), dy (arriba)
-1: dx (izquierda), dy (abajo)

"""
def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 2.0  # [m]
    sy = 5.0  # [m]
    gx = 45.0  # [m]
    gy = 20.0


    """
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m] """
    " reso "
    grid_size = 1.0  # [m]
    robot_size = 1.0  # [m]

    ox, oy = [], []

    " Generación de los obstáculos "
    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    " Parámetro que se debe enviar al carrito "
    rx, ry = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)
    print( rx, ry)
   
    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()