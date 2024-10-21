"""
Robótica - Práctica 3
Autores: Pablo Páramo Telle y Luis Cascón Padrón
"""

from controller import Robot  

import numpy as np 
import pandas as pd

# Máxima velocidad de las ruedas soportada por el robot (khepera4).
MAX_SPEED = 47.6
# Velocidad por defecto.
CRUISE_SPEED = 8
# Time step por defecto para el controlador.
TIME_STEP = 32


#Limite de linea negra
BLACK_LIMIT = 500
#Limite salirse de la linea
WHITE_LIMIT = 750

# Nombres de los sensores de distancia basados en infrarrojo.
INFRARED_SENSORS_NAMES = [
    "rear left infrared sensor",
    "left infrared sensor",
    "front left infrared sensor",
    "front infrared sensor",
    "front right infrared sensor",
    "right infrared sensor",
    "rear right infrared sensor",
    "rear infrared sensor",
    "ground left infrared sensor",
    "ground front left infrared sensor",
    "ground front right infrared sensor",
    "ground right infrared sensor"
]

MOTOR_NAMES = [
    "left wheel motor",
    "right wheel motor"
]

# Definir constantes
NUM_ESTADOS = 3
NUM_ACCIONES = 3
NUM_EPISODIOS = 20000

# Medidas del robot y del entorno
RADIO_RUEDA = 21 
ESPACIO_ENTRE_RUEDAS = 108.29 

# Definir los estados y las acciones del robot
estados = ['S1', 'S2', 'S3']
acciones = ['A1', 'A2', 'A3']

# Definir la matriz Q
Q = np.zeros((NUM_ESTADOS, NUM_ACCIONES))

# Definir los parámetros de aprendizaje
alpha = 0.5
gamma = 0.5
epsilon = 1.0


# Función para activar los sensores de distancia
def enable_distance_sensors(robot, timeStep, sensorNames):
    """
    Obtener y activar los sensores de distancia.

    Return: lista con los sensores de distancia activados, en el mismo orden
    establecido en la lista de  nombres (sensorNames).
    """

    sensorList = []

    for name in sensorNames:
        sensorList.append(robot.getDevice(name))

    for sensor in sensorList:
        sensor.enable(timeStep)

    return sensorList

# Función para inicializar los dispositivos
def init_devices(timeStep):
    """
    Obtener y configurar los dispositivos necesarios.

    timeStep: tiempo (en milisegundos) de actualización por defecto para los sensores/actuadores
      (cada dispositivo puede tener un valor diferente).
    """

    # Get pointer to the robot.
    robot = Robot()

    # Obtener dispositivos correspondientes a los motores de las ruedas.
    leftWheel = robot.getDevice("left wheel motor")
    rightWheel = robot.getDevice("right wheel motor")

    # Configuración inicial para utilizar movimiento por posición (necesario para odometría).
    leftWheel.setPosition(0)
    rightWheel.setPosition(0)
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)

    # Obtener una lista con los sensores infrarrojos ya activados
    irSensorList = enable_distance_sensors(robot, timeStep, INFRARED_SENSORS_NAMES)

    # Obtener sensores de posicion de las ruedas
    posL = robot.getDevice("left wheel sensor")
    posR = robot.getDevice("right wheel sensor")
    posL.enable(timeStep)
    posR.enable(timeStep)

    return robot, leftWheel, rightWheel, irSensorList, posL, posR


# Definir una función para obtener el estado actual
def obtener_estado(infrared_sensors):
    #Salir por la izquierda
    if infrared_sensors[9] > WHITE_LIMIT and infrared_sensors[11] < BLACK_LIMIT:
        print("S1 salir izquierda")
        return 'S1'
    #Salir por la derecha
    elif infrared_sensors[10] > WHITE_LIMIT and infrared_sensors[8] < BLACK_LIMIT:
        print("S2 salir derecha")
        return 'S2'
    #Resto de casos
    else:
        print("S3 resto")
        return 'S3'

def calcular_refuerzo(prev_sensor_values, new_sensor_values):
    # Definir el umbral para detectar el color negro
    BLACK_THRESHOLD = 500

    # Definir los pesos para los sensores del suelo
    FRONT_GROUND_WEIGHT = 1
    MIDDLE_GROUND_WEIGHT = 2

    # Calcular el número de sensores que detectaron el color negro antes y después de la acción
    prev_black_sensors = [i < BLACK_THRESHOLD for i in prev_sensor_values]
    new_black_sensors = [i < BLACK_THRESHOLD for i in new_sensor_values]

    # Calcular el refuerzo para los sensores del suelo delanteros y del medio
    front_ground_refuerzo = FRONT_GROUND_WEIGHT * (sum([new_black_sensors[9], new_black_sensors[10]]) - sum([prev_black_sensors[9], prev_black_sensors[10]]))
    middle_ground_refuerzo = MIDDLE_GROUND_WEIGHT * (sum([new_black_sensors[8], new_black_sensors[11]]) - sum([prev_black_sensors[8], prev_black_sensors[11]]))

    # Normalizar el refuerzo para que esté en el rango de -1 a 1
    refuerzo = (front_ground_refuerzo + middle_ground_refuerzo) / (4 * MIDDLE_GROUND_WEIGHT)

    # Si todos los sensores del suelo detectan el color negro después de la acción, dar un refuerzo positivo
    if sum(new_black_sensors[8:12]) == 4:
        return 0.5

    # De lo contrario, devolver el refuerzo normalizado
    return refuerzo



# Función para elegir una acción
def elegir_accion(estado_actual, epsilon):
    if np.random.random() < epsilon:
        # Exploración: elegir una acción aleatoria
        print("accion aleatoria")
        return np.random.choice(acciones)
    else:
        # Explotación: elegir la acción con el valor Q máximo para el estado actual
        print("ACCION OPTIMA")
        #return np.argmax(Q[estado_actual])
        return acciones[np.argmax(Q[estados.index(estado_actual), :])]


def advance(distancia, leftWheel, rightWheel, posL, posR, robot):
    """
    Hace que el robot avance un poco en linea recta

    leftWheel, rightWheel, posL, posR, robot: dispositivos del robot.
    
    """
    # Calcular el incremento de posición angular para un movimiento recto
    delta = distancia / RADIO_RUEDA

    # Obtener la posición actual de los encoders
    cord_l = posL.getValue()
    cord_r = posR.getValue()

    # Establecer velocidad
    leftWheel.setVelocity(10)
    rightWheel.setVelocity(10)

    # Establecer la nueva posición de los encoders
    leftWheel.setPosition(cord_l + delta)
    rightWheel.setPosition(cord_r + delta)

    # Esperar hasta que el robot alcance la posición deseada
    while abs(posL.getValue() - cord_l) <= delta or abs(posR.getValue() - cord_r) <= delta:
        robot.step(TIME_STEP)
    

def rotate(grados, leftWheel, rightWheel, posL, posR, robot):
    """
    Hace que el robot gire un numero de grados determinados sobre si mismo
    En positivo girara a la derecha, en caso de negativo a la izquierda

    grados: numero de grados a girar
    leftWheel, rightWheel, posL, posR, robot: dispositivos del robot.    
    """
    # Calcular el incremento de posición angular para un giro
    delta = np.radians(np.abs(grados)) * ESPACIO_ENTRE_RUEDAS / (2 * RADIO_RUEDA)

    # Obtener la posición actual de los encoders
    cord_l = posL.getValue()
    cord_r = posR.getValue()

    # Establecer la velocidad
    leftWheel.setVelocity(10)
    rightWheel.setVelocity(10)

    # Establecer la nueva posición de los encoders
    if grados > 0:
        leftWheel.setPosition(cord_l + delta)
        rightWheel.setPosition(cord_r - delta)

        # Esperar hasta que el robot alcance la posición deseada
        while posL.getValue() < cord_l + delta - 0.01 and posR.getValue() > cord_r - delta + 0.01: #Margen de error 0.01
            robot.step(TIME_STEP)
    else:
        leftWheel.setPosition(cord_l - delta)
        rightWheel.setPosition(cord_r + delta)

        # Esperar hasta que el robot alcance la posición deseada 
        while posL.getValue() > cord_l - delta + 0.01 and posR.getValue() < cord_r + delta - 0.01: #Margen de error 0.01
            robot.step(TIME_STEP)

# Función para girar a la izquierda
def turn_left(LM, RM, PosL, PosR, robot):
    rotate(-25, LM, RM, PosL, PosR, robot)
    advance(11, LM, RM, PosL, PosR, robot)
    
# Función para girar a la derecha
def turn_right(LM, RM, PosL, PosR, robot):
    rotate(25, LM, RM, PosL, PosR, robot)
    advance(11, LM, RM, PosL, PosR, robot)

# Función para ir recto
def go_straigth(LM, RM, PosL, PosR, robot):
    advance(25, LM, RM, PosL, PosR, robot)


# Función para ejecutar una acción
def ejecutar_accion(accion, LM, RM, PosL, PosR, robot):
    print("Ejecutando accion: ", accion)
    if accion == 'A1':
        turn_right(LM, RM, PosL, PosR, robot)
    elif accion == 'A2':
        turn_left(LM, RM, PosL, PosR, robot)
    elif accion == 'A3':
        go_straigth(LM, RM, PosL, PosR, robot)

# Esquivar obstáculos -> FALLA A VECES SE QUEDA PILLADO EN LAS ESQUINAS
def esquivar_obstaculos(ir_sensors, leftWheel, rightWheel, robot):
    # Definir el umbral para los sensores infrarrojos
    IR_THRESHOLD = 500

    # Inicializar la posición objetivo de las ruedas
    left_target = 0
    right_target = 0

    # Mientras los sensores frontales detecten un obstáculo
    while (ir_sensors[2].getValue() > IR_THRESHOLD or \
           ir_sensors[3].getValue() > IR_THRESHOLD or \
           ir_sensors[4].getValue() > IR_THRESHOLD):
        # Calcular el offset de velocidad basado en el sensor frontal
        speed_offset = 0.2 * (CRUISE_SPEED - 0.03 * ir_sensors[3].getValue())

        # Calcular la diferencia de velocidad basada en los sensores laterales
        speed_delta = 0.03 * ir_sensors[2].getValue() - 0.03 * ir_sensors[4].getValue()

        # Calcular las velocidades deseadas para las ruedas
        left_speed = speed_offset + speed_delta
        right_speed = speed_offset - speed_delta

        # Calcular las posiciones objetivo para las ruedas basándose en las velocidades deseadas
        left_target += left_speed * TIME_STEP
        right_target += right_speed * TIME_STEP

        # Ajustar las posiciones objetivo de las ruedas para esquivar el obstáculo
        leftWheel.setPosition(left_target)
        rightWheel.setPosition(right_target)

        robot.step(TIME_STEP)


def imprimir_matriz_q(Q, acciones, estados):
    df = pd.DataFrame(Q, index=estados, columns=acciones)
    
    print("------------ MATRIZ Q ------------")
    print(df)



#---------------------------------- MAIN ----------------------------------#
# Inicializar el robot y los dispositivos
robot, leftWheel, rightWheel, irSensorList, posL, posR = init_devices(TIME_STEP)

# Sincronizacion para el primer frame
robot.step(TIME_STEP)

# Inicializar los dispositivos y obtener los valores de los sensores infrarrojos
infrared_values = [sensor.getValue() for sensor in irSensorList]
# Inicializar el estado
estado = obtener_estado(infrared_values)
# Bucle principal de aprendizaje por refuerzo
# Bucle principal de aprendizaje por refuerzo
for episodio in range(NUM_EPISODIOS):
    # Elegir una acción
    accion = elegir_accion(estado, epsilon)
    ejecutar_accion(accion, leftWheel, rightWheel, posL, posR, robot)
    
    # Obtener los valores de los sensores infrarrojos después de la acción
    new_infrared_values = [sensor.getValue() for sensor in irSensorList]
    
    # Ejecutar la acción y obtener el refuerzo
    refuerzo = calcular_refuerzo(infrared_values, new_infrared_values)
    print("Refuerzo: ", refuerzo)
    # Observar el nuevo estado
    nuevo_estado = obtener_estado(new_infrared_values)    
    # Actualizar la matriz Q
    Q[estados.index(estado), acciones.index(accion)] = (1 - alpha) * Q[estados.index(estado), acciones.index(accion)] + alpha * (refuerzo + gamma * np.max(Q[estados.index(nuevo_estado), :]))
    print("Episodio: ", episodio)
    print("Epsilon: ", epsilon)
    imprimir_matriz_q(Q, acciones, estados)
    # Actualizar el estado y los valores de los sensores infrarrojos
    estado = nuevo_estado
    infrared_values = new_infrared_values

    # Esquivar obstáculos
    esquivar_obstaculos(irSensorList, leftWheel, rightWheel, robot)

    # Reducir el valor de epsilon para cambiar de exploración a explotación
    if epsilon > 0:
        epsilon -= 1/500

# Detener el robot
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)