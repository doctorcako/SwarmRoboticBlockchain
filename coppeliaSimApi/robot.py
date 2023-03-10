import sim
import numpy as np
import time
import sys
import requests
import firebase_admin from firebase_admin import credentials, messaging

def connect(port):
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
    if clientID == 0: print('Connected to remote API server at port {}'.format(port))
    else: print('Failed connecting to remote API server at port {}'.format(port))
    return clientID

def robot_handle(robotVisionSensorNumber, port):
    clientID = connect(int(port))
    err, obj = sim.simxGetObjectHandle(clientID, 'Vision_sensor' + str(robotVisionSensorNumber), sim.simx_opmode_oneshot_wait)
    err, resolution, image = sim.simxGetVisionSensorImage(clientID, obj, 0, sim.simx_opmode_streaming)
    situationDetected = False
    # Get the handle of the object
    while sim.simxGetConnectionId(clientID) != -1 and situationDetected==False:
        err, resolution, image = sim.simxGetVisionSensorImage(clientID, obj, 0, sim.simx_opmode_buffer)
        if err == sim.simx_return_ok:
            if image[0] != 0 and image[1]==0 and image[2]==0:
                print('Color red detected: ¡POSIBLE ACCIDENT!')
                # response=requests.get('http://localhost:3000/')
                # print(response.json())
                send_message(robotVisionSensorNumber)
                situationDetected = True

def send_message(robot):
    message = messaging.Message(
        data={
            'title': 'Accident detected',
            'body': {'robot':robot,'content':'Accident detected in the road'},
        },
        topic='road_situations'
    )
    response = messaging.send(message)
        
    

def main(argv):
    cred = credentials.Certificate('firebase_key.json')
    firebase_admin.initialize_app(cred)
    robotHandle(argv[2],argv[1])
    print('goin on with the execution')

main(sys.argv)
