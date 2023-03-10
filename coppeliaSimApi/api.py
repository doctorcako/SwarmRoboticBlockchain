import sim
import numpy as np
import time
import sys
import requests
import threading

def connect(port):
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
    if clientID == 0: print('Connected to remote API server at port {}'.format(port))
    else: print('Failed connecting to remote API server at port {}'.format(port))
    return clientID

def robot_handle(robotVisionSensorNumber, port, eventObj):
    clientID = connect(int(port))
    err, obj = sim.simxGetObjectHandle(clientID, 'Vision_sensor' + str(robotVisionSensorNumber), sim.simx_opmode_oneshot_wait)
    err, resolution, image = sim.simxGetVisionSensorImage(clientID, obj, 0, sim.simx_opmode_streaming)
    situationDetected = False
    # Get the handle of the object
    while sim.simxGetConnectionId(clientID) != -1 and situationDetected==False:
        err, resolution, image = sim.simxGetVisionSensorImage(clientID, obj, 0, sim.simx_opmode_buffer)
        if err == sim.simx_return_ok:
            if image[0] != 0 and image[1]==0 and image[2]==0:
                eventObj.set()
                print('Color red detected: ¡POSIBLE ACCIDENT!')
                # response=requests.get('http://localhost:3000/')
                # print(response.json())
                situationDetected = True
    
    time.sleep(5)

def initialize_environment(argv,robots, eventHandlers):
    for port in argv:
        eventObj = threading.Event()
        eventHandlers.append(eventObj)
        robot = threading.Thread(target=robot_handle, args=(0,port, eventObj))
        robots.append(robot)

def start_robots_threads(robots):
    for robot in robots:
        robot.start()

def main(argv):
    robots = []
    eventHandlers = []
    initialize_environment(argv[1:], robots, eventHandlers)
    
    print(robots)
    print(eventHandlers)

    start_robots_threads(robots)
    # robot1 = threading.Thread(target=robot_handle, args=(0,argv[1], eventObj))
    # robot1.start()
    
    while(True):
        for eventHandler in eventHandlers:
            if(eventHandler.is_set()):
                robots[0].join()
                # robots[robots.index(eventHandler)].join()
                print('Slowing down cars speed')

                eventHandler.clear()
                robots[0].start()
                break  

           
            

    # while(not eventObj.is_set()):
    #     if(eventObj.is_set()):
    #         print('Slowing down cars speed')
    #         robot1.join()
   
        # robot1.join()
        # robot1.start()



    

    # robot2 = threading.Thread(target=robot_handle, args=(0,argv[1]))
    # robot2.start()

    # robot3 = threading.Thread(target=robot_handle, args=(0,argv[1]))
    # robot3.start()

    print('goin on with the execution')
 
main(sys.argv)