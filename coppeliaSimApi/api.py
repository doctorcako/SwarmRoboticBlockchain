import sim
import time
import sys
import requests
import threading
import firebase_admin
from firebase_admin import messaging, credentials, firestore, db
import os
from datetime import datetime

def connect(port):
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
    if clientID == 0: print('Connected to remote API server at port {}'.format(port))
    else: print('Failed connecting to remote API server at port {}'.format(port))
    return clientID

def robot_handle(clientID, robotNumber, db, route, apiAddress, keys, location, infoVerified):

    err, obj = sim.simxGetObjectHandle(clientID, 'Vision_sensor' + str(robotNumber), sim.simx_opmode_oneshot_wait)

    err, resolution, image = sim.simxGetVisionSensorImage(clientID, obj, 0, sim.simx_opmode_streaming)
    situationDetected = False
    eventType = 'CLEAN'
    getData = {
                'location': location,
                'routeName': route,
                'nodeRPC': keys['sender']['url'],
                'fromPrivateKey': keys['sender']['accountPrivateKey'],
                'fromPublicKey': keys['sender']['publicKey'],
                'toPublicKey': keys['publicKeys']
            }
    # Get the handle of the object

    routeStatus = 'Unknown'
    try:
        response=requests.post('http://'+apiAddress+'/getRouteInfo',json=getData)
    except Exception as e:
        print('Error: '+str(e))
        print('You may not have permissions to make transactions or cannot connect to the simulator.')
    routeStatus = response.json()['status']

    while sim.simxGetConnectionId(clientID) != -1 :

            # if statusUpdated:
        
        if(routeStatus == 'CLEAN' and infoVerified == True):
            print(datetime.now().strftime("%H:%M:%S")+': Road clean for user')
            res = sim.simxCallScriptFunction(clientID, 'Robot' + str(robotNumber), sim.sim_scripttype_childscript, 'reduceSpeed', [], [1], [], '', sim.simx_opmode_blocking)
            situationDetected = False
            infoVerified = False

        if(routeStatus == 'ACCIDENT' and infoVerified == True):
            print(datetime.now().strftime("%H:%M:%S.%f")+': Reducing speed of all members, dangerous situation detected verified by blockchain system!')
            res = sim.simxCallScriptFunction(clientID, 'Robot' + str(robotNumber), sim.sim_scripttype_childscript, 'reduceSpeed', [], [0.25], [], '', sim.simx_opmode_blocking)
            situationDetected = False
            infoVerified = False

        if(routeStatus == 'RETENTION' and infoVerified == True):
            print(datetime.now().strftime("%H:%M:%S.%f")+': Reducing speed of all members, dangerous situation detected verified by blockchain system!')
            res = sim.simxCallScriptFunction(clientID, 'Robot' + str(robotNumber), sim.sim_scripttype_childscript, 'reduceSpeed', [], [0.5], [], '', sim.simx_opmode_blocking)
            situationDetected = False
            infoVerified = False

        if(routeStatus == 'ENVIRONMENT_DIFFICULTIES' and infoVerified == True):
            print(datetime.now().strftime("%H:%M:%S.%f")+': Reducing speed of all members, dangerous situation detected verified by blockchain system!')
            res = sim.simxCallScriptFunction(clientID, 'Robot' + str(robotNumber), sim.sim_scripttype_childscript, 'reduceSpeed', [], [0.65], [], '', sim.simx_opmode_blocking)
            situationDetected = False
            infoVerified = False


        err, resolution, image = sim.simxGetVisionSensorImage(clientID, obj, 0, sim.simx_opmode_buffer)
        if err == sim.simx_return_ok and infoVerified == False:
            if image[0] != 0 and image[1]==0 and image[2]==0 and routeStatus != 'ACCIDENT': #red
                # print('Color red detected')
                situationDetected = True
                eventType = 'ACCIDENT'
                print(datetime.now().strftime("%H:%M:%S.%f")+ ': Emmitin event accident for emergency braking')
                db.collection(u'road_situations').document(route).update({u'accident':True})

            if image[0] == 0 and image[1]!=0 and image[2]==0 and routeStatus != 'RETENTION': #green
                # print('Color green detected')
                situationDetected = True
                eventType = 'RETENTION'
                print(datetime.now().strftime("%H:%M:%S.%f")+ ': Emmitin event retention for emergency braking')
                db.collection(u'road_situations').document(route).update({u'retention':True})

            if image[0] == 0 and image[1]==0 and image[2]!=0 and routeStatus != 'ENVIRONMENT_DIFFICULTIES': #blue
                # print('Color blue detected')
                situationDetected = True
                eventType = 'ENVIRONMENT_DIFFICULTIES'
                print(datetime.now().strftime("%H:%M:%S.%f")+ ': Emmitin event environment difficulties for emergency braking')
                db.collection(u'road_situations').document(route).update({u'environment_difficulties':True})

            if image[0] != 0 and image[1] != 0 and image[2] != 0 and routeStatus != 'CLEAN':
                situationDetected = True
                eventType = 'CLEAN'

            if situationDetected == True and infoVerified == False:
                nodeData = keys['sender']
                data = {
                    'eventType':eventType,
                    'location': location,
                    'routeName': route,
                    'nodeRPC': nodeData['url'],
                    'fromPrivateKey': nodeData['accountPrivateKey'],
                    'fromPublicKey': nodeData['publicKey'],
                    'toPublicKey': keys['publicKeys']
                }
                try:
                    response=requests.post('http://'+apiAddress+'/updateStatus',json=data)
                    
                except Exception as e:
                    print('Error: '+str(e))
                    print('You may not have permissions to make transactions or cannot connect to the simulator.')

            response2 = 'Unknown'
           
            if infoVerified == False:
                try:
                    response2 = requests.post('http://'+apiAddress+'/getRouteInfo',json=getData)
                    infoVerified = True
                    
                except Exception as e:
                    print('Error: '+str(e))
                    print('You may not have permissions to make transactions or cannot connect to the simulator.')

                routeStatus = response2.json()['status']


def initialize_firestore(db):

    for doc in db.collection(u'road_situations').list_documents(page_size=4):
        doc.delete()

    road_situations = db.collection(u'road_situations').document(u'AP-7')
    road_situations.set({
        u'accident': False,
        u'environment_difficulties':False,
        u'retention':False
    })

    road_situations = db.collection(u'road_situations').document(u'M-40')
    road_situations.set({
        u'accident': False,
        u'environment_difficulties':False,
        u'retention':False
    })

    road_situations = db.collection(u'road_situations').document(u'N-333')
    road_situations.set({
        u'accident': False,
        u'environment_difficulties':False,
        u'retention':False
    })


def roadSituationEventControl(db, apiAddress, keys, location, clientID, robotNumber, infoVerified):
    callback_done = threading.Event()
    def on_snapshot(col_snapshot, changes, read_time):
        for change in changes:
            if change.type.name == 'MODIFIED' or change.type.name == 'ADDED':
                docName = change.document.id
                infoVerified = False
                print('--------------------------')
                if change.document.get('accident') == True:
                    
                    print(datetime.now().strftime("%H:%M:%S.%f")+': Posible Accident!')
                    print('--------------------------')
                    # print(time.time()*1000)
                    sim.simxCallScriptFunction(clientID, 'Robot' + str(robotNumber), sim.sim_scripttype_childscript, 'reduceSpeed', [], [0.52], [], '', sim.simx_opmode_blocking)

                elif change.document.get('retention') == True:

                    print(datetime.now().strftime("%H:%M:%S.%f")+': Posible Retention!')
                    print('--------------------------')
                    sim.simxCallScriptFunction(clientID, 'Robot' + str(robotNumber), sim.sim_scripttype_childscript, 'reduceSpeed', [], [0.65], [], '', sim.simx_opmode_blocking)

                elif change.document.get('environment_difficulties') == True:

                    print(datetime.now().strftime("%H:%M:%S.%f")+': Posible environment difficulties!')
                    sim.simxCallScriptFunction(clientID, 'Robot' + str(robotNumber), sim.sim_scripttype_childscript, 'reduceSpeed', [], [0.75], [], '', sim.simx_opmode_blocking)
                    print('--------------------------')

                db.collection(u'road_situations').document(docName).update({u'retention':False, u'accident': False, u'environment_difficulties':False})
                # res = sim.simxCallScriptFunction(clientID, 'Robot' + str(robotNumber), sim.sim_scripttype_childscript, 'reduceSpeed', [], [0.52], [], '', sim.simx_opmode_blocking)

        callback_done.set()


    col_query_accident = db.collection(u'road_situations').where(u'accident', u'==', True)
    query_watch_accident = col_query_accident.on_snapshot(on_snapshot)

    col_query_retention = db.collection(u'road_situations').where(u'retention', u'==', True)
    query_watch_retention = col_query_retention.on_snapshot(on_snapshot)

    col_query_environment_difficulties = db.collection(u'road_situations').where(u'environment_difficulties', u'==', True)
    query_watch_environment = col_query_environment_difficulties.on_snapshot(on_snapshot)

def getRoutesAndSelectOne(keys, apiIp):
    nodeData = keys['sender']
    data = {
        'nodeRPC': nodeData['url'],
        'fromPrivateKey': nodeData['accountPrivateKey'],
        'fromPublicKey': nodeData['publicKey'],
        'toPublicKey': keys['publicKeys']
    }
    try:
        response=requests.post('http://'+apiIp+'/getRoutes',json=data)
        routes = response.json()

        for i in range(len(routes)):
            print(str(i+1)+'. '+routes[i]['routeName'])

        selectedRoute = input('Select a route: ')
        routeSelected = routes[int(selectedRoute)-1]['routeName']

        name = routeSelected.split(' - ')[0]
        location = routeSelected.split(' - ')[1]
        data = {
            'location': location,
            'routeName': name,
            'nodeRPC': nodeData['url'],
            'fromPrivateKey': nodeData['accountPrivateKey'],
            'fromPublicKey': nodeData['publicKey'],
            'toPublicKey': keys['publicKeys']
        }
        response = requests.post('http://'+apiIp+'/enterRoute',json=data)
    except Exception as e:
        print('Error: '+str(e))
        print('You may not have permissions to make transactions or cannot connect to the simulator.')
        sys.exit()

    return [location, name]


# to run the robot autonomous simulation use the following command: python3 api.py 'robotNumber' localhost:4000 'privateKey'
# example: python3 api.py 0 localhost:4000 0x12345


def main(argv):
    cred = credentials.Certificate(os.getcwd()+'/firebase_key2.json')
    app = firebase_admin.initialize_app(cred)
    db = firestore.client()
    infoVerified = False
    if len(argv)>1 and argv[1] == '--refresh':
        initialize_firestore(db)
    else:
        print('---- You can restart the road situations simulation data with `--refresh` ----')
        data = {
                    'nodeAddress': argv[3]
                }

        apiIp = argv[2]
        robotSimulatorPort = argv[4]
        robotNumber = argv[1]

        keys=requests.post('http://'+apiIp+'/getKeys',json=data)
        keys = keys.json()
        try:
            location, name = getRoutesAndSelectOne(keys, apiIp)
            clientID = connect(int(robotSimulatorPort))
            roadSituationEventControl(db, apiIp, keys, location, clientID, robotNumber, infoVerified)

            robot_handle(clientID, robotNumber, db, name, apiIp, keys, location, infoVerified)
        except Exception as e:
            print('Error: '+str(e))
            print('You may not have permissions to make transactions or cannot connect to the simulator.')

    print('Done')


main(sys.argv)