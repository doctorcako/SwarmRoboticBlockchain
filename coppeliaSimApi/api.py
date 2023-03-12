import sim
import time
import sys
import requests
import threading
import firebase_admin 
from firebase_admin import messaging, credentials, firestore, db
import os

def connect(port):
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
    if clientID == 0: print('Connected to remote API server at port {}'.format(port))
    else: print('Failed connecting to remote API server at port {}'.format(port))
    return clientID

def robot_handle(robotVisionSensorNumber, port, db, route):
    clientID = connect(int(port))
    err, obj = sim.simxGetObjectHandle(clientID, 'Vision_sensor' + str(robotVisionSensorNumber), sim.simx_opmode_oneshot_wait)
    err, resolution, image = sim.simxGetVisionSensorImage(clientID, obj, 0, sim.simx_opmode_streaming)
    situationDetected = False
    # Get the handle of the object
    while sim.simxGetConnectionId(clientID) != -1 and situationDetected==False:
        err, resolution, image = sim.simxGetVisionSensorImage(clientID, obj, 0, sim.simx_opmode_buffer)
        
        if err == sim.simx_return_ok:
            if image[0] != 0 and image[1]==0 and image[2]==0: #red
                print('Color red detected')
                eventType = 'accident'
                db.collection(u'road_situations').document(route).update({u'accident':True})

            if image[0] == 0 and image[1]!=0 and image[2]==0: #green
                print('Color green detected')
                eventType = 'environment_difficulties'
                db.collection(u'road_situations').document(route).update({u'environment_difficulties':True})
            
            if image[0] == 0 and image[1]==0 and image[2]!=0: #blue
                print('Color blue detected')
                eventType = 'retention'
                db.collection(u'road_situations').document(route).update({u'retention':True})

            time.sleep(10)


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


def roadSituationEventControl(db, apiAddress):
    callback_done = threading.Event()
    def on_snapshot(col_snapshot, changes, read_time):
        # print(u'WARNING!')
        eventType = ''
        for change in changes:
            if change.type.name == 'MODIFIED' or change.type.name == 'ADDED':
                docName = change.document.id
                print(docName)
                if change.document.get('accident') == True:
                    eventType = 'accident'
                    print('Posible Accident!')
                    print('--------------------------')

                elif change.document.get('retention') == True:
                    eventType = 'retention'
                    print('Posible Retention!')
                    print('--------------------------')

                elif change.document.get('environment_difficulties') == True:
                    eventType = 'environment_difficulties'
                    print('Posible environment difficulties!')
                    print('--------------------------')

                db.collection(u'road_situations').document(docName).update({u'retention':False, u'accident': False, u'environment_difficulties':False})

                data = {
                    'eventType':eventType,
                    'route':docName,
                    'triggererId':'0x123456798'
                }

                response=requests.post('http://'+apiAddress+':3000/registerEvent',json=data)
                print(read_time)

        callback_done.set()
        
    col_query_accident = db.collection(u'road_situations').where(u'accident', u'==', True)
    query_watch_accident = col_query_accident.on_snapshot(on_snapshot)

    col_query_retention = db.collection(u'road_situations').where(u'retention', u'==', True)
    query_watch_retention = col_query_retention.on_snapshot(on_snapshot)

    col_query_environment_difficulties = db.collection(u'road_situations').where(u'environment_difficulties', u'==', True)
    query_watch_environment = col_query_environment_difficulties.on_snapshot(on_snapshot)



def main(argv):
    cred = credentials.Certificate(os.getcwd()+'/firebase_key2.json')
    app = firebase_admin.initialize_app(cred)
    db = firestore.client()

    if len(argv)>1 and argv[1] == '--refresh':
        initialize_firestore(db)
    else:
        print('---- You can restart the road situations simulation data with `--refresh` ----')
        roadSituationEventControl(db, argv[2])

        # robot_handle(argv[2],argv[1],db)

        if(argv[1]) == 'robot1':
            print('robot1')
            road_situations = db.collection(u'road_situations').document('AP-7').update({u'retention':True})
            time.sleep(3)
        
        if(argv[1]) == 'robot2':
            print('robot2')
            road_situations = db.collection(u'road_situations').document('M-40').update({u'accident':True})
            time.sleep(3)
        # # print('update AP-7 accident true')
        # road_situations = db.collection(u'road_situations').document(u'N-333').update({u'accident':True})
        # time.sleep(5)

        # # print('update AP-7 environment true')
        # road_situations = db.collection(u'road_situations').document(u'M-40').update({u'environment_difficulties':True})
        # time.sleep(5)

        # road_situations = db.collection(u'road_situations').document(u'N-333').update({u'environment_difficulties':True})
        # time.sleep(5)



    # time.sleep(10)
    print('Done')
 
main(sys.argv)