import sim
import numpy as np
import time
import sys
import requests
import firebase_admin 
from firebase_admin import messaging

cred = firebase_admin.credentials.Certificate('firebase_key.json')
firebase_admin.initialize_app(cred)

topic = 'road_situations'
messaging.subscribe_to_topic('road_situations','road_situations')

def callback(message):
    print('Received message:',message) 

listener = messaging.Listener(callback)

listener.start()

while True:
    pass
    


main()