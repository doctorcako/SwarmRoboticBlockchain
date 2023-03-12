import sim
import numpy as np
import time
import sys
import requests
import firebase_admin 
from firebase_admin import messaging, credentials, firestore

cred = credentials.Certificate('/home/cako/swarmRoboticBlockchain/coppeliaSimApi/firebase_key2.json')
    
app = firebase_admin.initialize_app(cred)

db = firestore.client()

robots = db.collection(u'robots').document(u'node1')
robots.set({
    u'address':u'1265846546'
})


def initialize_firestore():
    
