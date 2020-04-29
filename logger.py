'''
    logger.py

    Writes a .txt log with the json of the world state
'''

from datetime import datetime
import json
import os
import sys

class Logger :
    def __init__(self, path) :
        now = datetime.now()
        filename = now.strftime("%Y%m%d_%H%M%S.txt")
        print("Creating Logger: ", filename)
        if not os.path.exists(path):
            os.makedirs(path)
        self.file = open(os.path.join(path, filename), 'x')

    def update_log(self, string) :
        self.file.write(string)

    def close_log(self) :
        print("Closing Logger")
        self.file.close()

#Testing Driver
dict = {
    'test' : 43,
    'ytho' : 'mesayso'
}

dict_json = json.dumps(dict)
path = "/home/jarrettphilips/Desktop/marion_ws/src/marion_pkg/src/system_management/logs"

log = Logger(path)
log.update_log(dict_json + '\n')
log.update_log(dict_json + '\n')
log.close_log()