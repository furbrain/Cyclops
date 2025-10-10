import time
from xmlrpc.client import ServerProxy

with ServerProxy('http://localhost:8123') as server:
    server.beep([("C3",5000)])
    time.sleep(1)
    server.beep([("C5", 50), ("E5", 50),("G5", 50),("C6", 50)])