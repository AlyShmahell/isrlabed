import time
import zmq
import random

context = zmq.Context()
consumer_receiver = context.socket(zmq.SUB)    

consumer_receiver.setsockopt(zmq.CONFLATE, 1)

consumer_receiver.connect("tcp://127.0.0.1:5555") 
consumer_receiver.subscribe(b'')


while 1:
    d=random.randint(1,10)

    work = consumer_receiver.recv()

    print(work,"  :",d)

    time.sleep(d)