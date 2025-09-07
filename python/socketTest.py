import socketio
import time
import asyncio

sio : socketio.AsyncClient = socketio.AsyncClient(reconnection=True,reconnection_attempts=3,reconnection_delay_max=5)

async def async_main():
    print("start")
    
    await sio.connect('http://localhost:4000')
    time.sleep(3)

@sio.event
def connect():
    print("I'm connected!")

@sio.event
def connect_error(data):
    print("The connection failed!")

@sio.event
def disconnect(reason):
    print("I'm disconnected! reason:", reason)

while(True) :
    asyncio.run(async_main())

