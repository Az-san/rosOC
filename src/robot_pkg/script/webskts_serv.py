#!/usr/bin/env python3

import asyncio
import websockets

async def com_recv(websocket, path):
    comando = await websocket.recv()
    print(f"comando = {comando}")
    f = open('/home/user/rosOC/src/robot_pkg/script/class.txt','w')
    f.write(f"{comando}")
    f.close()

async def main():
    async with websockets.serve(com_recv, "172.17.6.210", 52358): #受け付けるipアドレスとポート
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())
	    
