#!/usr/bin/env python3
# -*- coding: utf-8 -*-




#==================================================
## @file lib_com
## @author Takumi FUJI
## @brief ライブラリクラス
#==================================================




#==================================================
# import
#==================================================
import sys
import roslib
import socket
import websockets
import asyncio

sys.path.append(roslib.packages.get_pkg_dir("robot_pkg") + "/script/import")
from common_import import *

#==================================================
# グローバル
#==================================================




#==================================================
## @class LibCom
## @brief 自作ライブラリクラス
#==================================================
class LibCom:
    #==================================================
    ## @fn __init__
    ## @brief コンストラクタ
    ## @param 
    ## @return
    #==================================================
    def __init__(
        self
    ):
        #==================================================
        # メンバ変数
        #==================================================
        self._lib = {
        }


        #==================================================
        # ROSインタフェース
        #==================================================
        ##2024/10/21 川原 追記
        #self.dir_list = [2,3,0,2,0]#0:直進、2:右、3:左、進行方向リスト
        self.dir_list = [2,0,2,0,3]#0:直進、2:右、3:左、進行方向リスト10/29
        self.dir_num = 0
        self.act = 0
        


        #==================================================
        # イニシャライズ
        #==================================================


        return


    #==================================================
    ## @fn initSocket
    ## @brief ソケット通信を確立する
    ## @param 
    ## @return
    #==================================================
    """def initSocket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(("172.17.6.79", 52350))  # 自身のIPアドレスとポート番号を指定
        print("waiting...")
        self.sock.listen(5)
        self.clientsocket, self.address = self.sock.accept()
        print(f"Connection from {self.address} has been established!")
        self.clientsocket.settimeout(15)


    #==================================================
    ## @fn readSocket
    ## @brief ソケット通信内容を取得する
    ## @param 
    ## @return
    #==================================================
    def readSocket(self):
        recvline = self.clientsocket.recv(1024).decode()
        print(recvline)
        return recvline


    #==================================================
    ## @fn readSocketConv
    ## @brief ソケット通信内容を取得して上下右左リストに変換
    ## @param ^v><がそれぞれ上下右左に対応
    ## @return
    #==================================================
    def readSocketConv(self):
        try:
            recvline = self.clientsocket.recv(1024).decode()
        except socket.timeout:
            return None
        
        print(recvline)
        if recvline == "^":
            data = [1,0,0,0]
        elif recvline == "v":
            data = [0,1,0,0]
        elif recvline == ">":
            data = [0,0,1,0]
        elif recvline == "<":
            data = [0,0,0,1]
        elif recvline == "exit":
            data = recvline
        else:
            print("Socket error!")
        return data"""

    #==================================================
    ## @fn com_recv
    ## @brief Websockets受信時に呼ばれる
    ## @param
    ## @return
    #==================================================
    async def com_recv(self, websocket, path):
        self.datas = await websocket.recv()
        print(f"datas = {self.datas}")
        return self.datas
        
    #==================================================
    ## @fn skts_main
    ## @brief Websocketsの接続を設定
    ## @param
    ## @return
    #==================================================
    async def skts_main(self):
        async with websockets.serve(self.com_recv, "172.17.6.210", 52350):            
            await asyncio.Future()  # run forever
            
            
    #==================================================
    ## @fn waitSockets
    ## @brief Websocketsの開始    
    ## @param
    ## @return
    #==================================================
    def waitSockets(
        self,
        timeout = 30
    ):
        # 開始の時刻を保存
        start_time = time.time()
        # 経過した時刻を取得
        end_time = time.time()

        # resultにwebsocketsの値を格納 timeout秒でbreak
        self.datas = None
        result = None
        asyncio.run(self.skts_main())
        while end_time - start_time <= timeout:
            if self.datas == "<":
                result = [0,0,0,1] # [上、下、右、左]
                self.datas = None
                break
                
            elif self.datas == ">":
                result = [0,0,1,0]
                self.datas = None
                break
        
        return result
        
    #==================================================
    ## @fn readSockets
    ## @brief Websocketsの記録されたtxt読み込み    
    ## @param
    ## @return
    #==================================================
    def readSockets(
        self,
        loc,
        latency = 3
    ):
        time.sleep(latency)
        result = None
        print("opening file...")
        f = open(loc + '/class.txt','r')
        data = f.read()
        print(f"data = {data}")
        if data == "0":
            print("command = 0")
            result = [0,0,0,1]
                
        elif data == "1":
            print("command = 1")
            result = [0,0,1,0]
            
        f.close() 
        
        return result
        
    #==================================================
    ## @fn readSocketsDirection
    ## @brief リスト順に進行   
    ## @param
    ## @return
    ##2024/10/21 川原 追記
    #==================================================
    def readSocketsDirection(
        self,
        loc,
        latency = 3
    ):
        time.sleep(latency)
        result = None
        '''
        print("opening file...")
        f = open(loc + '/class.txt','r')
        data = f.read()
        print(f"data = {data}")
        if data == "0":
            print("command = 0")
            result = [0,0,0,1]
                
        elif data == "1":
            print("command = 1")
            result = [0,0,1,0]
            
        f.close() 
        '''
        num = self.dir_list[self.dir_num]
        print("dir_num=", num)
        if num == 0:
        	result = [1,0,0,0]
        elif num == 2:
        	result = [0,0,1,0]
        elif num == 3:
        	result = [0,0,0,1]
        	
        self.dir_num += 1
        
        return result
        
     #==================================================
    ## @fn readSocketsFire
    ## @brief Websocketsの記録されたtxt読み込み    
    ## @param
    ## @return
    ##2024/10/21 川原 追記
    #==================================================
    def readSocketsFire(
        self,
        loc,
        latency = 3
    ):
        time.sleep(latency)
        result = None
        '''
        print("opening file...")
        f = open(loc + '/class.txt','r')
        data = f.read()
        print(f"data = {data}")
        if data == "0":
            print("command = 0")
            result = [0,0,0,1]
                
        elif data == "1":
            print("command = 1")
            result = [0,0,1,0]
            
        f.close() 
        '''
        self.act += 1
        result = self.act % 2
        
        return result
    
    #==================================================
    ## @fn delete
    ## @brief デストラクタ
    ## @param
    ## @return
    #==================================================
    def delete(
        self
    ):
        #==================================================
        # ファイナライズ
        #==================================================
        #self.clientsocket.close()
        #self.sock.close()

        return


