#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#==================================================
## @file sm_main.py
## @original author Kentaro NAKAMURA
## @author Takumi FUJI
## @brief メインステートマシン
#==================================================


#==================================================
# import
#==================================================
import sys
import roslib
import rospy
import os

import smach
import subprocess

sys.path.append(roslib.packages.get_pkg_dir("robot_pkg") + "/script/import")
from common_import import *

#==================================================
# グローバル
#==================================================
CheckIntersectionCLOCK = 0.5


#==================================================
## @class Init
## @berif 初期化ステート
#==================================================
class Init(
    smach.State
):
    #==================================================
    ## @fn __init__
    ## @brief コンストラクタ
    ## @param lib ライブラリ群
    ## @return
    #==================================================
    def __init__(
        self,
        lib = {}
    ):
        #==================================================
        # イニシャライズ
        #==================================================
        smach.State.__init__(
            self,
            outcomes = ["next", "except"]
        )

        self._lib = lib

        return




    #==================================================
    ## @fn __del__
    ## @brief デストラクタ
    ## @param
    ## @return
    #==================================================
    def __del__(self):
        #==================================================
        # ファイナライズ
        #==================================================

        return




    #==================================================
    ## @fn execute
    ## @brief 実行関数
    ## @param userdata
    ## @return
    #==================================================
    def execute(
        self,
        userdata
    ):
        #ソケット通信確立
        #if SocketActive:
            #self._lib["com"].initSocket()
            
        #実験情報入力
        return "next"




#==================================================
## @class Wait4Start
## @berif 開始待ちステート
#==================================================
class Wait4Start(
    smach.State
):
    #==================================================
    ## @fn __init__
    ## @brief コンストラクタ
    ## @param lib ライブラリ群
    ## @return
    #==================================================
    def __init__(
        self,
        lib = {}
    ):
        #==================================================
        # イニシャライズ
        #==================================================
        smach.State.__init__(
            self,
            outcomes = ["next", "except"]
        )

        self._lib = lib

        return




    #==================================================
    ## @fn __del__
    ## @brief デストラクタ
    ## @param
    ## @return
    #==================================================
    def __del__(self):
        #==================================================
        # ファイナライズ
        #==================================================

        return




    #==================================================
    ## @fn execute
    ## @brief 実行関数
    ## @param userdata
    ## @return
    #==================================================
    def execute(
        self,
        userdata
    ):
        commando = subprocess.Popen(['python3','/home/user/rosOC/src/robot_pkg/script/webskts_serv.py'])
        input("Enter to START >>> ")

        return "next"




#==================================================
## @class Start
## @brief 開始ステート
#==================================================
class Start(
    smach.State
):
    #==================================================
    ## @fn __init__
    ## @brief コンストラクタ
    ## @param lib ライブラリ群
    ## @return
    #==================================================
    def __init__(
        self,
        lib = {}
    ):
        #==================================================
        # イニシャライズ
        #==================================================
        smach.State.__init__(
            self,
            outcomes = ["next", "except"]
        )

        self._lib = lib

        return



    #==================================================
    ## @fn __del__
    ## @brief デストラクタ
    ## @param
    ## @return
    #==================================================
    def __del__(self):
        #==================================================
        # ファイナライズ
        #==================================================

        return




    #==================================================
    ## @fn execute
    ## @brief 実行関数
    ## @param userdata
    ## @return
    #==================================================
    def execute(
        self,
        userdata
    ):
        
        return "next"



#==================================================
## @class StartTrial
## @berif 
#==================================================
class StartTrial(
    smach.State
):
    #==================================================
    ## @fn __init__
    ## @brief コンストラクタ
    ## @param lib ライブラリ群
    ## @return
    #==================================================
    def __init__(
        self,
        lib = {}
    ):
        #==================================================
        # イニシャライズ
        #==================================================
        smach.State.__init__(
            self,
            outcomes = ["next", "restart", "loop", "except"]
        )

        self._lib = lib


        #==================================================
        # ROSインタフェース
        #==================================================


        return




    #==================================================
    ## @fn __del__
    ## @brief デストラクタ
    ## @param
    ## @return
    #==================================================
    def __del__(self):
        #==================================================
        # ファイナライズ
        #==================================================

        return



    #==================================================
    ## @fn execute
    ## @brief 実行関数
    ## @param 
    ## @return
    #==================================================
    def execute(
        self,
        userdata
    ):

        self._lib["record"].initPath(rospy.get_param("/exp_type", "")) # データ保存先の初期化
                
        #self._lib["record"].saveInfo(SocketActive,TTLActive) # 実験情報のテキストファイル保存
        
        #スタート時点の地図の保存
        self._lib["record"].saveMap()

        # GUIの変更
        point_pixel = self._lib["map"].getRobotPointPixel()
        self._lib["if"].changeGUI(point_pixel)
        
        rospy.set_param(rospy.get_name() + "/inf_rad_sorted", []) #進行可能方角
        rospy.set_param(rospy.get_name() + "/position_meter", []) #現在位置・角度
        rospy.set_param(rospy.get_name() + "/nearest_inf_rad_to_present_rad", 0.0) #正面に向けるための後進方向

        print("###    EEG  experiment    ###")
        if rospy.get_param("/data_file", "") == "":
            rospy.set_param(rospy.get_name() + "/q_table", [])
            rospy.set_param(rospy.get_name() + "/x_table", [])
        else:
            save_date_path = roslib.packages.get_pkg_dir("robot_pkg") + "/data" + "/" + rospy.get_param("/data_file", "")
            q_table = self._lib["record"].loadRecordQTable(save_date_path)
            x_table = self._lib["record"].loadRecordXTable(save_date_path)
            rospy.set_param(rospy.get_name() + "/q_table", q_table)
            rospy.set_param(rospy.get_name() + "/x_table", x_table)
        
        rospy.sleep(1)
        # ストップウォッチを開始する
        self._lib["record"].recordTime(state="start")

        return "next"





#==================================================
## @class CheckEEG
#==================================================
class CheckEEG(
    smach.State
):
    #==================================================
    ## @fn __init__
    ## @brief コンストラクタ
    ## @param lib ライブラリ群
    ## @return
    #==================================================
    def __init__(
        self,
        lib = {}
    ):
        #==================================================
        # イニシャライズ
        #==================================================
        smach.State.__init__(
            self,
            outcomes = ["false", "true", "except"]
        )

        self._lib = lib


        #==================================================
        # ROSインタフェース
        #==================================================

        return



    #==================================================
    ## @fn __del__
    ## @brief デストラクタ
    ## @param
    ## @return
    #==================================================
    def __del__(self):
        #==================================================
        # ファイナライズ
        #==================================================

        return



    #==================================================
    ## @fn execute
    ## @brief 実行関数
    ## @param 
    ## @return
    #==================================================
    def execute(
        self,
        userdata
    ):
        inf_rad_sorted = self._lib["scan"].getLaserInfRad()
        position_meter = self._lib["map"].getRobotPointMeter()

        rospy.set_param(rospy.get_name() + "/inf_rad_sorted", inf_rad_sorted)

        # GUIの変更
        self._lib["if"].changeGUI("move")
        
        path = roslib.packages.get_pkg_dir("robot_pkg") + "/script"
        eyeIO = []
        eyeIO = self._lib["com"].readSockets(loc = path, latency = 0)
        print(f"eye = {eyeIO}")
        rospy.sleep(0.1)

        # 進行可能方向がない場合
        if len(inf_rad_sorted) == 0:
            rospy.loginfo("no passible way !!!(ChkInterS)")

            rospy.sleep(CheckIntersectionCLOCK)

            # かつ、ロボットが動いていない時は回避行動を行う（アドホック）
            robot_vel = self._lib["nav"].getRobotVel()
            if robot_vel < 0.001:
                rospy.loginfo("escaping now ...(ChkInterS)")

                # ロボットを180deg回転させる
                self._lib["nav"].sendRotationTwist(position_meter[2] - math.pi, position_meter[2])
                # とにかく前進する
                self._lib["nav"].sendRunTwist(
                    position_meter[2],
                    position_meter[2],
                    rad_speed = 0.0
                )

            return "false"

        # 開眼時
        elif eyeIO == [0,0,0,1]:
            rospy.loginfo("eyes open")
            # ロボットを停止させる
            self._lib["nav"].sendRunTwist(
                inf_rad_sorted[0],
                position_meter[2],
                lin_speed = 0,
                rad_speed = 0
            )
            #地図の保存とGUIの変更
            self._lib["record"].saveMap()
            point_pixel = self._lib["map"].getRobotPointPixel()
            self._lib["if"].changeGUI(point_pixel)
            rospy.sleep(0.1)
            inf_rad_sorted = self._lib["scan"].getLaserInfRad()
            position_meter = self._lib["map"].getRobotPointMeter()
            rospy.set_param(rospy.get_name() + "/inf_rad_sorted", inf_rad_sorted)

            return "false"

        # 進行可能な空間がある場合(交差点ではない場合)
        elif eyeIO == [0,0,1,0]:
            rospy.loginfo("eyes closed")

            return "true"
            
        else:
            return "false"




#==================================================
## @class GoAhead
## @berif 
#==================================================
class GoAhead(
    smach.State
):
    #==================================================
    ## @fn __init__
    ## @brief コンストラクタ
    ## @param lib ライブラリ群
    ## @return
    #==================================================
    def __init__(
        self,
        lib = {}
    ):
        #==================================================
        # イニシャライズ
        #==================================================
        smach.State.__init__(
            self,
            outcomes = ["next", "loop", "end", "reset", "except"]
        )

        self._lib = lib

        #==================================================
        # ROSインタフェース
        #==================================================

        return




    #==================================================
    ## @fn __del__
    ## @brief デストラクタ
    ## @param
    ## @return
    #==================================================
    def __del__(self):
        #==================================================
        # ファイナライズ
        #==================================================

        return



    #==================================================
    ## @fn execute
    ## @brief 実行関数
    ## @param 
    ## @return
    #==================================================
    def execute(
        self,
        userdata
    ):
        inf_rad_sorted = rospy.get_param(rospy.get_name() + "/inf_rad_sorted", [])
        
        #ゴールに到着した場合、停止して終了
        goal_xy = [17.5,0]
        position_meter = self._lib["map"].getRobotPointMeter()
        if position_meter[0] < goal_xy[0]+0.7 and position_meter[0] > goal_xy[0]-0.7:
            if position_meter[1] < goal_xy[1]+0.7 and position_meter[1] > goal_xy[1]-0.7:
                print("goal reached!")
                self._lib["if"].changeGUI("goal")
                self._lib["nav"].sendRunTwist(
                    inf_rad_sorted[0],
                    position_meter[2],
                    lin_speed = 0,
                    rad_speed = 0
                )
                return "end"

        # 手前に壁がある場合、
        if self._lib["scan"].getAheadDistance() < 1.0:
            # ロボットを停止させる
            self._lib["nav"].sendRunTwist(
                inf_rad_sorted[0],
                position_meter[2],
                lin_speed = 0,
                rad_speed = 0
            )

            rospy.sleep(1.0)

            # 現在のロボットの前進方向から一番近いinf_radを発見する
            nearest_inf_rad_to_present_rad = inf_rad_sorted[0]
            for i in range(len(inf_rad_sorted)):
                if math.cos(inf_rad_sorted[i] - position_meter[2]) >= math.cos(nearest_inf_rad_to_present_rad - position_meter[2]):
                    nearest_inf_rad_to_present_rad = inf_rad_sorted[i]
            # ロボットを回転させる（位置調整のため）
            self._lib["nav"].sendRotationTwist(nearest_inf_rad_to_present_rad, position_meter[2])
            rospy.sleep(1.0)

        else:
            
            # 現在のロボットの前進方向から一番近いinf_radを発見する
            nearest_inf_rad_to_present_rad = inf_rad_sorted[0]
            for i in range(len(inf_rad_sorted)):
                if math.cos(inf_rad_sorted[i] - position_meter[2]) >= math.cos(nearest_inf_rad_to_present_rad - position_meter[2]):
                    nearest_inf_rad_to_present_rad = inf_rad_sorted[i]
            # nearest_inf_rad_to_present_rad方向に前進する
            self._lib["nav"].sendRunTwist(
                nearest_inf_rad_to_present_rad,
                position_meter[2]
            )

            rospy.sleep(CheckIntersectionCLOCK)

            # ロボットが動いていない場合、トライアルをリセットする
            robot_vel = self._lib["nav"].getRobotVel()
            if robot_vel < 0.001:
                """rospy.loginfo("escaping now ...(GoAhead)")
                self._lib["record"].recordGoal(False)
                self._lib["record"].recordStatus("stack")
                self._lib["record"].recordTime(state="lap")
                self._lib["nav"].resetTrial()"""
                self._lib["nav"].sendRunTwist(
                0,
                0,
                lin_speed = -0.3,
                rad_speed = 0.0,
                target_time = 1.0)
                return "loop"
                

        return "next"




#==================================================
## @class End
## @berif 終了ステート
#==================================================
class End(
    smach.State
):
    #==================================================
    ## @fn __init__
    ## @brief コンストラクタ
    ## @param lib ライブラリ群
    ## @return
    #==================================================
    def __init__(
        self,
        lib = {}
    ):
        #==================================================
        # イニシャライズ
        #==================================================
        smach.State.__init__(
            self,
            outcomes = ["end"]
        )

        self._lib = lib
        return




    #==================================================
    ## @fn __del__
    ## @brief デストラクタ
    ## @param
    ## @return
    #==================================================
    def __del__(self):
        #==================================================
        # ファイナライズ
        #==================================================

        return




    #==================================================
    ## @fn execute
    ## @brief 実行関数
    ## @param userdata
    ## @return
    #==================================================
    def execute(
        self,
        userdata
    ):

        return "end"




#==================================================
## @class Except
## @brief 例外ステート
#==================================================
class Except(
    smach.State
):
    #==================================================
    ## @fn __init__
    ## @brief コンストラクタ
    ## @param lib ライブラリ群
    ## @return
    #==================================================
    def __init__(
        self,
        lib = {}
    ):
        #==================================================
        # イニシャライズ
        #==================================================
        smach.State.__init__(
            self,
            outcomes = ["except"]
        )

        self._lib = lib

        return




    #==================================================
    ## @fn __del__
    ## @brief デストラクタ
    ## @param
    ## @return
    #==================================================
    def __del__(self):
        #==================================================
        # ファイナライズ
        #==================================================

        return




    #==================================================
    ## @fn execute
    ## @brief 実行関数
    ## @param userdata
    ## @return
    #==================================================
    def execute(
        self,
        userdata
    ):

        return "except"
