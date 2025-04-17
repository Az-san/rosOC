#!/usr/bin/env python3
# -*- coding: utf-8 -*-




#==================================================
## @file libnav
## @original_author Kentaro NAKAMURA
## @author Takumi FUJI
## @brief ライブラリクラス
#==================================================




#==================================================
# import
#==================================================
import sys
import roslib

sys.path.append(roslib.packages.get_pkg_dir("robot_pkg") + "/script/import")
from common_import import *

#==================================================
# グローバル
#==================================================




#==================================================
## @class GUI
## @brief GUIのアニメーションを操作する
#==================================================
class GUI(QtWidgets.QWidget):
    #==================================================
    ## @fn __init__
    ## @brief コンストラクタ
    ## @param 
    ## @return
    #==================================================
    def __init__(
        self,
        *args
    ):
        super(QtWidgets.QWidget, self).__init__()

        #==================================================
        # メンバ変数
        #==================================================
        # FPS
        self.fps = 24

        # imgCallback
        size_img = 1, 1, 3
        self.robot_img = np.zeros(size_img, dtype=np.uint8)

        # guiStateCallback
        self.gui_state = "start"

        # GUIに使用する画像の読み込み
        self.load_images()
        
        #map用パス
        self.data_path = roslib.packages.get_pkg_dir("robot_pkg") + "/io"

        #==================================================
        # ROSインタフェース
        #==================================================
        self.bridge = CvBridge()

        self.sub_img = rospy.Subscriber(
            "/camera/rgb/image_raw", 
            Image, 
            self.imgCallback
        )

        self.sub_gui_state = rospy.Subscriber(
            "/gui_state", 
            String, 
            self.guiStateCallback
        )
        
        #2024/10/18 川原 追記（進行方向表示)
        self.direction = rospy.Subscriber(
            "/direction", 
            Int16, 
            self.directionCallback
        )
         
        self.action = rospy.Subscriber(
            "/action", 
            Int16, 
            self.actCallback
        )
         
        self.fire = rospy.Subscriber(
            "/fire", 
            String, 
            self.fireCallback
        )
        self.fire_position = []
        self.direction = 1
        self.action = 3
        self.fire = ""
         ###

        #==================================================
        # イニシャライズ
        #==================================================
        # PyQT4の初期化
        self.video_frame = QtWidgets.QLabel()
        self.lay = QtWidgets.QVBoxLayout()
        self.lay.setContentsMargins(0,0,0,0)
        self.lay.addWidget(self.video_frame)
        self.setLayout(self.lay)

        # ゲームパットの初期化
        pygame.joystick.init()
        try:
            self.j = pygame.joystick.Joystick(0) # create a joystick instance
            self.j.init() # init instance
            print('Joystickの名称: ' + self.j.get_name())
            print('ボタン数 : ' + str(self.j.get_numbuttons()))
        except pygame.error:
            print('Joystickが見つかりませんでした。')

        # pygameの初期化
        pygame.init()


        return




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


        return


    #==================================================
    ## @fn imgCallback
    ## @brief 
    ## @param
    ## @return
    #==================================================
    def imgCallback(
        self,
        data
    ):
        try:
            tmp_sub_img_data = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #self.robot_view_img = cv2.resize(tmp_sub_img_data, (960, 540))
        self.robot_img = tmp_sub_img_data

        return



    #==================================================
    ## @fn guiStateCallback
    ## @brief 
    ## @param
    ## @return
    #==================================================
    def guiStateCallback(
        self,
        data
    ):
        self.gui_state = data.data

        return
    
    #2024/10/18 川原 追記（進行方向表示)
    #==================================================
    ## @fn guiStateCallback
    ## @brief 
    ## @param
    ## @return
    #==================================================
    def directionCallback(
        self,
        data
    ):
        self.direction = data.data

        return
     
    #==================================================
    ## @fn guiStateCallback
    ## @brief 
    ## @param
    ## @return
    #==================================================
    def actCallback(
        self,
        data
    ):
        self.action = data.data

        return
        
    #==================================================
    ## @fn guiStateCallback
    ## @brief 
    ## @param
    ## @return
    #==================================================
    def fireCallback(
        self,
        data
    ):
        self.fire = data.data
        fire_pixel_str = self.fire.split(",")
        fire_position_pixel = [int(fire_pixel_str[0]),int(fire_pixel_str[1])]
        
        self.fire_position.append(fire_position_pixel)
        print(self.fire_position)

        return
        
     #2024/10/18 川原 追記（進行方向表示)



    #==================================================
    ## @fn 
    ## @brief QT独自スレッドの開始？
    ## @param
    ## @return
    #==================================================
    def start(self):
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.gamepadEvent)
        self.timer.start(int(1000./self.fps))


    #==================================================
    ## @fn 
    ## @brief QTウィンドウ上でのゲームパッドイベント管理（ユーザー定義）
    ## @param
    ## @return
    #==================================================
    def gamepadEvent(self):
        for event in pygame.event.get():
            if event.type == pygame.locals.JOYHATMOTION:
                x, y = self.j.get_hat(0)
                if x == -1:
                    print("Le")
                elif x == 1:
                    print("Ri")
                elif y == 1:
                    print("St")
                elif y == -1:
                    print("Ba")


    #==================================================
    ## @fn
    ## @brief QTウィンドウ上でのキーイベント管理（スーパークラス定義）
    ## @param
    ## @return
    #==================================================
    def keyPressEvent(self, e):
        if e.key() == QtCore.Qt.Key_Escape:
            self.close()
            sys.exit()
        elif e.key() == QtCore.Qt.Key_D:
            print("D")
        elif e.key() == QtCore.Qt.Key_Q:
            print("Q")
        elif e.key() == QtCore.Qt.Key_R:
            print("R")
        elif e.key() == QtCore.Qt.Key_W:
            print("W")
        elif e.key() == QtCore.Qt.Key_M:
            print("M")
        elif e.key() == QtCore.Qt.Key_I:
            print("I")


    #==================================================
    ## @fn 
    ## @brief QTウィンドウ上での描画管理（スーパークラス定義）
    ## @param
    ## @return
    #==================================================
    def paintEvent(self, e):
        frame = self.robot_img

        if self.gui_state == "start":
            # タイトル画面
            img = QtGui.QImage(self.title_img, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888)
        else:
            # ロボットカメラ
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = QtGui.QImage(frame, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888)
        painter = QtGui.QPainter(img)
        # 四角形
        painter.setBrush(QtCore.Qt.yellow)
        if self.gui_state == "wait":
            painter.drawRect(102*2, 50*2, 110, 50)
        elif self.gui_state == "goal":
            painter.drawRect(102*2, 50*2, 130, 50)
        elif self.gui_state == "select":
            painter.drawRect(102*2, 50*2, 160, 50)
        elif self.gui_state == "error":
            painter.drawRect(102*2, 50*2, 145, 50)
        # 文字
        painter.setBrush(QtCore.Qt.lightGray)
        painter.setPen(QtCore.Qt.red)
        painter.setFont(QtGui.QFont(u'メイリオ', 30, QtGui.QFont.Bold, False))
        if self.gui_state == "wait":
            painter.drawText(QtCore.QPoint(105*2, 140), 'WAIT')
        elif self.gui_state == "goal":
            painter.drawText(QtCore.QPoint(105*2, 140), 'GOAL!')
        elif self.gui_state == "select":
            painter.drawText(QtCore.QPoint(105*2, 140), 'SELECT')
        elif self.gui_state == "error":
            painter.drawText(QtCore.QPoint(105*2, 140), 'ERROR')
        
        #2024/10/18 川原 追記（進行方向表示）
        if self.action == 2:#炎
            try:
                #self.map_img = cv2.imread(self.data_path + "/map.pgm")
                fire_img = cv2.imread(self.data_path + "/fire.jpg")
            except:
                print("Fire load error!")
            else:
                fire_img = cv2.resize(fire_img, dsize=None, fx=0.8, fy = 1.5)#810x540→ 0.8は調整
                mask = cv2.inRange(fire_img,(0,0,0),(10,10,10))#黒背景を消す
                fire_img = cv2.cvtColor(fire_img, cv2.COLOR_BGR2RGBA)
                fire_img[:,:,3] = 200#透明度
                fire_img[mask == 255 , 3] = 0
                height, width, dim = fire_img.shape
                bytesPerLine = dim * width
                fimg = QtGui.QImage(fire_img, width, height, bytesPerLine, QtGui.QImage.Format_RGBA8888)
                fire_pos_x = int(frame.shape[1]/2 - width/2)
                fire_pos_y = int(frame.shape[0]/2 - height/2)
                painter.drawImage(fire_pos_x,fire_pos_y,fimg)
                
        painter.setBrush(QtCore.Qt.lightGray)
        painter.setPen(QtCore.Qt.red)
        painter.setFont(QtGui.QFont(u'メイリオ', 72, QtGui.QFont.Bold, False))
        if self.gui_state == "select":
            if self.direction == 0:
                painter.drawText(QtCore.QPoint(480*2, 260*2), '↑')#上
            elif self.direction == 2:
                painter.drawText(QtCore.QPoint(490*2, 270*2), '→')#右
            elif self.direction == 3:
                painter.drawText(QtCore.QPoint(470*2, 270*2), '←')#左
            elif self.action == 1:
                painter.drawText(QtCore.QPoint(400*2, 270*2), '魔法モーション')#魔法
        	 #炎が消える演出
        	 #for i in range(self.fps):	
            elif self.action == 0:
                painter.drawText(QtCore.QPoint(480*2, 280*2), '↓')#引き返し
            
        		
        		
        #アバター
        if self.gui_state != "start":
            try:
                #self.map_img = cv2.imread(self.data_path + "/map.pgm")
            	 ava_img = cv2.imread(self.data_path + "/avatar.jpg")
            except:
                print("Avater load error!")
            else:
                ava_img = cv2.resize(ava_img, dsize=None, fx=1.3, fy = 2.0)
                mask = cv2.inRange(ava_img,(240,240,240),(255,255,255))#白背景を消す
                ava_img = cv2.cvtColor(ava_img, cv2.COLOR_BGR2BGRA)
                ava_img[:,:,3] = 255#透明度
                ava_img[mask == 255 , 3] = 0
                height, width, dim = ava_img.shape
                bytesPerLine = dim * width
                avaimg = QtGui.QImage(ava_img, width, height, bytesPerLine, QtGui.QImage.Format_RGBA8888)
                ava_pos_x = int(frame.shape[1]/2 - width/2)
                ava_pos_y = int(frame.shape[0]*3/4 - height/2)
                painter.drawImage(ava_pos_x,ava_pos_y,avaimg)
        	
        ###
        """20224/110/29川原追記
        # 注視点
        if self.gui_state != "start":
            painter.setPen(QtCore.Qt.cyan)
            painter.drawText(QtCore.QPoint(480*2, 270*2), '+')
        """    
        # ミニマップ
        print(self.gui_state)
        
        
        if self.gui_state != "move" and self.gui_state != "wait" and self.gui_state != "goal" and self.gui_state != "select" and self.gui_state != "error":
            if self.gui_state == "start":
                position_pixel = [0,0]
                angle = 0.0
            else:
                position_pixel_str = self.gui_state.split(",")
                position_pixel = [int(position_pixel_str[0]),int(position_pixel_str[1])]
                angle = float(position_pixel_str[2])
                
            self.load_mapimg(position_pixel, angle)
        if self.gui_state != "start":
            height, width, dim = self.map_img.shape
            bytesPerLine = dim * width
            mapimg = QtGui.QImage(self.map_img, width, height, bytesPerLine, QtGui.QImage.Format_RGBA8888)
            #map_pos_x = int(frame.shape[1]/2 - width/2)
            #map_pos_y = int(frame.shape[0]/4*3 - height/2)
            #2024/10/22 川原 追記
            map_pos_x = int(frame.shape[1]/6*5 + width/2)
            map_pos_y = int(frame.shape[0]/6*1 - height/2)
            ###
            painter.drawImage(map_pos_x,map_pos_y,mapimg)

        painter.end()

        pix = QtGui.QPixmap.fromImage(img)
        self.video_frame.setPixmap(pix)


    #==================================================
    ## @fn 
    ## @brief シャットダウン
    ## @param
    ## @return
    #==================================================
    def shutdown(self):
        self.close() # 閉じる



    #==================================================
    ## @fn 
    ## @brief 描画する画像のロード
    ## @param
    ## @return
    #==================================================
    def load_images(self):
        self.title_img = cv2.imread(roslib.packages.get_pkg_dir("robot_pkg") + "/io" + "/titlex2.jpg")


    #==================================================
    ## @fn 
    ## @brief map画像のロード
    ## @param
    ## @return
    #==================================================
    def load_mapimg(self, pos_pix = [0,0], angle = 0.0):
        try:
            self.map_img = cv2.imread(self.data_path + "/map.pgm")
            #self.map_img = cv2.imread(self.data_path + "/map_complete.pgm")#2024/10/18 川原 追記
        except:
            print("Map load error!")
        else:
            self.map_img = cv2.rotate(self.map_img,cv2.ROTATE_90_COUNTERCLOCKWISE)
            #cv2.circle(self.map_img, (pos_pix[0],pos_pix[1]), 6, (0,255,0), thickness = -1)
            pts = np.array(( (int(pos_pix[0]-7*np.sin(angle)), int(pos_pix[1]-7*np.cos(angle))), (int(pos_pix[0]-7.2*np.sin(angle+2.356)), int(pos_pix[1]-7.2*np.cos(angle+2.356))), (int(pos_pix[0] -2*np.sin(angle+3.141)), int(pos_pix[1]-2*np.cos(angle+3.141))), (int(pos_pix[0]-7.2*np.sin(angle-2.356)), int(pos_pix[1]-7.2*np.cos(angle-2.356))) ))
            cv2.fillPoly(self.map_img, [pts], (255,0,0))
            
            #2024/10/18 川原 追記（ミニマップに炎表示）
            if len(self.fire_position) != 0:
                for i in range(len(self.fire_position)):
                    cv2.circle(self.map_img, (self.fire_position[i][0], self.fire_position[i][1]), 5,(253,126,0), -1)#画像、中心、半径、色
                    #cv2.circle(self.map_img, (pos_pix[0], pos_pix[1]), 5, (253,126,0), -1)
                    print("fire")
                    
            
            ###
            
            
            self.map_img = cv2.resize(self.map_img, dsize=None, fx=0.8, fy = 0.8)
            mask = cv2.inRange(self.map_img,(204,204,204),(206,206,206))
            self.map_img = cv2.cvtColor(self.map_img, cv2.COLOR_BGR2BGRA)
            self.map_img[:,:,3] = 128
            self.map_img[mask == 255 , 3] = 0
            
            
        

    #==================================================
    ## @fn クラスメイン関数
    ## @brief
    ## @param
    ## @return
    #==================================================
    def main(
        self
    ):
        # 描画開始
        self.setWindowFlags(QtCore.Qt.Tool)
        
        self.start()
        self.setWindowTitle("Experiment GUI")
        self.show()
        print("Graphic Que")
        return



#==================================================
# メイン
#==================================================
if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    app = QtWidgets.QApplication(sys.argv)
    gui = GUI(0)
    gui.main()
    rospy.on_shutdown(gui.shutdown) 
    app.exec_()
    rospy.spin()
