#!/usr/bin/env python3
import pygame.mixer
import time

pygame.mixer.init() #初期化
pygame.mixer.music.load("/home/user/rosOC/src/robot_pkg/script/import/chime.mp3") #読み込み
pygame.mixer.music.play(1) #再生
time.sleep(3)
pygame.mixer.music.stop() #終了

