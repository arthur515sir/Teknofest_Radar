# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'tekno.ui'
#
# Created by: PyQt5 UI code generator 5.14.2
#
# WARNING! All changes made in this file will be lost!
#coders: Güven Nazlıcan
#        Hayrullah Uğur Güvenen





from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import random
import requests
from pymavlink import mavutil # Needed for command message definitions
from datetime import datetime
import numpy as np
from itertools import cycle

from PyQt5 import QtCore, QtGui, QtWidgets
import random, sys
import threading
from PyQt5.QtGui import QPixmap,QTransform
import sys
import numpy as np
import math

import requests
import json
import pandas as pd


###helper 
maxlat=4023388
minlat=4022927
maxlong=2900891
minlong=2900060

###helper 
def latlong_to_xy(lat,long):
    y=((lat-minlat)/(maxlat-minlat))
    x=((long-minlong)/(maxlong-minlong))
    x=x*1000
    y=y*500
    return x,y
    
def center_of_area():
    
    mid_lat=(maxlat+minlat)/2
    mid_long=(maxlong+minlong)/2
    return mid_lat,mid_long

def fly_to_point(target_location, vehicle):
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != VehicleMode("GUIDED"):
        print("wait for GUIDED mode entry")

   
    # vehicle.send_mavlink(
    #     Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
    #             20, 10, 0, target_location.lat, target_location.lon, target_location.alt))

#Vehicle Connection
connection_string_v1 = "udp:127.0.0.1:14550"
print('Connecting to vehicle on: %s' % connection_string_v1)
vehicle = connect(connection_string_v1, wait_ready=False, baud=4800)
vehicle.wait_ready(True, timeout=400)
print('Connected to vehicle')

# API Connection
post_add = "http://192.168.20.10:64559"
s = requests.Session()
login = {"kadi" : "esoguhorus","sifre" :"8coqje215k"}
res_ = s.post(post_add+"/api/giris", json=login)
print(res_.status_code)
takim_numarasi = res_.json()

data = {'GPSSaati':{'saat':0,
                    'dakika':0,
                    'saniye':0,
                    'milisaniye':0},
        'takim_numarasi':takim_numarasi,
        'IHA_enlem':0.0,
        'IHA_boylam':0.0,
        'IHA_irtifa':0.0,
        'IHA_dikilme':0.0,
        'IHA_yonelme':0.0,
        'IHA_yatis':0.0,
        'IHA_hiz':0.0,
        'IHA_batarya':0.0,
        'IHA_otonom':0,
        'IHA_kilitlenme':0,
        'Hedef_merkez_X':0,
        'Hedef_merkez_Y':0,
        'Hedef_genislik':0,
        'Hedef_yukseklik':0,
        }


def send_plane_info(post_add,s):
    global data
    res = s.post(post_add+'/api/telemetri_gonder', json=data)
    print("POST:",res.status_code)
    if res.status_code == 200:
        recieved_data = res.json()
    else:
        recieved_data = {}
    return recieved_data, res.status_code

##### helper function
gps_time = None
@vehicle.on_message('SYSTEM_TIME')
def listener(self, name, message):
#    unix_time = message.time_unix_usec
    #print(message.time_unix_usec)
    global gps_time
    unix_time = message.time_unix_usec
    unix_time /= 1000000
    gps_time = datetime.fromtimestamp(unix_time)
    data["GPSSaati"]["saat"] = gps_time.hour-3
    data["GPSSaati"]["dakika"] = gps_time.minute
    data["GPSSaati"]["saniye"] = gps_time.second
    data["GPSSaati"]["milisaniye"] = int(gps_time.microsecond/1000)

@vehicle.on_attribute('location')
def location_listener(self, name, msg):
    data["IHA_enlem"] = msg.global_relative_frame.lat
    data["IHA_boylam"] = msg.global_relative_frame.lon
    data["IHA_irtifa"] = msg.global_relative_frame.alt
    
@vehicle.on_attribute('attitude')
def attitude_listener(self, name, msg):
    data["IHA_dikilme"] = msg.pitch * 180 / math.pi#pitch
    data["IHA_yonelme"] = msg.yaw * 180 / math.pi #yaw
    data["IHA_yatis"] = msg.roll * 180 / math.pi #roll

@vehicle.on_attribute('groundspeed')
def groundspeed_listener(self, name, msg):
    data["IHA_hiz"] = msg

@vehicle.on_attribute('battery')
def battery_listener(self, name, msg):
    data["IHA_batarya"] = int((msg.voltage - 12.0) * (100 - 10) / (16.8 - 12.0) + 10)

# @vehicle.on_message('MAV_MODE_FLAG')
# def mode_listener(self, name, message):
#     print(message)

@vehicle.on_attribute('mode')
def mode_listener(self, name, msg):
    if msg == "GUIDED" or msg == "RTL" or msg == "LAND" or msg == "AUTO":
        data["IHA_otonom"] = 1
    else:
        data["IHA_otonom"] = 0


prev_tracking_count = 0
flag = False
Target_first = None

while(gps_time ==  None):
    print("gps_time waiting for initilization")
    continue


#print(gps_time)





##GUI section
class Ui_TEKNOFEST21_YKI(object):
    def setupUi(self, TEKNOFEST21_YKI):
        self.NofPlane = 20 #Burayı uçan takımlar sayısına göre değiştir
        self.Time_interval = 1000
    
        TEKNOFEST21_YKI.setObjectName("TEKNOFEST21_YKI")
        TEKNOFEST21_YKI.resize(1364, 848)
        TEKNOFEST21_YKI.setStyleSheet("")
        self.widget_1 = QtWidgets.QWidget(TEKNOFEST21_YKI)
        self.widget_1.setObjectName("widget_1")
        self.widget_2 = QtWidgets.QWidget(self.widget_1)
        self.widget_2.setGeometry(QtCore.QRect(10, 690, 1341, 141))
        self.widget_2.setAutoFillBackground(False)
        self.widget_2.setObjectName("widget_2")
        self.stopFollowBtn = QtWidgets.QPushButton(self.widget_2)
        self.stopFollowBtn.setGeometry(QtCore.QRect(400, 20, 93, 28))
        self.stopFollowBtn.setStyleSheet("background-color:rgba(100,250,250,50);\n"
"border-style:outset;\n"
"border-width:2px;\n"
"border-radius:5px;\n"
"border-color:rgba(0,0,0,180);\n"
"font:bold;\n"
"")
        self.stopFollowBtn.setObjectName("stopFollowBtn")
        self.followBtn = QtWidgets.QPushButton(self.widget_2)
        self.followBtn.setGeometry(QtCore.QRect(60, 20, 181, 28))
        self.followBtn.setStyleSheet("background-color:rgba(100,250,250,50);\n"
"border-style:outset;\n"
"border-width:2px;\n"
"border-radius:5px;\n"
"border-color:rgba(0,0,0,180);\n"
"font:bold;\n"
"")
        self.followBtn.setObjectName("followBtn")
        self.status = QtWidgets.QLineEdit(self.widget_2)
        self.status.setGeometry(QtCore.QRect(110, 70, 71, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        font.setStyleStrategy(QtGui.QFont.PreferDefault)
        self.status.setFont(font)
        self.status.setStyleSheet("background-color:rgba(100,250,250,50);\n"
"border-style:outset;\n"
"border-width:2px;\n"
"border-radius:5px;\n"
"border-color:rgba(0,0,0,180);\n"
"font:bold;\n"
"")
        self.status.setCursorPosition(6)
        self.status.setObjectName("status")
        self.statusLine = QtWidgets.QLineEdit(self.widget_2)
        self.statusLine.setGeometry(QtCore.QRect(320, 70, 251, 22))
        font = QtGui.QFont()
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.statusLine.setFont(font)
        self.statusLine.setStyleSheet("background-color:rgba(0,0,0,10);\n"
"border-style:initial;\n"
"border-width:1px;\n"
"border-radius:5px;\n"
"border-color:rgba(0,0,0,180);\n"
"font:bold;\n"
"")
        self.statusLine.setText("")
        self.statusLine.setCursorPosition(0)
        self.statusLine.setObjectName("statusLine")
        self.manevra = QtWidgets.QPushButton(self.widget_2)
        self.manevra.setGeometry(QtCore.QRect(760, 20, 101, 28))
        self.manevra.setStyleSheet("background-color:rgba(100,250,250,50);\n"
"border-style:outset;\n"
"border-width:2px;\n"
"border-radius:5px;\n"
"border-color:rgba(0,0,0,180);\n"
"font:bold;\n"
"")
        self.manevra.setObjectName("manevra")
        self.manevraLine = QtWidgets.QLineEdit(self.widget_2)
        self.manevraLine.setGeometry(QtCore.QRect(680, 70, 251, 22))
        font = QtGui.QFont()
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.manevraLine.setFont(font)
        self.manevraLine.setStyleSheet("background-color:rgba(0,0,0,10);\n"
"border-style:initial;\n"
"border-width:1px;\n"
"border-radius:5px;\n"
"border-color:rgba(0,0,0,180);\n"
"font:bold;\n"
"")
        self.manevraLine.setText("")
        self.manevraLine.setCursorPosition(0)
        self.manevraLine.setObjectName("manevraLine")
        self.velocity = QtWidgets.QPushButton(self.widget_2)
        self.velocity.setGeometry(QtCore.QRect(1030, 20, 101, 28))
        self.velocity.setStyleSheet("background-color:rgba(100,250,250,50);\n"
"border-style:outset;\n"
"border-width:2px;\n"
"border-radius:5px;\n"
"border-color:rgba(0,0,0,180);\n"
"font:bold;\n"
"")
        self.velocity.setObjectName("velocity")
        self.altitude = QtWidgets.QPushButton(self.widget_2)
        self.altitude.setGeometry(QtCore.QRect(1030, 60, 101, 28))
        self.altitude.setStyleSheet("background-color:rgba(100,250,250,50);\n"
"border-style:outset;\n"
"border-width:2px;\n"
"border-radius:5px;\n"
"border-color:rgba(0,0,0,180);\n"
"font:bold;\n"
"")
        self.altitude.setObjectName("altitude")
        self.flightTime = QtWidgets.QPushButton(self.widget_2)
        self.flightTime.setGeometry(QtCore.QRect(1030, 100, 101, 28))
        self.flightTime.setStyleSheet("background-color:rgba(100,250,250,50);\n"
"border-style:outset;\n"
"border-width:2px;\n"
"border-radius:5px;\n"
"border-color:rgba(0,0,0,180);\n"
"font:bold;\n"
"")
        self.flightTime.setObjectName("flightTime")
        self.velocityLine = QtWidgets.QLineEdit(self.widget_2)
        self.velocityLine.setGeometry(QtCore.QRect(1170, 20, 151, 22))
        font = QtGui.QFont()
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.velocityLine.setFont(font)
        self.velocityLine.setStyleSheet("background-color:rgba(0,0,0,10);\n"
"border-style:initial;\n"
"border-width:1px;\n"
"border-radius:5px;\n"
"border-color:rgba(0,0,0,180);\n"
"font:bold;\n"
"")
        self.velocityLine.setText("")
        self.velocityLine.setCursorPosition(0)
        self.velocityLine.setObjectName("velocityLine")
        self.altitudeLine = QtWidgets.QLineEdit(self.widget_2)
        self.altitudeLine.setGeometry(QtCore.QRect(1170, 60, 151, 22))
        font = QtGui.QFont()
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.altitudeLine.setFont(font)
        self.altitudeLine.setStyleSheet("background-color:rgba(0,0,0,10);\n"
"border-style:initial;\n"
"border-width:1px;\n"
"border-radius:5px;\n"
"border-color:rgba(0,0,0,180);\n"
"font:bold;\n"
"")
        self.altitudeLine.setText("")
        self.altitudeLine.setCursorPosition(0)
        self.altitudeLine.setObjectName("altitudeLine")
        self.timeLine = QtWidgets.QLineEdit(self.widget_2)
        self.timeLine.setGeometry(QtCore.QRect(1170, 100, 151, 22))
        font = QtGui.QFont()
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.timeLine.setFont(font)
        self.timeLine.setStyleSheet("background-color:rgba(0,0,0,10);\n"
"border-style:initial;\n"
"border-width:1px;\n"
"border-radius:5px;\n"
"border-color:rgba(0,0,0,180);\n"
"font:bold;\n"
"")
        self.timeLine.setText("")
        self.timeLine.setCursorPosition(0)
        self.timeLine.setObjectName("timeLine")
        self.velocityLine.raise_()
        self.timeLine.raise_()
        self.statusLine.raise_()
        self.status.raise_()
        self.manevraLine.raise_()
        self.altitudeLine.raise_()
        self.altitude.raise_()
        self.flightTime.raise_()
        self.followBtn.raise_()
        self.manevra.raise_()
        self.stopFollowBtn.raise_()
        self.velocity.raise_()
        self.textEdit_2 = QtWidgets.QTextEdit(self.widget_1)
        self.textEdit_2.setGeometry(QtCore.QRect(577, 10, 210, 65))
        font = QtGui.QFont()
        font.setFamily("Bauhaus 93")
        font.setPointSize(16)
        self.textEdit_2.setFont(font)
        self.textEdit_2.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.BlankCursor))
        self.textEdit_2.setMouseTracking(True)
        self.textEdit_2.setContextMenuPolicy(QtCore.Qt.DefaultContextMenu)
        self.textEdit_2.setAutoFillBackground(False)
        self.textEdit_2.setStyleSheet("background-color:rgba(150,150,150,200);")
        self.textEdit_2.setFrameShape(QtWidgets.QFrame.Panel)
        self.textEdit_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.textEdit_2.setLineWidth(7)
        self.textEdit_2.setMidLineWidth(0)
        self.textEdit_2.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.textEdit_2.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.textEdit_2.setSizeAdjustPolicy(QtWidgets.QAbstractScrollArea.AdjustIgnored)
        self.textEdit_2.setTabChangesFocus(False)
        self.textEdit_2.setObjectName("textEdit_2")
        self.widget = QtWidgets.QWidget(self.widget_1)
        self.widget.setGeometry(QtCore.QRect(10, 80, 1341, 591))
        self.widget.setObjectName("widget")
        self.label = QtWidgets.QLabel(self.widget_1)
        self.label.setGeometry(QtCore.QRect(-4, -8, 1371, 861))
        self.label.setStyleSheet("transparent:200")
        self.label.setText("")
        self.label.setPixmap(QtGui.QPixmap("entry4.png"))
        self.label.setScaledContents(True)
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.widget_1)
        self.label_2.setGeometry(QtCore.QRect(800, 0, 81, 71))
        self.label_2.setText("")
        self.label_2.setPixmap(QtGui.QPixmap("Esogu_logo.png"))
        self.label_2.setScaledContents(True)
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.widget_1)
        self.label_3.setGeometry(QtCore.QRect(480, 0, 81, 71))
        self.label_3.setText("")
        self.label_3.setPixmap(QtGui.QPixmap("Esogu_logo.png"))
        self.label_3.setScaledContents(True)
        self.label_3.setObjectName("label_3")
        self.widget_2.raise_()
        self.widget.raise_()
        self.textEdit_2.raise_()
        self.label_3.raise_()
        self.label_2.raise_()
        self.label.raise_()
        TEKNOFEST21_YKI.setCentralWidget(self.widget_1)
        
        
        self.label_info=[]
        self.radio = []
        self.foto = []
        self.fotolabel=[]
        self.old_xy = [[0]*2]*self.NofPlane
        self.plane=0
        self.plane_lat=0
        self.plane_long=0
        self.plane_alt=0
      
        # self.horus_png=QPixmap('unnamed.png')
        # self.horus_label = QtWidgets.QLabel(self.widget)
        # self.horus_label.resize(50,50)
        # self.horus_label.move(1000-19,1000-18)
        # self.horus_label.setScaledContents(True)
        # self.horus_label.lower()
        for i in range(self.NofPlane):
            
            
            
           
            self.radio.append(i)
            self.radio[i] = QtWidgets.QRadioButton(self.widget)
            self.radio[i].setGeometry(QtCore.QRect(1000, 1000, 9, 9))
            self.radio[i].setObjectName("radioButton")
            self.radio[i].setText('{0}'.format(i))
            self.radio[i].clicked.connect(lambda a=i: self.etki(a))
            self.label_info.append(i)
            self.label_info[i]=QtWidgets.QLabel(self.widget)
            self.radio[i].setGeometry(QtCore.QRect(1000, 1000, 9, 9))
            self.label_info[i].setText('grup {0}'.format(i))
            self.label_info[i].move(1000,1000)
            self.foto.append(i)
            self.fotolabel.append(i)
            self.foto[i]=QPixmap('969140.png')
            
            
            self.fotolabel[i] = QtWidgets.QLabel(self.widget)
            
            self.fotolabel[i].setPixmap(self.foto[i])
            self.fotolabel[i].resize(50,50)
            self.fotolabel[i].move(1000-19,1000-18)
            self.fotolabel[i].setScaledContents(True)
            self.fotolabel[i].lower()
            self.label.lower()
            
        self.stopFollowBtn.clicked.connect(self.takibi_birak)
        self.followBtn.clicked.connect(self.takibe_basla)
        self.manevra.clicked.connect(self.maneuver)

        self.retranslateUi(TEKNOFEST21_YKI)
        QtCore.QMetaObject.connectSlotsByName(TEKNOFEST21_YKI)
        self.time = QtCore.QTimer()
        self.time.timeout.connect(self.update_new)
        self.time.setInterval(900)
        self.time.start()
        TEKNOFEST21_YKI.show()
        
    def retranslateUi(self, TEKNOFEST21_YKI):
        _translate = QtCore.QCoreApplication.translate
        TEKNOFEST21_YKI.setWindowTitle(_translate("TEKNOFEST21_YKI", "ESOGU_HORUS"))
        self.stopFollowBtn.setText(_translate("TEKNOFEST21_YKI", "Takibi Bırak"))
        self.followBtn.setText(_translate("TEKNOFEST21_YKI", "Seçili IHA\'ya Git, Takip Et"))
        self.status.setText(_translate("TEKNOFEST21_YKI", "Durum:"))
        self.manevra.setText(_translate("TEKNOFEST21_YKI", "Kaçış Manevrası"))
        self.velocity.setText(_translate("TEKNOFEST21_YKI", "Hız:"))
        self.altitude.setText(_translate("TEKNOFEST21_YKI", "Yükseklik:"))
        self.flightTime.setText(_translate("TEKNOFEST21_YKI", "Sistem Saati:"))
        self.textEdit_2.setHtml(_translate("TEKNOFEST21_YKI", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Bauhaus 93\'; font-size:16pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:14pt;\">Eskisehir Osmangazi Üniversitesi</span></p></body></html>"))



    def update_new(self):
        j=0
        #### buraya comm while döngüsü gelicek ###
        received_data, res_3_status = send_plane_info(post_add, s)  
        print(time.time())
        self.velocityLine.setText(str(round(vehicle.groundspeed,2)) + "  m/s")
        self.altitudeLine.setText(str(vehicle.location.global_relative_frame.alt) + "  m")
        self.timeLine.setText(str())
        self.recdat=received_data
        print(data)
        sunucu_saati=0
        sunucu_dakika=0
        sunucu_saniye=0
        try:
            konum_bilgileri = received_data["konumBilgileri"]
            sunucu_saati = received_data['sistemSaati']['saat']
            sunucu_milisaniye = received_data['sistemSaati']['milisaniye']
            sunucu_gun = received_data['sistemSaati']['gun']
            sunucu_dakika = received_data['sistemSaati']['dakika']
            sunucu_saniye = received_data['sistemSaati']['saniye']
        except:
            pass
    
      
        start_tracking = True

        #closest_team = calculate_distances(vehicle.location.global_relative_frame, received_data, takim_numarasi)
    
        self.timeLine.setText(str(sunucu_saati)+ "." + str(sunucu_dakika)+"."+ str(sunucu_saniye) + "   UTC")
        #print(received_data)
        self.manevraLine.setText("")
         
        # self.horus_png=QPixmap('unnamed.png')
        # self.horus_label = setPixmap(self.horus_png.transformed(trans_angle_horus))
        # self.horus_label.move(x1-19,y1-18)
        
        
        # self.horus_label.resize(50,50)
        # self.horus_label.move(1000-19,1000-18)
        # self.horus_label.setScaledContents(True)
        # self.horus_label.lower()
        
        
        
        for i in (konum_bilgileri):
            lat=abs(i["iha_enlem"])*100000
            long=abs(i["iha_boylam"])*100000
            x1,y1=latlong_to_xy(lat,long)
            x1=int(x1)
            y1=int(y1)
            
            self.radio[j].setText(str(i["takim_numarasi"]))
            self.label_info[j].setText(str(i["takim_numarasi"]))
            self.radio[j].setToolTip(str(i["iha_enlem"])+" "+str(i["iha_boylam"]))  
            self.radio[j].move(x1,y1)
            self.radio[j].setText(str(i["takim_numarasi"]))
            angle=math.degrees(math.atan2( (self.old_xy[j][0]-x1), (self.old_xy[j][1]-y1)))

            trans_angle=QTransform()
            trans_angle.rotate(-angle)

            self.foto[j]=QPixmap('969140.png')
            self.fotolabel[j].setPixmap(self.foto[j].transformed(trans_angle))
            self.fotolabel[j].move(x1-19,y1-18)
            self.fotolabel[j].setScaledContents(True)
            self.label_info[j].move(x1+15,y1+15)
            self.label_info[j].lower()
            
            self.old_xy[j]=[x1,y1]
            j=j+1
    
            
    

    def etki(self,a):
        a=1
        # k=0
        # print("*********************1111111111")

       
        # for i in (self.recdat["konumBilgileri"]):
        #     print(i)
        #     if self.radio[k].isChecked():
        #         self.radio_current_index=k
        #         #self.set_plane(i["takim_numarasi"],i["iha_enlem"],i["iha_boylam"],i["iha_irtifa"])
        #         self.plane=i["takim_numarasi"]
        #         self.plane_lat=i["iha_enlem"]
        #         self.plane_long=i["iha_boylam"]
        #         self.plane_alt=i["iha_irtifa"]
        #     k=k+1    
            
   
             
    
 
    def takibi_birak(self):
        a=0  

        # if self.plane<0:
        #     print("uçak seçilmedi")
        #     self.statusLine.setText("uçak seçilmedi")
        # else: 
        #     k=0
        #     for i in range(self.NofPlane):
        #         self.radio[i]
                
        #     self.radio[k]
        #     print("stop following team "+ str(self.plane))
        #     self.statusLine.setText("stop following team " + str(self.plane))
        #     vehicle.mode = VehicleMode("AUTO")
            
    def takibe_basla(self):
        a=1

        # #for index in range(5):#gvn olmasa olabilir
        #     k=0
          
        #     for i in (self.recdat["konumBilgileri"]):
        #             print(i)
        #             if self.radio[k].isChecked():
        #                     self.radio_current_index=k
        #                     #self.set_plane(i["takim_numarasi"],i["iha_enlem"],i["iha_boylam"],i["iha_irtifa"])
        #                     self.plane=i["takim_numarasi"]
        #                     self.plane_lat=i["iha_enlem"]
        #                     self.plane_long=i["iha_boylam"]
        #                     self.plane_alt=i["iha_irtifa"]
        #             k=k+1    
            
            
            
        #     Locations = LocationGlobalRelative(self.plane_lat,self.plane_long,self.plane_alt-3)
        #     fly_to_point(Locations, vehicle)
        #     print("start following team "+ str(self.plane))
        #     print(str(self.plane_lat))
        #     print(str(self.plane_long))
            
        #     self.statusLine.setText("start following team " + str(self.plane))
        #     #time.sleep(1)#gvn olmasa olabilir 2
    def maneuver(self):
        a=0

        # if self.plane==0:
        #     print("uçak seçilmedi")
        # else :
        #       print("manevra "+ self.plane)
        # mid_x,mid_y=center_of_area()
        # Locations = LocationGlobalRelative(mid_x,mid_y,100)
        # fly_to_point(Locations, vehicle)
        
        # self.manevraLine.setText("Kaçış algoritması çalışıyor...")
        
if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    TEKNOFEST21_YKI = QtWidgets.QMainWindow()
    ui = Ui_TEKNOFEST21_YKI()
    ui.setupUi(TEKNOFEST21_YKI)
    TEKNOFEST21_YKI.show()
    sys.exit(app.exec_())
