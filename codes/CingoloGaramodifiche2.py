#!/user/bin/env python3
import time
import cv2
import math
import numpy as np
import RPi.GPIO as GPIO
import smbus
import warnings
from picamera2 import Picamera2, Preview
from libcamera import controls, Transform
from threading import Thread
from adafruit_servokit import ServoKit
from hcsr04sensor import sensor
import board
import smbus2



warnings.simplefilter("ignore")

GPIO.setmode(GPIO.BCM)  # pin modalità BOARD pin hardware modalita BCM pin Logici
GPIO.setwarnings(False)

Kp =1 #0.8
boost = 1.6#1.2

VelSalita=45 #55
VelMax=55
VelMin=30
VelDiscesa = 25
VelCro=VelMax

WebLinea=102#113      #  Inclinazione WEBCAM Seguilinea
WebStanza=170           #25 Angolo inclinazione Webcam in Arena tra 75 e 80 posizioni ottimali
WebUscita=70
WebSicura=50

ChiudiThred=False
#SET Ulrasuoni

TrigFrSx=14     
EchoFrSx=15

TrigFrDx=18     
EchoFrDx=23

TrigLato=25     
EchoLato=24


SwitchOnOFF=6 
ArgSx= 27
ArgDx= 22

speed_of_sound=340


GPIO.setup(SwitchOnOFF,GPIO.IN)
GPIO.setup(ArgSx,GPIO.IN)
GPIO.setup(ArgDx,GPIO.IN)

#SET IPCamera

RisX=320
RisY=240
camera = Picamera2()
vidconfig = camera.create_preview_configuration(main={"format": 'XRGB8888', "size": (RisX,RisY)})
vidconfig["transform"] = Transform(hflip=1, vflip=1)
vidconfig["size"] = (RisX,RisY)
camera.configure(vidconfig)
camera.start()
camera.set_controls({"AfMode": controls.AfModeEnum.Continuous})
with camera.controls as controls:
    #controls.ExposureTime = 1000#10000
    #controls.AnalogueGain = 0.9 
    controls.Brightness = -0.1 #-0.15
    controls.ExposureValue = 1.5
    controls.Contrast = 1  
    controls.FrameRate = 25
    controls.AwbEnable = True
    controls.AfMode: "Continuous"
    controls.AeEnable: True
    controls.AwbMode: "auto"
    controls.NoiseReductionMode:"HighQuality"
DatiCam= camera.capture_metadata()
#print(DatiCam)     
print('avvio')
time.sleep(1)

xDim, yDim = (320,240)

OffSetX = 30
OffSetY = 30 

ExtX = xDim-OffSetX
ExtY = yDim-OffSetY

CentroX = OffSetX+(xDim-(2*OffSetX))//2
CentroY = OffSetY+(yDim-(2*OffSetY))//2

Immagine=None
Attesa=False
InArena=False
DistanzaDx=20
DistanzaSx=20
DistanzaLato=20
OutArena=False

bus = smbus.SMBus(1)
time.sleep(1)
i2c = board.I2C()
#mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
#accel = adafruit_lsm303_accel.LSM303_Accel(i2c)
#address = 0x68 # indirizzo del giroscopio MPU6050 
#GiroSco = mpu6050(address)# INIZIALIZZA SISTEMA MAGNRTOMETRO e GIROSCOPIO

   
# INIZIALIZZA SISTEMA MAGNRTOMETRO e GIROSCOPIO
xRotation=0
yRotation=0
zRotation=0

ADDRESS_ACC = 0x19  
ADDRESS_MAG = 0x1E  

# Registri accelerometro
REG_CTRL1_A = 0x20  
REG_OUT_X_L_A = 0x28  

# Registri magnetometro
REG_MAG_CRA = 0x00
REG_MAG_CRB = 0x01
REG_MAG_MR = 0x02
REG_MAG_OUT_X_MSB = 0x03  

# Filtro Complementare
ALPHA = 0.3 
heading_filtered = 0  
pitch_filtered = 0
roll_filtered = 0

# Inizializza la comunicazione I2C
bus2 = smbus2.SMBus(1)

def setup():
    bus2.write_byte_data(ADDRESS_ACC, REG_CTRL1_A, 0x57)
    bus2.write_byte_data(ADDRESS_MAG, REG_MAG_CRA, 0x10)
    bus2.write_byte_data(ADDRESS_MAG, REG_MAG_CRB, 0x20)
    bus2.write_byte_data(ADDRESS_MAG, REG_MAG_MR, 0x00)

def read_accelerometer():
    data = bus2.read_i2c_block_data(ADDRESS_ACC, REG_OUT_X_L_A | 0x80, 6)
    x = (data[1] << 8) | data[0]
    y = (data[3] << 8) | data[2]
    z = (data[5] << 8) | data[4]

    if x > 32767: x -= 65536
    if y > 32767: y -= 65536
    if z > 32767: z -= 65536

    return x, y, z

def read_magnetometer():
    data = bus.read_i2c_block_data(ADDRESS_MAG, REG_MAG_OUT_X_MSB, 6)
    x = (data[0] << 8) | data[1]
    z = (data[2] << 8) | data[3]
    y = (data[4] << 8) | data[5]

    if x > 32767: x -= 65536
    if y > 32767: y -= 65536
    if z > 32767: z -= 65536

    return x, y, z

setup()

def Inclinazioni():
    global zRotation  
    global yRotation
    global xRotation
    global heading_filtered  
    global pitch_filtered
    global roll_filtered    
    acc_x, acc_y, acc_z = read_accelerometer()
    mag_x, mag_y, mag_z = read_magnetometer()

    roll = math.atan2(acc_y, acc_z) * (180 / math.pi)
    pitch = math.atan2(-acc_x, math.sqrt(acc_y**2 + acc_z**2)) * (180 / math.pi)

    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)

    mag_x_comp = mag_x * math.cos(pitch_rad) + mag_z * math.sin(pitch_rad)
    mag_y_comp = mag_x * math.sin(roll_rad) * math.sin(pitch_rad) + mag_y * math.cos(roll_rad) - mag_z * math.sin(roll_rad) * math.cos(pitch_rad)

    heading = math.atan2(mag_y_comp, mag_x_comp) * (180 / math.pi)

    if heading < 0:
        heading += 360

    # **Filtro Complementare**
    #if abs(heading_filtered - heading) > 300:
    #    heading_filtered = 360
    #else:    
    heading_filtered = (ALPHA * heading_filtered + (1 - ALPHA) * heading)
    pitch_filtered = (ALPHA * pitch_filtered + (1 - ALPHA) * pitch)
    roll_filtered = (ALPHA * roll_filtered + (1 - ALPHA) * roll)
    zRotation=int(heading_filtered)
    yRotation=int(pitch_filtered)
    xRotation=int(roll_filtered)
    #print('Misuro inclinazioni' ,xRotation,yRotation,zRotation)
    #return zRotation, yRotation, xRotation
    
#AvviaMisure = Thread(target=Misure)
#AvviaMisure.start()
#time.sleep(1.2)


def CatturaImmagini():
    global ChiudiThred
    #global rawCapture
    global Immagine
    global InArena
    global Attesa

    print("Aperto Cattura Immagini")
    #rawCapture.truncate(0)
    while True:
        if ChiudiThred:
            #rawCapture.truncate(0)
            break
        Immagine = camera.capture_array()

        if yRotation > -9 and yRotation < 9:
            InArena= not GPIO.input(ArgSx) or not GPIO.input(ArgDx)
        else:
            InArena = False
            
            
        # Carico Stato programma: Attesa o Procedi
        Attesa=GPIO.input(SwitchOnOFF)

        #cv2.imshow('Immagine', Immagine)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    break
        #time.sleep(0.1)
    print("chiuso cattura Immagini")    
        
VideoCam = Thread(target=CatturaImmagini)
VideoCam.start()
time.sleep(1.1)
      

UltraFronteDx = sensor.Measurement
UltraFronteSx = sensor.Measurement
UltraLato= sensor.Measurement
def MisDista():
    global ChiudiThred
    global DistanzaDx
    global DistanzaSx
    global DistanzaLato

    while True:    
        try:
            DistanzaDx = int(UltraFronteDx.basic_distance(TrigFrDx, EchoFrDx))
            time.sleep(0.1)
        except:
            print("ErroreDx")
        try:
            DistanzaSx = int(UltraFronteSx.basic_distance(TrigFrSx, EchoFrSx))
            time.sleep(0.1)
        except:
            print("ErroreSx")
        try:    
            DistanzaLato = int(UltraLato.basic_distance(TrigLato, EchoLato))
            time.sleep(0.1)
        except:
            print("ErroreLT")

        if ChiudiThred:
            break
        time.sleep(0.1)

AvviaMisDista = Thread(target=MisDista)
AvviaMisDista.start()
time.sleep(1.3)        
print ("Distanza Avviato")

#SET DRIVE MOTORI SERVO
MSx = 15  # Canale Scheda controllo
MDx = 0  # Canale Scheda controllo


MPalSx = 14
MPalDx = 13

MPzDx = 11
MPzSx = 3



Web = 8    # Canale WebCam

Motori = ServoKit(channels=16) 

#Motori.frequency = 60

def Stop():
    global Motori
    Motori.continuous_servo[MDx].throttle = 0
    Motori.continuous_servo[MSx].throttle = 0

def Dritto(Vel):        #valori positivi va avanti negativi indietro con 0 si ferma
    global Motori 
    if Vel > 100:
        Vel=100
    if Vel < -100:
        Vel=-100
    StrDX=(Vel/100)
    Motori.continuous_servo[MDx].throttle = StrDX 
    Motori.continuous_servo[MSx].throttle = -(StrDX)
        
def Sterzo(Str):
    global Motori
    if Str < -100:
        Motori.continuous_servo[MDx].throttle = VelCro/100
        
        Motori.continuous_servo[MSx].throttle = VelCro/100
               
    elif Str < -90:
        Motori.continuous_servo[MDx].throttle = VelCro/100
        
        Motori.continuous_servo[MSx].throttle = VelCro/150
                
    elif Str < -80:
        Motori.continuous_servo[MDx].throttle = VelCro/100
        
        Motori.continuous_servo[MSx].throttle = VelCro/300
               
    elif Str < -70:
        Motori.continuous_servo[MDx].throttle = VelCro/100
        
        Motori.continuous_servo[MSx].throttle = VelCro/500
               
    elif Str < -60:
        Motori.continuous_servo[MDx].throttle = VelCro/100
        
        Motori.continuous_servo[MSx].throttle = VelCro/750
                
    elif Str < -50:
        Motori.continuous_servo[MDx].throttle = VelCro/100
        
        Motori.continuous_servo[MSx].throttle = -VelCro/1000
                
    elif Str < -40:
        Motori.continuous_servo[MDx].throttle = VelCro/100
        
        Motori.continuous_servo[MSx].throttle = -VelCro/750
                
    elif Str < -30:
        Motori.continuous_servo[MDx].throttle = VelCro/100
        
        Motori.continuous_servo[MSx].throttle = -VelCro/500
               
    elif Str < -20:
        Motori.continuous_servo[MDx].throttle = VelCro/100
        
        Motori.continuous_servo[MSx].throttle = -VelCro/300
                
    elif Str < -10:
        Motori.continuous_servo[MDx].throttle = VelCro/100
        
        Motori.continuous_servo[MSx].throttle = -VelCro/150
          
    elif Str <= 0:
        Motori.continuous_servo[MDx].throttle = VelCro/100
        
        Motori.continuous_servo[MSx].throttle = -VelCro/100
        
    elif Str < 10:
        Motori.continuous_servo[MDx].throttle = VelCro/150
        
        Motori.continuous_servo[MSx].throttle = -VelCro/100
          
    elif Str < 20:
        Motori.continuous_servo[MDx].throttle = VelCro/300
        
        Motori.continuous_servo[MSx].throttle = -VelCro/100
          
    elif Str < 30:
        Motori.continuous_servo[MDx].throttle = VelCro/500
        
        Motori.continuous_servo[MSx].throttle = -VelCro/100
         
    elif Str < 40:
        Motori.continuous_servo[MDx].throttle = VelCro/750
        
        Motori.continuous_servo[MSx].throttle = -VelCro/100
           
    elif Str < 50:
        Motori.continuous_servo[MDx].throttle = VelCro/1000
        
        Motori.continuous_servo[MSx].throttle = -VelCro/100
         
    elif Str < 60:
        Motori.continuous_servo[MDx].throttle = -VelCro/750
        
        Motori.continuous_servo[MSx].throttle = -VelCro/100
        
    elif Str < 70:
        Motori.continuous_servo[MDx].throttle = -VelCro/500
        
        Motori.continuous_servo[MSx].throttle = -VelCro/100
        
    elif Str < 80:
        Motori.continuous_servo[MDx].throttle = -VelCro/300
        
        Motori.continuous_servo[MSx].throttle = -VelCro/100
        
    elif Str < 90:
        Motori.continuous_servo[MDx].throttle = -VelCro/150
        
        Motori.continuous_servo[MSx].throttle = -VelCro/100
         
    else:
        Motori.continuous_servo[MDx].throttle = -VelCro/100
        
        Motori.continuous_servo[MSx].throttle = -VelCro/100
                


    
def AbbassaPala():
    Adx=0
    Asx=180
    Motori.servo[Web].angle = WebSicura
    for i in range(90):
        Adx=Adx+2
        Asx=Asx-2
        Motori.servo[MPalDx].angle = Adx
        Motori.servo[MPalSx].angle = Asx
        time.sleep(0.02) 
    Motori.servo[Web].angle = WebLinea
    
def AlzaPala():
    Adx=180
    Asx=0
    Motori.servo[Web].angle = WebSicura
    for i in range(90):
        Adx=Adx-2
        Asx=Asx+2
        Motori.servo[MPalDx].angle = Adx
        Motori.servo[MPalSx].angle = Asx
        time.sleep(0.02)
    Motori.servo[Web].angle = WebLinea
        
def ApriPinza():
    Adx=134 
    Asx=58
    for i in range(48):
        Adx=Adx-2
        Asx=Asx+2
        Motori.servo[MPzDx].angle = Adx
        Motori.servo[MPzSx].angle = Asx
        time.sleep(0.02)
    
    
    
def ChiudiPinza():
    Adx=38
    Asx=154
    for i in range(48):
        Adx=Adx+2
        Asx=Asx-2
        Motori.servo[MPzDx].angle = Adx
        Motori.servo[MPzSx].angle = Asx
        time.sleep(0.02) 
     
def ScaricaPalla():   #ScaricaPalla
    #La pinza è posizionata in alto al massimo AlzaPala
    Adx=0
    Asx=180
    Motori.servo[Web].angle = WebSicura
    for i in range(55):
        Adx=Adx+2
        Asx=Asx-2
        Motori.servo[MPalDx].angle = Adx
        Motori.servo[MPalSx].angle = Asx
        time.sleep(0.02)
    time.sleep(1)
    ApriPinza()
    time.sleep(0.5)
    Motori.servo[MPalDx].angle = Adx+4
    Motori.servo[MPalSx].angle = Asx-4
    time.sleep(0.05)
    Motori.servo[MPalDx].angle = Adx-4
    Motori.servo[MPalSx].angle = Asx+4
    time.sleep(1)
    for i in range(55):
        Adx=Adx-2
        Asx=Asx+2
        Motori.servo[MPalDx].angle = Adx
        Motori.servo[MPalSx].angle = Asx
        time.sleep(0.02)    

    time.sleep(1)
    Motori.servo[Web].angle = WebStanza

def Girata():
    global Immagine
    Lv=80  #70
    Lt=15
    TipoAzione=""
    Stop()
    time.sleep(0.1)
    #Dritto(-30)
    #time.sleep(0.2) #0.4
    Stop()
    time.sleep(0.1)
    print("Leggo Verde")
    #Immagine = camera.capture_array()
    img=Immagine
    img_ruotata=img
    cv2.imshow('SLinea', img)
    key = cv2.waitKey(1) & 0xFF
    time.sleep(0.1)
    # Carica l'immagine
    if img is None:
        raise ValueError("Immagine non trovata o formato non supportato.")
    
    (h, w) = img.shape[:2]
    # Converte l'immagine nello spazio colore HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Definisci i limiti del colore verde
    lower_green = np.array([35, 35, 35])  # Limiti inferiori (Hue, Saturation, Value)
    upper_green = np.array([85, 255, 255])  # Limiti superiori
    
    # Applica la maschera per isolare il verde
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # Trova i contorni della maschera
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        #raise ValueError("Nessuna zona verde trovata nell'immagine.")
        TipoAzione="None"
    # Seleziona il contorno più grande
    else:
        largest_contour = max(contours, key=cv2.contourArea)
    
        # Calcola il rettangolo orientato (bounding box rotato)
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        centro , _ ,_ = rect
    
        box = np.int0(box)
    
        # Disegna il rettangolo sull'immagine originale (opzionale, per debug)
        img_with_box = cv2.drawContours(img.copy(), [box], 0, (0, 0, 255), 2)
        cv2.imshow('SLinea', img_with_box)
        key = cv2.waitKey(1) & 0xFF
        time.sleep(0.1)
    
    # L'inclinazione rispetto all'asse x è data dall'angolo del rettangolo
        angle=rect[-1] 
        if angle>0:
            angle = -(90-rect[-1]) 
        else:
            angle = -rect[-1]    
        print("Trovato Verde inclinato di: ", angle)
        time.sleep(0.1)
        img_ruotata = img
        '''
        if angle < -34:
            M = cv2.getRotationMatrix2D(centro, angle, 1.0)
            img_ruotata = cv2.warpAffine(img, M, (w, h))
            #RuotaImmagine verso orario pari ad angolo
        elif angle > 34:
            M = cv2.getRotationMatrix2D(centro, angle, 1.0)
            img_ruotata = cv2.warpAffine(img, M, (w, h))
            #RuotaImmagine verso antiorario
        else:
            img_ruotata = img
        '''
        TrovVer1,(cVe1X,cVe1Y),TrovVer2,(cVe2X,cVe2Y) = TrovaColore(img_ruotata,MinRangeVeHSV,MaxRangeVeHSV,(0,255,255),AreaSigCol)
        cv2.imshow('SLinea', img_ruotata)
        key = cv2.waitKey(1) & 0xFF
        time.sleep(0.1)
        if TrovVer1 and TrovVer2 and (cVe1Y > (cVe2Y-50) and cVe1Y < (cVe2Y+50)): # Doppio verde verifica solo se falso
            if (Lv+Lt+cVe1Y+10)>h: #¶Il verde è basso bisogna verificare la parte alta
                #Verifica se esiste il nero sopra
                ImmBkAlto=img_ruotata[cVe1Y-Lv-Lt:cVe1Y-Lv,cVe1X:cVe1X+Lt]
                TrovBkAlto,(cBkCeXSx,cBkCeYSx),_,_ = TrovaNeri(ImmBkAlto,SGreyVerdi,cv2.THRESH_BINARY,(255,0,0),AreaSigBkVe)
                if TrovBkAlto:
                    TipoAzione="Fr"
                else:
                    TipoAzione="FF"
            else:
                #verifica se esiste il nero sotto
                ImmBkBasso=img_ruotata[cVe1Y+Lv:cVe1Y+Lv+Lt,cVe1X:cVe1X+Lt]
                TrovBkBasso,(cBkCeXSx,cBkCeYSx),_,_ = TrovaNeri(ImmBkBasso,SGreyVerdi,cv2.THRESH_BINARY,(255,0,0),AreaSigBkVe)
                if TrovBkBasso:
                    TipoAzione="FF"
                else:
                    TipoAzione="Fr"
        elif TrovVer1 and TrovVer2 and not (cVe1Y > (cVe2Y-50) and cVe1Y < (cVe2Y+50)):
            if cVe2Y > cVe1Y:
                (cVe1X,cVe1Y)=(cVe2X,cVe2Y)
            if (Lv+Lt+cVe1Y+10)>h: #¶Il verde è basso bisogna verificare la parte alta
                #Verifica se esiste il nero sopra
                ImmBkAlto=img_ruotata[cVe1Y-Lv-Lt:cVe1Y-Lv,cVe1X:cVe1X+Lt]
                TrovBkAlto,(cBkCeXSx,cBkCeYSx),_,_ = TrovaNeri(ImmBkAlto,SGreyVerdi,cv2.THRESH_BINARY,(255,0,0),AreaSigBkVe)
                if TrovBkAlto:
                    TipoAzione="CROSS"
                else:
                    TipoAzione="FF"
            else:
                #verifica se esiste il nero sotto
                ImmBkBasso=img_ruotata[cVe1Y+Lv:cVe1Y+Lv+Lt,cVe1X:cVe1X+Lt]
                TrovBkBasso,(cBkCeXSx,cBkCeYSx),_,_ = TrovaNeri(ImmBkBasso,SGreyVerdi,cv2.THRESH_BINARY,(255,0,0),AreaSigBkVe)
                if TrovBkBasso:
                    TipoAzione="FF"
                else:
                    TipoAzione="CROSS"
        elif TrovVer1:            
            if (Lv+Lt+cVe1Y+10)>h: #¶Il verde è basso bisogna verificare la parte alta
                #Verifica se esiste il nero sopra
                ImmBkAlto=img_ruotata[cVe1Y-Lv-Lt:cVe1Y-Lv,cVe1X:cVe1X+Lt]
                TrovBkAlto,(cBkCeXSx,cBkCeYSx),_,_ = TrovaNeri(ImmBkAlto,SGreyVerdi,cv2.THRESH_BINARY,(255,0,0),AreaSigBkVe)
                if TrovBkAlto:
                    TipoAzione="CROSS"
                else:
                    TipoAzione="FF"
            else:
                #verifica se esiste il nero sotto
                ImmBkBasso=img_ruotata[cVe1Y+Lv:cVe1Y+Lv+Lt,cVe1X:cVe1X+Lt]
                TrovBkBasso,(cBkCeXSx,cBkCeYSx),_,_ = TrovaNeri(ImmBkBasso,SGreyVerdi,cv2.THRESH_BINARY,(255,0,0),AreaSigBkVe)
                if TrovBkBasso:
                    TipoAzione="FF"
                else:
                    TipoAzione="CROSS"
        else:
                TipoAzione="None"
        if TipoAzione=="CROSS":       
            if (Lv+Lt+cVe1X+10)>w: #il verde è troppo a destra bisogna verificare a sinistra
                ImmBkSx=img_ruotata[cVe1Y:cVe1Y+Lt,cVe1X-Lv-Lt:cVe1X-Lv]
                TrovBkSx,(cBkCeXSx,cBkCeYSx),_,_ = TrovaNeri(ImmBkSx,SGreyVerdi,cv2.THRESH_BINARY,(255,0,0),AreaSigBkVe)
                if TrovBkSx:
                    TipoAzione="Dx"
                else:
                    TipoAzione="Sx"
            else:
                ImmBkDx=img_ruotata[cVe1Y:cVe1Y+Lt,cVe1X+Lv:cVe1X+Lv+Lt]
                TrovBkDx,(cBkCeXSx,cBkCeYSx),_,_ = TrovaNeri(ImmBkDx,SGreyVerdi,cv2.THRESH_BINARY,(255,0,0),AreaSigBkVe)
                if TrovBkDx:
                    TipoAzione="Sx"
                else:
                    TipoAzione="Dx"
    if not (img_ruotata is None):
        cv2.imshow('SLinea', img_ruotata)
        key = cv2.waitKey(1) & 0xFF
        time.sleep(0.1)

    print(TipoAzione)
    return TipoAzione

#SET VIDEO COMPUTING

AreaSigBkVe=10
AreaSigBk=1100       #Area ritenuta significativa per la ricerca
AreaSigCol=3000             #1800
AreaSigPal=4000 #0
AreaSigRoss=1900

SGrey=130 #80
SGreyPal=135   #130
SGreyBasso=130 #70
SGreyAlto=130 #60
SGrayOutArena=130
SGreyVerdi=160

ZonaVer=5  #Profondita rettangolo per verificare i verdi
#ZonaBkVer=70    #Superficie minima di nero accettabile per verificare i verdi


#MinRangeVeHSV=np.array([40, 40, 40]) #169
#MaxRangeVeHSV=np.array([80, 255, 250])
MinRangeVeHSV=np.array([45, 60, 90])
MaxRangeVeHSV=np.array([80, 255, 255])
 

#MinRangeRoHSVL=np.array([0, 135, 135])
#MaxRangeRoHSVL=np.array([20, 255, 255])

MinRangeRoHSVL=np.array([170, 105, 105])
MaxRangeRoHSVL=np.array([184, 255, 255])


MinRangeRoHSVH=np.array([165, 60, 60])
MaxRangeRoHSVH=np.array([180, 255, 255])

def TrovaNeri(Img,SG,TipoSoglia,ColContorno,AreaSig):  # TipoSoglia rappresenta il tipo di selezione B/W Scala Grigio Inverso
    global CentroX                                          # tipo soglia è di solito cv2.THRESH_BINARY
    global CentroY
    #Traccia il contorno e definisce il centro dello stesso
    Trov1=False
    cX1=CentroX
    cY1=CentroY
    Trov2=False
    cX2=CentroX
    cY2=CentroY
    grey = cv2.cvtColor(Img, cv2.COLOR_BGR2GRAY)
    kernel = np.ones((3, 3), np.uint8)
    #AlBk= cv2.erode(grey, kernel, iterations=2)
    #AlBk = cv2.dilate(AlBk, kernel, iterations=2)
    _,sogliaGrey =cv2.threshold(grey,SG, 255, TipoSoglia)
    AlBk = cv2.inRange(sogliaGrey, 0, 40)
    #cv2.imshow('Prova',AlBk)
    ContTorno,_ = cv2.findContours(AlBk.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    NumeroContorni = len(ContTorno)
    NumeroSelezionati=0
    if NumeroContorni > 0:
        Candidati=[]
        for NuCont in range(NumeroContorni):
            BoxNero=cv2.minAreaRect(ContTorno[NuCont])
            _,(w,h),_=BoxNero
            #seleziona i rettangoli di aree significative di rettangoli
            if w*h > AreaSig:
                box=cv2.boxPoints(BoxNero)
                (x_box,y_box) = box[0]
                Candidati.append((y_box,NuCont))
        Candidati = sorted(Candidati)
        NumeroSelezionati = len(Candidati)
        if NumeroSelezionati > 0:
            Trov1=True
            (y_best,Nu_best) = Candidati[NumeroSelezionati-1]
            RectBkAl = cv2.minAreaRect(ContTorno[Nu_best])
            (cX1,cY1) , _ ,_ = RectBkAl
            cX1=int(cX1)
            cY1=int(cY1)
            cv2.drawContours(Img,ContTorno[Nu_best],-1,ColContorno,2)
            cv2.circle(Img, (cX1,cY1), 5, ColContorno, -1)
            if NumeroSelezionati >1:
                Trov2 = True
                (y_best,Nu_best) = Candidati[NumeroSelezionati-2]
                RectBkAl = cv2.minAreaRect(ContTorno[Nu_best])
                (cX2,cY2) , _ ,_ = RectBkAl
                cX2=int(cX2)
                cY2=int(cY2) 
                cv2.drawContours(Img,ContTorno[Nu_best],-1,ColContorno,2)   
                cv2.circle(Img, (cX2,cY2), 5, ColContorno, -1)
            else:
                Trov2 = False
        else:
            Trov1 = False
            Trov2 = False
    return Trov1, (cX1,cY1), Trov2, (cX2, cY2)


def TrovaColore(Img,MinRgCol,MaxRgCol,ColContorno,AreaSig):   # Misura la dimensione del blocco colorato trovato e seleziona i piu grandi
    global CentroX
    global CentroY
    Trov1=False
    cX1=CentroX
    cY1=CentroY
    Trov2=False
    cX2=CentroX
    cY2=CentroY
    hsv = cv2.cvtColor(Img, cv2.COLOR_BGR2HSV)
    kernel = np.ones((3, 3), np.uint8)
    AlBk= cv2.erode(hsv, kernel, iterations=3)
    AlBk = cv2.dilate(AlBk, kernel, iterations=3)
    AlBk = cv2.inRange(AlBk, MinRgCol, MaxRgCol)
    #cv2.imshow("MASK", AlBk)
    ContTorno,_ = cv2.findContours(AlBk.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    NumeroContorni = len(ContTorno)
    NumeroSelezionati=0
    if NumeroContorni > 0:
        Candidati=[]
        for NuCont in range(NumeroContorni):
            BoxNero=cv2.minAreaRect(ContTorno[NuCont])
            _,(w,h),_=BoxNero
            #seleziona i rettangoli di aree significative di rettangoli
            if w*h > AreaSig:
                box=cv2.boxPoints(BoxNero)
                (x_box,y_box) = box[0]
                Candidati.append((y_box,NuCont))
        Candidati = sorted(Candidati)
        NumeroSelezionati = len(Candidati)
        if NumeroSelezionati > 0:
            Trov1=True
            (y_best,Nu_best) = Candidati[NumeroSelezionati-1]
            RectBkAl = cv2.minAreaRect(ContTorno[Nu_best])
            (cX1,cY1) , _ ,_ = RectBkAl
            cX1=int(cX1)
            cY1=int(cY1)
            cv2.drawContours(Img,ContTorno[Nu_best],-1,ColContorno,2)
            cv2.circle(Img, (cX1,cY1), 5, ColContorno, -1)
            if NumeroSelezionati >1:
                (y_best,Nu_best) = Candidati[NumeroSelezionati-2]
                RectBkAl = cv2.minAreaRect(ContTorno[Nu_best])
                (cX2,cY2) , _ ,_ = RectBkAl
                cX2=int(cX2)
                cY2=int(cY2)
                Trov2 = True
                cv2.drawContours(Img,ContTorno[Nu_best],-1,ColContorno,2)
                cv2.circle(Img, (cX2,cY2), 5, ColContorno, -1)
            else:
                Trov2 = False
        else:
            Trov1 = False
            Trov2 = False
    return Trov1, (cX1,cY1), Trov2, (cX2, cY2)

def CrosDx():           #Esegue incrocio a Destra
    #Funzione incrocio Destro
    Dritto(40)
    time.sleep(0.3)
    Sterzo(80)
    time.sleep(0.4)
    Sterzo(40)
    time.sleep(0.2)
    Dritto(-30)
    time.sleep(0.25)
    Stop()

def CrosSx():           #Esegue incrocio a Sinistra
    Dritto(40)
    time.sleep(0.3)  
    Sterzo(-80)
    time.sleep(0.9)  #1.0
    Sterzo(-40)
    time.sleep(0.2)
    Dritto(-30)
    time.sleep(0.25)
    Stop()


def CrosFF():           #Esegue Dietrofront
    #Funzione Dietro Front
    Stop()    
    time.sleep(0.4)
    Dritto(-20)
    time.sleep(0.1)
    Sterzo(100)
    time.sleep(2.5)
    Dritto(-30)
    time.sleep(0.2)
    #Sterzo(100)
    #time.sleep(1.1)
    Stop()    
    
    
def CrosArena():           #Esegue Dietrofront
    #Funzione Dietro Front
    Dritto(-50)
    time.sleep(0.2)
    Sterzo(100)
    time.sleep(1.9)
    #Dritto(-30)
    #time.sleep(0.3) 
    Stop()    

def Ostacolo():         #Esegue Agira ostacolo a Sinistra
    global VelCro
    VelCro=VelMin
    Dritto(-40)
    time.sleep(0.9)
    Sterzo(-80)            
    time.sleep(1.2) 
    Dritto(40)
    time.sleep(1.5) #1.3
    Sterzo(80)
    time.sleep(0.7)  ##1 0.85
    Dritto(40)
    time.sleep(1.9) #2.5
    Sterzo(80)
    time.sleep(0.8)
    Dritto(40)
    time.sleep(0.1) #0.7 0.5
    VelCro=VelMax

def OstacoloOpposto():         #Esegue Agira ostacolo a Destra
    global VelCro
    VelCro=VelMin
    Dritto(-40)
    time.sleep(0.9)
    Sterzo(80)            
    time.sleep(0.7) 
    Dritto(40)
    time.sleep(1.8) #1.3
    Sterzo(-80)
    time.sleep(1.5)  ##1 0.85
    Dritto(40)
    time.sleep(1.5) #2.5
    Sterzo(-80)
    time.sleep(1.6)
    Dritto(40)
    time.sleep(0.)
    VelCro=VelMax    
    
    
def Seguilinea():
    global CentroX
    global CentroY
    global Kp
    global Ka
    global InArena
    global Attesa
    global PalArg
    global VelMin
    global VelMax
    global discontinuita
    global Immagine
    giu=False
    discontinuita = False
    contagiu=0
    Errore = 0
    Cor=0
    ErrAng=0
    Ya=10
    YB=100
    Lv=60
    Hv=50
    Lt=20   #alpiezza tacchetto verifica verdi
    ###################  IMPOSTA INCLINAZIONE WEBCAM ########################
    Motori.servo[Web].angle = WebLinea  #per accorciare la visuale aumentare i gradi e viceversa
    time.sleep(0.5)
    moltiplicatore = Kp
    CONTAVerdi=0
    print("SEGUI LINEA START")
    Stop()
    time.sleep(0.2)
    start_timeCiclo = time.time()
    while True:
        

        Inclinazioni()
        #MisuraDistanze()
        Imma=Immagine
        cv2.imshow('SLinea', Imma)
        key = cv2.waitKey(1) & 0xFF
        time.sleep(0.01)

        if Attesa or InArena:
            Stop()
            cv2.destroyAllWindows()
            break
        TrovAlto= False 
        TrovBasso= False        
        TrovDestro= False
        TrovSinistro=False
        TrovBlu=False
        TrovVer1=False
        TrovVer2=False
        TrovBkVer=False
        TrovBkBasDx=False
        TrovBkBasSx=False
        TrovBkCentroSx=False
        TrovBkCentroDx=False
        cBkVeBasX=0
        cBkVeAltX=0
        if (DistanzaDx <= 14 or DistanzaSx <= 14) and yRotation > -6 and yRotation < 6: #robot è in piano e trova Presenza ostacolo
            Stop()
            time.sleep(0.5)
            if (DistanzaDx <= 14 or DistanzaSx <= 14):
                print("Ostacolo")
                print(yRotation)
                Stop()
                Ostacolo()
                #OstacoloOpposto()
        else:                           #Determina movimento Seguilinea        
            Alto = Imma[0:OffSetY,0:xDim]
            Basso = Imma[ExtY:yDim,OffSetX:ExtX] 
            Destro = Imma[OffSetY:yDim,ExtX:xDim]
            Sinistro = Imma[OffSetY:yDim,0:OffSetX]
            CentroVe =Imma[OffSetY+ZonaVer:ExtY-10,OffSetX+10:ExtX-10]
            Centro =Imma[OffSetY:ExtY,OffSetX:ExtX]
            TrovAlto,(cBkAlX,cBkAlY),_,_ = TrovaNeri(Alto,SGreyAlto,cv2.THRESH_BINARY,(52,103,171),AreaSigBk)
            TrovBasso,(cBkBaX,cBkBaY),_,_ = TrovaNeri(Basso,SGreyBasso,cv2.THRESH_BINARY,(52,103,171),AreaSigBk)
            TrovDestro,(cBkDeX,cBkDeY),_,_ = TrovaNeri(Destro,SGrey,cv2.THRESH_BINARY,(52,103,171),AreaSigBk)
            TrovSinistro,(cBkSiX,cBkSiY),_,_ = TrovaNeri(Sinistro,SGrey,cv2.THRESH_BINARY,(52,103,171),AreaSigBk)
            #TrovBkCentro,(cBkCeX,cBkCeY),_,_ = TrovaNeri(CentroVe,SGrey,cv2.THRESH_BINARY,(52,103,171),AreaSigBk)
            TrovVer1,(cVe1X,cVe1Y),TrovVer2,(cVe2X,cVe2Y) = TrovaColore(CentroVe,MinRangeVeHSV,MaxRangeVeHSV,(0,255,255),AreaSigCol)
            # VALUTA LE DUE FASCE DI ROSSO
            TrovRos1,(cBl1X,cBl1Y),_,_ = TrovaColore(Centro,MinRangeRoHSVL,MaxRangeRoHSVL,(255,0,0),AreaSigRoss)
            if not TrovRos1:
                TrovRos1,(cBl1X,cBl1Y),_,_ = TrovaColore(Centro,MinRangeRoHSVH,MaxRangeRoHSVH,(255,0,0),AreaSigRoss)
            #if TrovVer1:
            #    Stop()
            #    time.sleep(0.1)
            cv2.imshow("SLinea",Imma)
            key = cv2.waitKey(1) & 0xFF
            time.sleep(0.01)
            if not TrovAlto and not TrovBasso and not TrovDestro and not TrovSinistro and not discontinuita:
                print("###############################Non vedo niente")
                start_time = time.time()  #inizio cronometro
                discontinuita = True
                print(time.time())
            if discontinuita:
                #tempo_trascorso = int(time.time()-start_time)  #verifica tempo
                tempo_trascorso = time.time()-start_time
                tempo_fine = time.time()
                print(tempo_trascorso)
                #print("####Tempo fine", tempo_fine)
                if tempo_trascorso > 0.5: #0.7
                    Stop()
                    time.sleep(0.3)
                    Imma = Immagine
                    cv2.imshow('SLinea', Imma)
                    key = cv2.waitKey(1) & 0xFF
                    time.sleep(0.1)
                    TrovDisc,(cBkBaX,cBkBaY),_,_ = TrovaNeri(Imma,SGreyBasso,cv2.THRESH_BINARY,(52,103,171),AreaSigBk)
                    cv2.imshow('SLinea', Imma)
                    key = cv2.waitKey(1) & 0xFF
                    time.sleep(0.01)
                    if TrovDisc:
                        #Dritto(20)
                        #time.sleep(0.2)
                        Stop()
                        discontinuita = False
                    else:
                        Dritto(-20)
                        time.sleep(0.4)
                        Stop()
                        print("Cerco discontinuita")
                        Sterzo(100)
                        time.sleep(0.75)
                        Stop()
                        
                        #tolto dritto (20) time sleep 0.2
                        
                        Imma = Immagine
                        TrovDisc,(cBkBaX,cBkBaY),_,_ = TrovaNeri(Imma,SGreyBasso,cv2.THRESH_BINARY,(52,103,171),AreaSigBk)
                        cv2.imshow('SLinea', Imma)
                        key = cv2.waitKey(1) & 0xFF
                        time.sleep(0.1)
                        if TrovDisc:
                            print("Trovato Discontinuita")
                         #   Dritto(20)
                          #  time.sleep(0.2)
                            Stop()
                            discontinuita = False
                        else:
                            Dritto(-20)
                            time.sleep(0.4)
                            Stop()
                            Sterzo(-100)
                            time.sleep(1.5)
                            Stop()
                            Dritto(20)
                            time.sleep(0.4)
                            Stop()
                            time.sleep(0.1)
                            Imma = Immagine
                            TrovDisc,(cBkBaX,cBkBaY),_,_ = TrovaNeri(Imma,SGreyBasso,cv2.THRESH_BINARY,(52,103,171),AreaSigBk)
                            cv2.imshow('SLinea', Imma)
                            key = cv2.waitKey(1) & 0xFF
                            time.sleep(0.1)
                            if TrovDisc:
                                print("Trovato Discontinuita")
                                Dritto(20)
                                time.sleep(0.2)
                                Stop()
                                discontinuita = False
                            else:
                                Dritto(-20)
                                time.sleep(0.4)
                                Stop()
                                Sterzo(100)
                                time.sleep(0.75)
                                Stop()
                                discontinuita = False
                                #Forse va fatto tornare indietro fino a che non trova la linea perduta
                                Dritto(-20)
                                time.sleep(0.8)
                                Stop()
            if TrovRos1 and yRotation > -4 and yRotation < 4:               #Se incontra la linea rossa di fine gara:
                Dritto(20)                     ##
                time.sleep(0.1)
                Stop()
                print("Rosso Trovato  STOP") 
                time.sleep(7)
                #Fine Gara nel caso di lettura errata continua il seguilinea
            elif TrovVer1:               #Presenza di un incrocio singolo o doppio
                #Stop()
                #Dritto(40)                     ##
                #time.sleep(0.1)
                Stop()
                time.sleep(0.1)
                print("Verdi", (cVe1X,cVe1Y))
                #Verifico la centralità dell'incrocio per una buona lettura
                
                #if cVe1X<50 or cVe1X>190 or cVe1Y<60 or cVe1Y>190:
                    #CentraImma()
                #Stop()
                #time.sleep(1)                    
                Imma=Immagine
                cv2.imshow('SLinea', Imma)
                key = cv2.waitKey(1) & 0xFF
                time.sleep(0.1)
                Direzione = Girata()
                if Direzione == 'Dx':#or Direzione == '1 'or Direzione == '2 ':
                    print('giroDx')
                    CrosDx()
                elif Direzione == 'Sx':#or Direzione == '4 'or Direzione == '5 ':
                    print('giroSx')
                    CrosSx()
                elif Direzione == 'Fr':#or Direzione == '7 'or Direzione == '8 ':
                    print('giroFr')
                    CrosFF()
                elif Direzione == 'FF':#or Direzione == '10'or Direzione == '11':
                    print('SALTO')
                    Dritto(30)
                    time.sleep(0.4)  #0.6
                    Stop()
                else:
                    print('NON HO CAPITO')
                    Stop()
                    time.sleep(0.2)
                    Dritto(-30)
                    time.sleep(0.8)
                    Stop()
                print("Fatto")
                cv2.imshow('SLinea', Imma)
                key = cv2.waitKey(1) & 0xFF
                time.sleep(0.1)
                Stop()
                time.sleep(0.1)
            else:                               #Determina il movimento di correzione del seguilinea
                if yRotation < -17:
                    print("Accellero velocita Abbasso Pala - SALITA")
                    VelCro=VelSalita
                    print("Velocita",VelCro)
                    #Motori.servo[MPalDx].angle = 120
                    #Motori.servo[MPalSx].angle = 60
                    if TrovBasso:
                        Xa=cBkBaX
                        Ya=cBkBaY
                    else:
                        Xa=CentroX
                        Ya=ExtY
                        
                elif yRotation > 10:  #10
                    print("Decellero-discesa")
                    VelCro=VelDiscesa
                    Dritto(VelCro)
                    time.sleep(0.05)
                    print("Velocita",VelCro)
                    #Motori.servo[MPalDx].angle = 120
                    #Motori.servo[MPalSx].angle = 60
                    if TrovBasso:
                        Xa=cBkBaX
                        Ya=cBkBaY
                    else:
                        Xa=CentroX
                        Ya=ExtY        
                elif yRotation <= -45:          # Si e rovesciato in avanti, abbasso pala per rialzarlo
                    #Motori.servo[MPalDx].angle = 22
                    #Motori.servo[MPalSx].angle = 158
                    time.sleep(1)
                    Stop()
                    Dritto(-20)
                    time.sleep(0.4) #0.6
                    Stop()
                else:
                    VelCro=VelMin
                    print("Velocità ", VelCro)
                    #Motori.servo[MPalDx].angle = 174
                    #Motori.servo[MPalSx].angle = 6
                    if (TrovAlto and TrovDestro and TrovBasso)or(TrovAlto and TrovSinistro and TrovBasso) or (TrovAlto and TrovDestro and TrovSinistro and TrovBasso): 
                        if (cBkVeAltX < CentroX+40 and cBkVeAltX > CentroX-40) and (cBkVeBasX < CentroX+40) and (cBkVeBasX > CentroX-40): 
                            if (((cBkCeYSx < CentroY+35) and (cBkCeYSx > CentroY-35)) and ((cBkCeYDx < CentroY+35) and (cBkCeYDx > CentroY-35))):
                                CONTAVerde=CONTAVerde+1
                    if TrovAlto: # and not TrovDestro and not TrovSinistro:
                        #Vel=2*VelMax
                        Xa=cBkAlX
                        Ya=cBkAlY
                        moltiplicatore = Kp
                    elif TrovDestro or TrovSinistro: # il riferimento basso est sul lato destro ma nella parte inferiore
                        #moltiplicatore = boost*Kp
                        VelCro=VelMax
                        print("Velocità ", VelCro)

                        if TrovDestro and TrovSinistro:
                            if cBkSiY < cBkDeY:
                                Xa=cBkSiX
                                Ya=cBkSiY
                            else:
                                Xa=cBkDeX+ExtX
                                Ya=cBkDeY
                        elif TrovDestro:
                            if Ya >=130 and TrovBasso:
                                Stop()
                                Dritto(-20)
                                time.sleep(0.1)
                                Stop()
                                time.sleep(0.01)
                                Sterzo(80)
                                time.sleep(0.4)
                                Stop()
                                print('BOOOOOSter DX')
                                time.sleep(0.01)
                                Imma=Immagine
                                cv2.imshow('SLinea', Imma)
                                key = cv2.waitKey(1) & 0xFF
                                #time.sleep(0.1)
                                Destro = Imma[OffSetY:yDim,ExtX:xDim]
                                TrovDestro,(cBkDeX,cBkDeY),_,_ = TrovaNeri(Destro,SGrey,cv2.THRESH_BINARY,(52,103,171),AreaSigBk)
                            Xa=cBkDeX+ExtX
                            Ya=cBkDeY
                        elif TrovSinistro: # il riferimento basso est sul lato sinistro ma nella parte inferiore
                            if Ya >=130 and TrovBasso:
                                Stop()
                                Dritto(-20)
                                time.sleep(0.3)
                                Stop()
                                time.sleep(0.01)
                                Sterzo(-80)
                                time.sleep(0.4)
                                Stop()
                                print('BOOOOOSter SX')
                                time.sleep(0.01)
                                Imma=Immagine
                                cv2.imshow('SLinea', Imma)
                                key = cv2.waitKey(1) & 0xFF
                                Sinistro = Imma[OffSetY:yDim,0:OffSetX]
                                TrovSinistro,(cBkSiX,cBkSiY),_,_ = TrovaNeri(Sinistro,SGrey,cv2.THRESH_BINARY,(52,103,171),AreaSigBk)
                            Xa=cBkSiX
                            Ya=cBkSiY
                    elif TrovBasso:
                        VelCro=VelMin
                        print("Velocità ", VelCro)
                        moltiplicatore=Kp
                        Xa=cBkBaX
                        Ya=cBkBaY
                    else:
                        Xa=CentroX
                        Ya=ExtY
                ErrAng = ((Xa-CentroX)*100)/CentroX
                        
                Cor =(moltiplicatore*ErrAng)
                print(Xa, ErrAng,", ", Cor,"Dis:", DistanzaDx, DistanzaDx, Attesa, InArena )
                Sterzo(Cor)                ##
                time.sleep(0.05)
        Imma=Immagine
        cv2.imshow("SLinea",Imma)
        key = cv2.waitKey(1) & 0xFF
        time.sleep(0.1)
        tempo_trascorsoCiclo = time.time()-start_timeCiclo
        print(tempo_trascorsoCiclo)
        #print('DisDx: ',Distanza,'DistDx: ',DistaDx,'GiroX:',xRotation,'GiroY: ',yRotation, 'Arena: ',InArena, 'Attesa ', Attesa)
        if Attesa:
            Stop()
            cv2.destroyAllWindows()
            break
        time.sleep(0.01)
        
        if InArena:
            Stop()
            time.sleep(0.5)
            if yRotation > -9 and yRotation < 9: 
                cv2.destroyAllWindows()
                time.sleep(0.01) 
                break
            else:
                InArena = False
                Stop()
                time.sleep(1)
        start_timeCiclo = time.time()        
    Stop()

def TrovaPalla(Img,ColContorno):  # TipoSoglia rappresenta il tipo di selezione B/W Scala Grigio Inverso
    global CentroX                                          # tipo soglia è di solito cv2.THRESH_BINARY
    global CentroY
    #Traccia il contorno e definisce il centro dello stesso
    #Motori.servo[Web].angle = WebStanza
    time.sleep(0.5)    
    Trov1=False
    PallinaBk=None
    xM=CentroX
    yM=CentroY
    rag=0
    ImmPalla=None
    grey = cv2.cvtColor(Img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(grey, (7, 7), 0)
    kernel = np.ones((3, 3), np.uint8)
    dilation = cv2.dilate(blur, kernel, iterations=4)
    Pulita = cv2.erode(dilation, kernel, iterations=4)
    #cv2.imshow('blur', Pulita)                         #1.2,220,param1=35,param2=35,minRadius=7,maxRadius=60,
    circles = cv2.HoughCircles(Pulita,cv2.HOUGH_GRADIENT,1.1,220,param1=35,param2=30,minRadius=9,maxRadius=80,)
    if circles is not None:                             #Se ci sono cerchi rilevati
        Trov1=True
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:                       #Seleziona quello piu grande
            if r > rag:
                xM, yM, rag = x, y, r
    if Trov1:
        MinX=xM-50
        if MinX<0:
            MinX=0
        MinY=yM-50
        if MinY<0:
            MinY=0
        MaxY=yM+50
        if MaxY> yDim:
            MaxY=yDim
        MaxX=xM+50
        if MaxX> xDim:
            MaxX=xDim
        ImgPalla=Img[MinY:MaxY,MinX:MaxX]
        PallinaBk,(cBkCeX,cBkCeY),_,_ = TrovaNeri(ImgPalla,SGreyPal,cv2.THRESH_BINARY,(52,103,171),AreaSigPal)
        cv2.circle(Img, (xM, yM), rag, ColContorno, 4)
        cv2.rectangle(Img, (xM - 3, yM - 3),(xM + 3, yM + 3), ColContorno, -1)        
    cv2.imshow("Immagine", Img)
    key = cv2.waitKey(1) & 0xFF
    time.sleep(0.01)
    print(Trov1,xM,yM,rag,not (PallinaBk))
    return Trov1, (xM,yM),rag,not(PallinaBk)
    
    
def ScaricaArg():                    #Trova il triangolo verde e scarica la pinza
    global CentroX
    global Uscita
    global Attesa
    VerdeOK=False
    ContGiro=0
    ###################  IMPOSTA INCLINAZIONE WEBCAM ########################
    Motori.servo[Web].angle = WebStanza
    time.sleep(0.5)
    
    while True:
        Imm = Immagine
        cv2.imshow("Immagine",Imm)
        key = cv2.waitKey(1) & 0xFF
        time.sleep(0.01)
        if Attesa or VerdeOK:
            #Uscita=True
            break
        TrovVer1,(cVe1X,cVe1Y),TrovVer2,(cVe2X,cVe2Y) = TrovaColore(Imm,MinRangeVeHSV,MaxRangeVeHSV,(161,0,244),AreaSigCol)
        cv2.imshow("Immagine",Imm)
        key = cv2.waitKey(1) & 0xFF

        if TrovVer1:
            #if DistanzaDx > 30 or DistanzaSx > 30:
            if cVe1Y<50:
                if cVe1X < (CentroX -15):
                    Sterzo(-40)
                    time.sleep(0.05)
                    #print("Muovi -100")
                elif cVe1X > (CentroX +15):
                    Sterzo(40)
                    time.sleep(0.05)
                    #print("Muovi +100")
                else: 
                    #print("Vai avanti")
                    Sterzo(4)
                    time.sleep(0.05)
            #elif DistanzaDx < 40 or DistanzaSx < 40:
            else:
                Stop()
                print("Vicino Triangolo")
                Dritto(40)
                time.sleep(1.6)
                Stop()
                Dritto(-40)
                time.sleep(0.4)
                Stop()
                ScaricaPalla()   #ScaricaPalla
                time.sleep(0.5)
                Dritto(-50)
                time.sleep(1)
                CrosFF()
                Dritto(-50)
                time.sleep(1.5)
                Stop()
                time.sleep(0.2)
                VerdeOK=True
                ContGiro = 8
        else:
            ContGiro=ContGiro+1
            if ContGiro == 6:
                Dritto(70)
                time.sleep(1.3)   #3.3
                Stop()
                ContGiro=0
            else:    
                Sterzo(60)
                time.sleep(0.5)
                Stop()
                if DistanzaDx < 20 or DistanzaSx < 20:
                    print("Allontano Parete")
                    while DistanzaDx < 30 or DistanzaSx < 30:
                        Dritto(-70)
                    Stop()   
                    Sterzo(45) 
                    time.sleep(1.2) 
                    Stop()
                time.sleep(0.3)


        cv2.imshow("Immagine",Imm)
        key = cv2.waitKey(1) & 0xFF
        time.sleep(0.01)
        if Attesa or VerdeOK:
            #Uscita=True
            break
        time.sleep(0.15)
        
def Arena():
    global Immagine
    global CentroX
    global Uscita
    global Attesa
    global PalArg
    global OutArena
    global VelCro
    global DistanzaDx
    global DistanzaSx
    global DistanzaLato
    Arg=0
    Nero=0
    Viva=True
    ContGiro=0
    VelCro=VelMax
    print("Arena aperta")
    ###################  IMPOSTA INCLINAZIONE WEBCAM ########################
    if Attesa or OutArena:
        Stop()
        print("Esci Arena")
        time.sleep(1)
        #cv2.destroyAllWindows()
        #break
    else:
        Motori.servo[Web].angle = WebStanza
        print("Entra Arena")
        time.sleep(0.5)    #0.1
        Sterzo(0)              #Siamo sulla riga argento quindi entra dentro l'arena
        time.sleep(1.0)
        Stop()
        time.sleep(0.1)
    while True:
        if Arg>=3:
            OutArena= True
            time.sleep(0.1)
        else:
            OutArena= False
 
        if Attesa or OutArena:
            Stop()
            cv2.destroyAllWindows()
            break
        #Stop()
        Imma = Immagine
        print("Trovo Palla")
     
        TrovPal,(cPalX,cPalY),R,PalArg = TrovaPalla(Imma,(0,255,0))
        time.sleep(0.1) 
        if TrovPal: 
            cv2.imshow("Imma", Imma)
            key = cv2.waitKey(1) & 0xFF
            if cPalY < 70:     #170                # Avvicinamento alla pallina trovata
                print("Punto")
                if cPalX < (CentroX -15):
                    Sterzo(-30)  #20
                    #time.sleep(0.35)  #0.20
                    print("Muovi Sx")
                    #Stop()
                elif cPalX > (CentroX +15):
                    Sterzo(30)  #20
                    #time.sleep(0.35)  #0.20
                    print("Muovi DX")
                    #Stop()
                else:    
                    print("Vai avanti")
                    Sterzo(0)
                    #time.sleep(0.40)  #0.20
                    #Stop()
            else:                               # Spostamento per Caricare la pallina     
                Stop()
                print("Arrivato")
                Motori.servo[Web].angle = WebLinea
                Dritto(-60)
                time.sleep(0.4)
                Stop()
                AbbassaPala()
                Dritto(60)
                time.sleep(1.4)
                Stop()
                ChiudiPinza()
                time.sleep(0.5)
                Dritto(-60)
                time.sleep(0.5)
                Stop()
                time.sleep(0.1)
                AlzaPala()
                time.sleep(1)
                Motori.servo[Web].angle = WebStanza
                Arg=Arg+1  
                ScaricaArg()
                print("VERDE")
        else:
            cv2.imshow("Imma", Imma)
            key = cv2.waitKey(1) & 0xFF
            time.sleep(0.001)  #0.01
            print("Cerco Palla")
            #Stop()
            ContGiro=ContGiro+1
            if DistanzaDx < 20 or DistanzaSx < 20:
                Dritto(-60)
                time.sleep(1.5)  # 1.1
                #Stop()
                Sterzo(60)
                time.sleep(1.1)     # 1.5
                #Stop()
                    
            elif ContGiro == 1:
                print("Cerco Destra", ContGiro)
                Sterzo(-60)       #60
                time.sleep(0.8)   #0.7
                #Stop()
                #time.sleep(0.5)

            elif ContGiro == 2:
                print("Cerco Destra", ContGiro)
                Sterzo(-60)       #60
                time.sleep(0.8)   #0.7
                #Stop()
                #time.sleep(0.5)
                
            elif ContGiro == 3:
                print("Cerco Destra", ContGiro)
                Sterzo(60)
                time.sleep(0.5)
                #Stop()
                #time.sleep(0.5)  
 
            elif ContGiro == 4:
                print("Cerco Destra", ContGiro)
                Sterzo(60)
                time.sleep(0.5)
                #Stop()
                #time.sleep(0.5)  
 
            elif ContGiro == 5:
                print("Cerco Destra", ContGiro)
                Sterzo(-60)   #60
                time.sleep(0.8)   #0.7
                #Stop()
                #time.sleep(0.5) 
            
            elif ContGiro == 6:
                print("Cerco Destra", ContGiro)
                Sterzo(-60)
                time.sleep(0.5)
                #Stop()
                #time.sleep(0.5) 
                
                           
            else:
                print("Cerco Dritto", ContGiro)
                #Sterzo(60)
                #time.sleep(0.9)
                #Stop()
                Dritto(60)
                time.sleep(1)
                #Stop()
                time.sleep(0.6)
                ContGiro=0
            cv2.imshow("Immagine",Imma)
            key = cv2.waitKey(1) & 0xFF
        time.sleep(0.01)
        if Attesa or OutArena:
            Stop()
            cv2.destroyAllWindows()
            
            
        time.sleep(0.1)
    VelCro=VelMax
        


def TrovaUscita():
    global Immagine
    global Uscita
    global Attesa
    Trovata=False
    ContGiro=0
    VelCro=VelMin
    ###################  IMPOSTA INCLINAZIONE WEBCAM ########################
    Motori.servo[Web].angle = WebLinea   #WebUscita #WebStanza
    time.sleep(1)
    TrovNero=False
    NeroOK=False
    TrovExit=False
    Indietro=False
    TrovNeroFro=False
    Dritto(-40)              
    time.sleep(0.5)
    Stop()
    Sterzo(100)              
    time.sleep(1)
    Stop()
    Dritto(40)              
    time.sleep(0.6)
    Stop() 
    Sterzo(100)              
    time.sleep(0.8)
    Stop()
    Dritto(40)              
    time.sleep(0.6)
    Stop()         
    while True:    
        if DistanzaLato > 45:      #35
            TrovNero =True
            print ("Trovato Buco Sinistra")
        else:
            if DistanzaDx > 75 or DistanzaSx > 75:  #35
                TrovNeroFro =True
                print ("Trovato Buco Fronte")
            else:
                if DistanzaDx < 20 or DistanzaSx < 20:  #35
                    Sterzo(100) 
                    time.sleep(1)
                    Stop()
                    print ("Giro al Muro")                    
        if not (TrovNero or TrovNeroFro) and not Attesa:
            Dritto(40)              
            time.sleep(1.2)
            Stop()
        else:
            if TrovNero:
                Sterzo(-100)              
                time.sleep(1.2)
                Stop()
            Dritto(40)              
            time.sleep(1)
            Stop()
        Imma=Immagine
        cv2.imshow("Image",Imma)
        key = cv2.waitKey(1) & 0xFF
        time.sleep(0.01)
        if not Attesa or TrovNero or TrovNeroFro:
            cv2.destroyAllWindows()
            break
        time.sleep(0.1)        
              
    
Motori.servo[Web].angle = WebSicura    # WEB Guarda a terra per la linea  15-17
time.sleep(0.5)
Motori.servo[MPalDx].angle = 0
Motori.servo[MPalSx].angle = 180
time.sleep(0.2)
Motori.servo[MPzDx].angle = 38
Motori.servo[MPzSx].angle = 160
time.sleep(0.2)
Motori.servo[Web].angle = WebLinea
print('Tutto Pronto!!!!')
time.sleep(0.2)

while True:
    #LeggiComandi()
    Inclinazioni()
    #MisuraDistanze()
    print('DisDx: ',DistanzaDx,'DisSx: ',DistanzaSx,'DisLato: ',DistanzaLato,'GiX: ',xRotation,'GiY: ',yRotation, "GiZ ", zRotation, 'Are: ', InArena, 'Att ', Attesa)
    Imma=Immagine
    cv2.imshow('SLinea', Immagine)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q") or ChiudiThred:
        ChiudiThred=True
        break    
    time.sleep(0.1)

    if not Attesa:
        print('AZIONE')
        Imma=Immagine
        cv2.imshow('SLinea', Imma)
        time.sleep(0.1)
        
        
        if Attesa:
            time.sleep(0.1)
        else:
            #cv2.destroyAllWindows()
            #print('Entro Seguilinea')
            Seguilinea()
            #print('Fine Seguilinea')
            #TrovaUscita()
            #ScaricaRosso()
            #ScaricaVerde()
            #GiroGrSx(180)
            #Dritto(20)
            #time.sleep(1)
            #Dritto(-20)
            #time.sleep(2)
            #time.sleep(3)
            #AbbassaPala()
            #time.sleep(3)
            #AlzaPala()
            #time.sleep(3)
            #ChiudiPinza()
            #time.sleep(3)
            #ApriPinza()
            #time.sleep(3)
            #ChiudiPinza()
            #time.sleep(3)
            #AlzaPala()
            #time.sleep(3)
            #AbbassaPala()
            #time.sleep(3)
            #ChiudiPinza()
            #time.sleep(3)
            #AlzaPala()
            #time.sleep(3)
            #ScaricaPalla()
            #time.sleep(3)


        
        if Attesa or not OutArena:
            Arena()
            time.sleep(0.1)
        else:
            #provUscita()
            #TrovaUscita()
            time.sleep(0.1)
            
        if Attesa or not OutArena:
            TrovaUscita()
            time.sleep(0.1)
        else:
            #provUscita()
            #TrovaUscita()
            time.sleep(0.1)
         
    else:
        print('---PAUSA---')
        time.sleep(0.1)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q") or ChiudiThred:
        ChiudiThred=True
        break
    time.sleep(0.05)
    
Stop()
ChiudiThred=True        
time.sleep(1)    
cv2.destroyAllWindows()
