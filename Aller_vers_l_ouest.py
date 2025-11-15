import time
import numpy as np
import math
import sys

sys.path.append('../drivers-ddboat-v2')
import imu9_driver_v2 as imuDD
import arduino_driver_v2 as DD
import gps_driver_v2 as GPS


imu = imuDD.Imu9IO()   # Ajuste selon ton driver
arduino = DD.ArduinoIO()
gps = GPS.GpsIO()

# --- Matrices de calibration corrigées ---
Q_sqrt = np.array([
    [3.07076411e-04,  9.28607201e-06, -1.18149596e-06],
    [9.28607201e-06,  3.14851314e-04, -1.38932829e-06],
    [-1.18149596e-06, -1.38932829e-06, 3.06167699e-04]
])

b = np.array([-7.50452182e-10, -2.44257380e-08, 8.24672782e-09])

n = 0


k=20

def sawtooth(x):
    return (x+np.pi)%(2*np.pi)-np.pi

def wd(psid, psi):
    return k*sawtooth(psid-psi)

def FFC(wd0):
    if wd0>0:
        arduino.send_arduino_cmd_motor(150, 50+k*np.pi-wd0)
    else: 
        arduino.send_arduino_cmd_motor(50+k*np.pi+ wd0,150)



# --- Étape 3 : Lecture temps réel et calcul du cap ---
while n < 600:
    # Lecture IMU
    xmag, ymag, zmag = imu.read_mag_raw()
    accx, accy, accz = imu.read_accel_raw()

    # Calibration des mesures magnétiques
    y = Q_sqrt @ (np.array([xmag, ymag, zmag]) - b)
    #print(y)

    # Vecteurs de référence mesurés
    y_N_mes = [-0.77446287, 0.04003321, -0.97301665]
    y_W_mes = [-0.26368797, 0.32461022, -1.0153281]
    y_up_mes = [0.64377218, -0.33588851, -0.38119525]



    y_mes = np.array([y_N_mes, y_W_mes, y_up_mes]).T

    i = 64 * np.pi / 180
    Z = np.array([
        [np.cos(i), 0, -np.sin(i)],
        [0, -np.cos(i), 0],
        [-np.sin(i), -np.sin(i), np.cos(i)]
    ])

    R_imu = Z @ np.linalg.inv(y_mes)
    z = R_imu @ y

    # Construction de la base locale
    a1 = np.array([0, 0, 1])
    C1 = (z - (z.T @ a1) * a1)
    C1 /= np.linalg.norm(C1)
    C2 = np.cross(a1, C1)
    C3 = a1
    R = np.array([C1, C2, C3]).T

    # Calcul du cap (psi)
    psi = np.arctan2(R[1, 0].real, R[0, 0].real)
    psi_deg = psi*180/(np.pi)
    psi_deg_360 = (psi_deg + 360)%360
    psi_rad=psi_deg_360/180*(np.pi)
    #print(psi_deg_360)

    # Commande moteur en fonction du cap
    '''if 90 < psi_deg_360 < 270:
                    arduino.send_arduino_cmd_motor(100, 30)
                else:
                    arduino.send_arduino_cmd_motor(30, 100)'''
    print(wd(270*np.pi/180, psi_rad))
    FFC(wd(270*np.pi/180, psi_rad))

    rmc_ok,rmc_data=gps.read_rmc_non_blocking()
    if rmc_ok:
        print (rmc_data)

    time.sleep(0.1)
    n += 1


# Stop moteurs à la fin
arduino.send_arduino_cmd_motor(0, 0)


