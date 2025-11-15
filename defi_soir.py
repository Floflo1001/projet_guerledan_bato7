import time
import numpy as np
import math
import sys

sys.path.append('../drivers-ddboat-v2')
import imu9_driver_v2 as imuDD
import arduino_driver_v2 as DD
import gps_driver_v2 as GPS

#target_lat = 48.19925333333334  #tableau
#target_lon = -3.0161133333333328

#target_lat = 48.199055
#target_lon = -3.016136666666667

target_lat = 48.199245000000005
target_lon = -3.0163166666666665

file_path = "traj.txt"

#target_lat = 48.198906666666666 # gros ponton
#target_lon = -3.0147199999999996

lat_pont= 48.198906666666666 #ponton
long_pont = -3.0147199999999996

rho=6378137

def repere(latp,longp,lato,longo):
    latp=latp*np.pi/180
    longp=longp*np.pi/180
    lato=lato*np.pi/180
    longo=longo*np.pi/180
    xtilde=(longo-longp)*np.cos(lato)*rho
    ytilde=rho*(lato-latp)
    return xtilde, ytilde

def angle_demande(x1, y1, x2, y2):
    dx=x2-x1
    dy=y2-y1
    norme = np.sqrt(dx**2+dy**2)
    print ("norme = ", norme)
    psid=np.arctan2(dx,dy)
    return psid, norme

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

k=3

vi=100

boue_atteinte= [False, True]

def sawtooth(x):
    return (x+np.pi)%(2*np.pi)-np.pi

def wd(psid, psi):
    return k*np.degrees(sawtooth(psid-psi))

def vd(norme, angle):
    return vi + norme/2   

def FFC(wd0, vd0):
    arduino.send_arduino_cmd_motor(vd0+wd0, vd0-wd0)

def convertir_gps(data):
    """
    data : [lat, 'N'/'S', lon, 'E'/'W', ...]
    Renvoie (latitude, longitude) en degrés décimaux
    """

    raw_lat = data[0]
    lat_dir = data[1]
    raw_lon = data[2]
    lon_dir = data[3]

    # --- Conversion latitude ---
    lat_deg = int(raw_lat / 100)
    lat_min = raw_lat - lat_deg * 100
    lat = lat_deg + lat_min / 60.0

    # --- Conversion longitude ---
    lon_deg = int(raw_lon / 100)
    lon_min = raw_lon - lon_deg * 100
    lon = lon_deg + lon_min / 60.0

    # --- Gestion des signes ---
    if lat_dir == 'S':
        lat = -lat
    if lon_dir == 'W':
        lon = -lon

    return lat, lon

lat_list = [48.198755,] # coord du quai 
long_list = [-3.01408333333333]                

with open("traj.txt", "w") as fichier:

    # --- Étape 3 : Lecture temps réel et calcul du cap ---
    for _ in range(2000):
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
        print("cap du bateau", psi_deg_360 )
        print ("cap du bateau en rad", psi_rad)

        # Commande moteur en fonction du cap
        '''if 90 < psi_deg_360 < 270:
                        arduino.send_arduino_cmd_motor(100, 30)
                    else:
                        arduino.send_arduino_cmd_motor(30, 100)'''


        rmc_ok,rmc_data=gps.read_rmc_non_blocking()
        print('cest ça 2', rmc_data)
        if rmc_ok and rmc_data[0] != 0:
            print('cest ça', rmc_data)
            print(convertir_gps(rmc_data))
            latitude,longitude=convertir_gps(rmc_data)
            fichier.write(str(latitude)+";"+str(longitude)+";"+str(psi_deg_360)+"\n")
            lat_list.append(latitude)
            long_list.append(longitude)

        xbateau,ybateau = repere(lat_list[0],long_list[0],lat_list[-1],long_list[-1])    

        if True in boue_atteinte : 
            xobj, yobj  = repere(lat_list[0],long_list[0],lat_pont, long_pont)
        else : 
            xobj, yobj  = repere(lat_list[0],long_list[0],target_lat, target_lon)


        psid=angle_demande(xbateau,ybateau,xobj, yobj)[0]
        norme=angle_demande(xbateau,ybateau,xobj, yobj)[1]
        psid2= (np.degrees(psid)+360)%360

        if norme < 5 : 
            boue_atteinte.append(True)

        print('norme',norme)
        print('boue_atteinte',boue_atteinte)    

        print('list_bat', lat_list)
        print('lat bat', lat_list[-1], 'long bat', long_list[-1])

        print("angle demande =", psid2)
        dif_cap=psid2-psi_deg_360
        print("différence entre cap et cap demandé", dif_cap)
        FFC(wd(psid, psi_rad), vd(norme,dif_cap))


        time.sleep(0.1)

    # Stop moteurs à la fin
    arduino.send_arduino_cmd_motor(0, 0)