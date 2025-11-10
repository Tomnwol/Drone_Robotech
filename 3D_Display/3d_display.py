from vpython import *
import math
import numpy as np
import serial
# --- Initialisation VPython --- 
scene = canvas(title="Orientation Drone", width=600, height=600, align='left')
cube = box(size=vector(1,1,1), color=vector(0.2,0.5,0.8))
x_axis = arrow(pos=cube.pos, axis=vector(1,0,0), color=color.red) 
y_axis = arrow(pos=cube.pos, axis=vector(0,1,0), color=color.green) 
z_axis = arrow(pos=cube.pos, axis=vector(0,0,1), color=color.blue) 
# --- Texte --- 
text_q = wtext(text="Quaternion: 0,0,0,0") 
scene.append_to_caption("\n") 
scene.forward = vector(0, -1, 0) 
# Caméra regarde vers le bas (axe Y-) 
scene.up = vector(0, 0, 1) 
# Le haut de la caméra est l’axe Z 
scene.center = vector(0, 0, 0) 
# Centre de la scène 

# --- Panneau latéral à droite ---
panel = canvas(width=300, height=500, background=color.white, align='right')
text_q = wtext(canvas=panel, text="Quaternion: 0,0,0,0\n")
panel.append_to_caption("\n\t\t--- Puissance moteurs ---\n\n\n")

def no_action(s): pass
panel.append_to_caption("\t\tFL\t----------------\tFR\n")
slider1 = slider(canvas=panel, min=48, max=2047, value=0, length=200, bind=lambda s: no_action())
slider2 = slider(canvas=panel, min=48, max=2047, value=0, length=200, bind=lambda s: no_action())
panel.append_to_caption("\n\n\n")
panel.append_to_caption("\t\tBL\t----------------\tBR\n")
slider3 = slider(canvas=panel, min=48, max=2047, value=0, length=200, bind=lambda s: no_action())
slider4 = slider(canvas=panel, min=48, max=2047, value=0, length=200, bind=lambda s: no_action())


txt_throttle = wtext(canvas=panel, text="\n\n\n\nMFL=0% | MFR=0% | MBL=0% | MBR=0%\n")

# --- Fonction quaternion -> orientation ---
def set_orientation_from_quaternion(cube, q0, q1, q2, q3):
    norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
    q0, q1, q2, q3 = q0/norm, q1/norm, q2/norm, q3/norm

    R = np.array([
        [1 - 2*(q2*q2 + q3*q3), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3), 1 - 2*(q1*q1 + q3*q3), 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1*q1 + q2*q2)]
    ])

    x_vec = vector(R[0,0], R[1,0], R[2,0])
    y_vec = vector(R[0,1], R[1,1], R[2,1])
    z_vec = vector(R[0,2], R[1,2], R[2,2])

    cube.axis = x_vec
    cube.up = z_vec
    x_axis.axis = x_vec
    y_axis.axis = y_vec
    z_axis.axis = z_vec

# --- Boucle principale ---
ser = serial.Serial('COM8', 115200) 
while True: 
    rate(50)
    try: 
        line = ser.readline().decode('utf-8').strip() 
        if not line: continue
        data_read = map(float, line.split(','))
        print(line)
        q0, q1, q2, q3, MOT_FL, MOT_FR, MOT_BL, MOT_BR = data_read #Ajouter les valeurs des moteurs
        set_orientation_from_quaternion(cube, q0, q1, q2, q3)
        text_q.text = f"Quaternion: {q0:.3f} | {q1:.3f} | {q2:.3f} | {q3:.3f}\n"
        slider1.value, slider2.value, slider3.value, slider4.value = MOT_FL, MOT_FR, MOT_BL, MOT_BR #Ajouter les valeurs des moteurs
        txt_throttle.text = f"\n\n\n\nMFL={MOT_FL} | MFR={MOT_FR} | MBL={MOT_BL} | MBR={MOT_BR}\n"  
    except Exception as e: 
        #pass
        print("Erreur lecture port série:", e)
