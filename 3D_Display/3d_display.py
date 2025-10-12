from vpython import *
import serial
import math
import numpy as np

# --- Initialisation VPython ---
scene = canvas(title="Orientation MPU (quaternion)")
cube = box(size=vector(1,1,1), color=vector(0.2,0.5,0.8))

# --- Texte ---
text_q = wtext(text="Quaternion: 0,0,0,0")
scene.append_to_caption("\n")

def set_orientation_from_quaternion(cube, q0, q1, q2, q3):
    # Normaliser (au cas où)
    norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
    q0, q1, q2, q3 = q0/norm, q1/norm, q2/norm, q3/norm

    # Matrice de rotation issue du quaternion
    R = np.array([
        [1 - 2*(q2*q2 + q3*q3),     2*(q1*q2 - q0*q3),     2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3),         1 - 2*(q1*q1 + q3*q3), 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2),         2*(q2*q3 + q0*q1),     1 - 2*(q1*q1 + q2*q2)]
    ])

    # Appliquer la rotation au cube
    cube.axis = vector(R[0,0], R[1,0], R[2,0])
    cube.up   = vector(R[0,2], R[1,2], R[2,2])

# --- Port série ---
ser = serial.Serial('COM8', 115200)

while True:
    rate(50)
    try:
        line = ser.readline().decode('utf-8').strip()
        if not line:
            continue

        # Attendu : "q0,q1,q2,q3"
        q0, q1, q2, q3 = map(float, line.split(','))
        set_orientation_from_quaternion(cube, q0, q1, q2, q3)
        text_q.text = f"Quaternion: {q0:.3f}, {q1:.3f}, {q2:.3f}, {q3:.3f}"

    except Exception as e:
        print("Erreur lecture port série:", e)


"""
from vpython import *
import serial
import math
import numpy as np

# --- Initialisation VPython ---
scene = canvas(title="Orientation MPU")
cube = box(size=vector(1,1,1), color=vector(0.2,0.5,0.8))

# Texte pour afficher les angles
roll_text = wtext(text="Roll: 0.0°")
pitch_text = wtext(text=" | Pitch: 0.0°")
yaw_text = wtext(text=" | Yaw: 0.0°")
scene.append_to_caption("\n")  # Saut de ligne
def set_orientation(cube, roll, pitch, yaw):
    # Convertir degrés en radians
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    # Matrice de rotation ZYX
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw),  math.cos(yaw), 0],
                   [0, 0, 1]])
    Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                   [0, 1, 0],
                   [-math.sin(pitch), 0, math.cos(pitch)]])
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(roll), -math.sin(roll)],
                   [0, math.sin(roll),  math.cos(roll)]])

    R = Rz @ Ry @ Rx
    cube.axis = vector(R[0,0], R[1,0], R[2,0])
    cube.up   = vector(R[0,2], R[1,2], R[2,2])

# --- Initialisation du port série ---
ser = serial.Serial('COM8', 115200, timeout=1)  # Remplace 'COM3' par ton port

while True:
    rate(50)  # Limiter à 50 FPS

    try:
        line = ser.readline().decode('utf-8').strip()

        if not line:
            continue
        #print(line)
        # Attendu : "roll,pitch,yaw"
        roll_str, pitch_str, yaw_str = line.split(',')
        roll = float(roll_str)
        pitch = float(pitch_str)
        yaw = float(yaw_str)
        set_orientation(cube, roll, pitch, yaw)
        # Mise à jour du texte à l'écran
        roll_text.text = f"Roll: {roll:.2f}°"
        pitch_text.text = f" | Pitch: {pitch:.2f}°"
        yaw_text.text = f" | Yaw: {yaw:.2f}°"
    except Exception as e:
        print("Erreur lecture port série:", e)
"""
