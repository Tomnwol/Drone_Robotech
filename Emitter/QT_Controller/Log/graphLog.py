import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Lire le CSV
df = pd.read_csv("data.csv", header=None)

# Créer l'axe temps (20 Hz)
fs = 20  # fréquence d'échantillonnage
t = np.arange(len(df)) / fs

# Tracé
plt.plot(t, df[0], label="FL Motor")
plt.plot(t, df[1], label="FR Motor")
plt.plot(t, df[2], label="BR Motor")
plt.plot(t, df[3], label="BL Motor")

plt.xlabel("Temps (s)")
plt.ylabel("Valeur")
plt.title("Commandes moteurs en fonction du temps")
plt.legend()
plt.grid()

plt.show()
