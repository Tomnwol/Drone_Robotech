Bienvenue dans le git du drone de Robotech 2025/2026.
Notre projet est de créer un drone 5" basé sur une ESP32.

-- Démarche globale --

	L'idée principale est d'utiliser un IMU 9 axes (accéléromètre, gyroscope, magnétomètre) pour obtenir une estimation d'attitude et ainsi pouvoir obtenir une commande angulaire qui sera par la suite exécutée par les moteurs.

-- COMPOSANTS --

- Microprocesseur : ESP32 DevKit
- Accéléromètre/Gyroscope : MPU-9265
- Magnétomètre : GY-271 (optionnel)
- Moteurs : XING-E Pro 2207 1800KV 2-6S - Iflight
- ESC : Hobbywing - XRotor Micro 65A G2 4in1 AM32
- Batterie : LiPo GNB 6S 2200mAh 120C - XT60 - Gaoneng
- Antenne RF : NRF24L01 + PA/LNA
- Frame : créée sur Fusion et imprimée en 3D
- PCB : créé sur Kicad 9
	

-- CODE --

	Pour le code, après avoir eu des problèmes de librairie en ESP-IDF sur PlatformIo pour le DSHOT, nous avons finalement migré vers Arduino IDE pour avoir une version plus récente de la librairie DSHOTRMT.

--SIMULATION--

	Un fichier de simulation est disponible dans le répertoire /Simulation et permet de visualiser si le calcul d'attitude est cohérent et de visualiser 4 réponses moteurs, permettant ainsi de modifier les PID en conséquence.


	

