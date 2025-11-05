#MultiBender-Radar-System
Dieses Repo beinhaltet die Treiber für die Sensoren sowie den Code für die Lokalisierung mit dem Mulitbenderradarsystems
## RADAR-Sensoren starten 
Zum Starten der Radarsensoren muss der ordner mmwave_ti_ros im Terminal geöffnet werden. 
Danach muss in den Ornder **mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg/launch** navigiert werden 

Um die 4 Sensoren gleichzeitig zu Starten gibt es eine Startskript Datei namens **"StartSkriptmmWaveRadar.py"** dieses muss im Terminal ausgeführt werden. 
Nachdem ausführen werden die 4 Sensoren in einer Reihenfolge gestartet. 

Wenn man auf den Aufbau schaut an der flasche startet der Sensor links als ersten und mitte und dann rechts und zum schluss oben 

Wenn alles gestartet ist sollten unter ros2 topic list folgende Topics auftauchen 

1. /ti_mmwave/radar_scan_pcl_0 links 
2. /ti_mmwave/radar_scan_pcl_1 mitte 
3. /ti_mmwave/radar_scan_pcl_2 rechts 
4. /ti_mmwave/radar_scan_pcl_3 oben

## IMU Starten 
ZUm starten der IMU muss der der ordner imu_publisher genutzt werden un im Terminal geöffnet werden. 

Dort muss dann in folgenden Ordner navigiert werden: **imu_publisher/src/imu_node/launch** 

Im dem launch Ordner befindet sich eine ros2 launch.py Datei die ausführbar ist und den imu_publisher startet : 

**ros2 launch imu_launch.py** 

Nachdem launchen der Node sollte folgendes Topic auftauchen 

1. /imu/data_BNO0x

## Starten der Lokalisierung mit ICP und der Rotation der IMU 

Im Workspace MultiBender befindet sich der Code zum starten der Lokalisieurng mit ICP. 

Dort gibt es im Unterverzeichnis **MultiBender/src/Bender_Location/launch eine launch.py** mit dem Namen bender_radar_launch.py 

Diese kann gestartet werden mit : 

**ros2 launch bender_radar_launch.py**

Mit dem starten des Launch-Files werden folgende Nodes gestartet: 

1. bender_radar_processing
2. bender_radar_filter
3. bender_radar_icp
4. bender_radar_visu

Nachdem Start sollten folgende Topic vorhanden sein : 

1. /combined_pcl		Kombinierte Punktwolke 
2. /filtered_pcl		Gefilterte Punktwolke 
3. /aligned_pcl		Registrierte Punktwolke
4. /pose_estimation	Zusammengefügte Posen 
5. /translation_estimation Einzelne Posen 
6. /path/radar 		Trajektorie aus den zusammengefügten Posen
7. /map			Map-Server (siehe Unten wie dieser Konfiguriert wird) 

## Sensordaten aufnahme mit ros2 bag 
Um die rohen Sensordaten aufzunehmen muss in einer neuen Konsole ein rosbag2 gestartter werden dieses lässt sich wie folgt machen für die Radare plus IMU: 

**ros2 bag record /ti_mmwave/radar_scan_pcl_0 /ti_mmwave/radar_scan_pcl_1 /ti_mmwave/radar_scan_pcl_2 /ti_mmwave/radar_scan_pcl_3 /imu/data_BNO0x**

Nachdem ausführen des Befehls sollte er anzeigen das er diese 5 Topic abonniert/subscribed hat. 

## Sensordaten abspielen
Zunächst muss in das Verzeichnis navigiert werden in welchem die bag(SqLite3-Dateien) liegen. 
Um diese Daten abzuspielen muss folgender Befehl genutzt werden : 

**ros2 bag play "Name der Datei"**   

**Beispiel: ros2 bag play rosbag2_2024_10_22-14_29_27**

Danach sollte die Aufnahme gestartet sein und die Topics aus der Aufnahme werden veröffentlicht. 









