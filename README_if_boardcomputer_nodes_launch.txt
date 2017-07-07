VORBEDINGUNG:
- öffne putty, gehe auf vision, drücke load, drücke open, password: hyq ==> du bist mit dem borad computer verbunden. du musst das FÜR JEDES SKRIPT neu machen. 
- öffne XMING (es öffnet sich nichts aber du siehst unten rechts bei den mini icons dass es läuft) ==> du kannst alles bilder die gestreamt werden auf windows anschauen. das musst du nur 1 mal machen.

SKRIPT 1: DRIVING
- type "./autostart/start_driving.sh" in die Konsole
- warte bis du das GUI siehst
- drive

SKRIPT 2: TAG KAMERAS
- type "roslaunch pointgrey_driver tag_detector_2cams.launch" in die Konsole
- warte bis du beide kamera bilder siehst
- ruf die state estimation services ab von grasshoppers

SKRIPT 3: WIRE DETECTION CAMERAS
- type "roslaunch if_repos if_repos.launch" in die Konsole
- schaue im terminal bis er zu beiden kameras connected hat. 
"""""""
Connected to: Point Grey Research Blackfly BFLY-PGE-05S2M 14256302
Connected to: Point Grey Research Blackfly BFLY-PGE-05S2M 14256320
""""""""
- ruf den service ab von grasshoppers
