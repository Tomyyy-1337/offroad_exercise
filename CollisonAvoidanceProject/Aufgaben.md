# Übung 2

# Aufgabe 0

- Compile und starte das CollisonAvoidanceProject
    - Navigiere in das finroc Verzeichnis udn führe `source scripts/setenv -p CollisonAvoidanceProject` aus, um die Umgebungsvariablen zu setzen.
    - Navigiere in das CollisonAvoidanceProject Verzeichnis und führe `./link_to_finroc.sh` aus, um das Projekt zu linken.
    - Navigiere in das finroc Verzeichnis und führe `make CollisonAvoidanceProject-bin` aus, um das Projekt zu kompilieren und anschließend `CollisonAvoidanceProject` um das Projekt zu starten.

# Aufgabe 1

- Schaue dir den Visualization Port des Moduls mSimulation in fingui an.
- Erstelle in finstruct ein Joystick-Widget und verbinde es mit den Input Ports des Moduls mSimulation. Das Dreieck sollte sich nun bewegen lassen. Ziel ist es das Dreieck zum gelben Kreis zu bewegen, ohne dass es mit den roten Kreisen kollidiert.
- Schreibe Module um das Dreieck zum Ziel zu steuern. Die Simulation stellt die Distanz zum Ziel, den Winkel zum Ziel und die Distanz zu Hindernissen als output ports bereit. Nutze diese Werte um Hindernisse zu vermeiden und das Ziel zu erreichen.