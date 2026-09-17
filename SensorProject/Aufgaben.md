# Übung 1

# Aufgabe 0

- Folge dem Tutorial unter [https://gitlab.rhrk.uni-kl.de/aor-hiwis/finroc/finroc] um finroc zu installieren 

- Compile und starte das Sensor Projekt
    - Navigiere in das finroc Verzeichnis udn führe `source scripts/setenv -p SensorProject` aus, um die Umgebungsvariablen zu setzen.
    - Navigiere in das SensorProject Verzeichnis und führe `./link_to_finroc.sh` aus, um das Projekt zu linken.
    - Navigiere in das finroc Verzeichnis und führe `make SensorProject-bin` aus, um das Projekt zu kompilieren und anschließend `SensorProject` um das Projekt zu starten.

# Aufgabe 1
- Verifiziere in finstruct, dass die Sensoren out_a1, out_a2, out_a3, out_b1, out_b2 existieren und oszillierende Werte ausgeben.
- Visualisiere die Werte der Sensoren out_a1, out_a2, out_a3, out_b1, out_b2 in fingui mit dem Oszilloscope Widget.
- Welche Frequenzen haben die Signale ungefähr? 

# Aufgabe 2
- Schreibe ein Modul, dass die Frequenz eines signals bestimmt und über einen Port ausgibt. 
- Das Modul soll einen Input Port für das Signal und einen Output Port für die Frequenz haben.
- Teste das Modul, indem du die Frequenz der Sensoren aus Aufgabe 1 bestimmst 
