# Übung 2

# Aufgabe 0

- Compile und starte das PointcloudProject
    - Navigiere in das finroc Verzeichnis und führe `source scripts/setenv -p PointcloudProject` aus, um die Umgebungsvariablen zu setzen.
    - Navigiere in das PointcloudProject Verzeichnis und führe `./link_to_finroc.sh` aus, um das Projekt zu linken.
    - Navigiere in das finroc Verzeichnis und führe `make PointcloudProject-bin` aus, um das Projekt zu kompilieren und anschließend `PointcloudProject` um das Projekt zu starten.



# Aufgabe 1
- Create a height map from the pointcloud using the tGridAspectMap class. You can use the GridmapPainter class to visualize the map.

```cpp
// Beispielcode zur Visualisierung der Höhenkarte
auto canvas = vis_height_map_2D.GetUnusedBuffer(); // tVisualizationOutput<rrlib::canvas::tCanvas2D, tLevelOfDetail::ALL> vis_height_map_2D;
canvas->Clear();
GridmapPainter<2> painter;
painter.color_range(
  rrlib::coviroa::color_spaces::tRGB(52.0f, 152.0f, 235.0f),
  rrlib::coviroa::color_spaces::cRGB_BLACK,
  10.0f,
  11.0f);
painter.value_coloring(0.0f, rrlib::coviroa::color_spaces::cRGB_WHITE);
painter.draw(*canvas, height_map);
vis_height_map_2D.Publish(canvas);
```

# Aufgabe 2
- The `mSimulation` module provides a pointcloud, the position of the triangle, and the position of the goal as outputs. Use the `in_velocity` and `in_steering` inputs to move the triangle towards the goal while avoiding collisions with the trees.
