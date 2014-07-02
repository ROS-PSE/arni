##Anleitung rqt-Plugins einbinden
* Gehe in den catkin-workspace:
`cd catkin_ws`

* Erstelle die Packages für die Plugins: 
`catkin_create_pkg rqt_overview`, `catkin_create_pkg rqt_tree`

* Gehe zurück in catkin_ws

* Übersetze es: 
`catkin_make`

* Starte den roscore: 
`roscore`

* Nun starte die rqt-gui:
`rqt`

* Sollten die 2 Plugins nun nicht unter *Introspection* auftauchen, schließe rqt wieder und lösche die rqt-config-Datei und starte es erneut: `rm ~/.config/ros.org/rqt_gui.ini`

* Theoretisch sollten die Plugins jetzt erscheinen ;)* 