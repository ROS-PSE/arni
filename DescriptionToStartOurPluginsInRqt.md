##Description to start our Plugins in rqt
* goto catkin_ws:
`cd catkin_ws`

* copy our packages into the workspace (in the moment the packages rqt_overview and rqt_tree)


* "compile" it: 
`catkin_make`

* start the roscore: 
`roscore`

* now start in a new terminal rqt:
`rqt`

* if the 2 plugins don't appear in the *Introspection* section, close rqt and delete the rqt-config-file, now restart rqt: `rm ~/.config/ros.org/rqt_gui.ini`

* **IT'S NOW POSSIBLE TO START OUR PLUGINS VIA :**`rosrun arni_rqt_overview_plugin arni_rqt_overview_plugin` **AND** `rosrun arni_rqt_detail_plugin arni_rqt_detail_plugin`
