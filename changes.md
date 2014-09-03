Change
===

* MyPlugin: added a PluginClass which holds the Widgets tree and selection
* TreeWidget: changed the param type from **..check_box_state_changed from bool to int**
* OverviewPlugin: added missing method **on_range_combo_box_index_changed(index)**

## Configuration files
* Now uses seuid where possible
=> specifications now take **seuid**: {} instead of **type: name**: {} (see arni_processing/resources/testconfig.yaml)
* Now uses list of dictionaries against UserErrors from invalid parameter names

* /arni/aggregation_window (3) Length of time to aggregate topics

## MetadataStorage
* automatic cleanup now depends on the rosparameters /arni/storage/auto_cleanup (True) and /arni/storage/cleanup_timer (30)[seconds]


* AbstractItem needs a new method: get_identifier() --> this is needed in LogFilterProxy to determine the name of this element (at least I think so)
* getInstance no longer needed in ROSModel (add additional classes), the singleton effect is implemented with metaclasses
* *convert some functions from CamelCase to lower_case --> ROSModel.headerData (header_data), ROSModel.columnCount (column_count), ROSModel.rowCount to row_count* --> !!!ERROR!!! These are Qt functions and should not be renamed!!!!
* todo: network diagram is still missing in the design, also in the final versions --> add!
* found solution to method overloading in one case --> 4 new methods __add_rated_statistics_item(..), the old will be removed, are changed to *private* cause only needed internally --> 4 locks are needed, names adapated
* in ROSModel: Thread not longer needed --> Timer replaced its functionality, therefore also start() can deleted, only update_model is stillt needed as callback for the timer
* update_model in ROSModel no longer public
* AbstractItem needed some more methods to match the qt interfaces --> child_count, column_count, row, 
* constructor/ __init__ from ROSModel should only get a parent item, nothing more --> data is no longer a parameter
* new private item in ROSModel: __mapping
* AbstractItem constructor changed to def __init__(self, parent_item=None, identifier, type, can_execute_actions) --> constructor of the subclasses have been changed accordingly
*  **IMPORTANT**: should rethink the naming conventions in the gui part because of qt and python mixing the convetions can be fulfilled on the python/ros side...
* the transform_data method has been deleted, 4 other methods have taken its place **docu still missing**
* the names of the services now are /get_statistic_history and /execute_node_reaction
* name change from add_log_item to add_log_entry
* new class **ErrorDelegate** for painting the log model with color
* get_detailed_data 
* extract the painting part from get_detailed_data to the widgets
* get_erroneous_entries in abstractItem
* overview.ui from log_tab_tree_widget to log_tab_tree_view.

## Big TODOS:
* delete items nolder than currently does **NOT** erase rated data!!! 
* from some data in the overview widget the mean has to be calculated (or e.g. display the data from the last minute or something!!!)
* topic aggregation not yet working!!! --> get the algorithm from matthias and build it in :) --> cannot be applied here
* because of new changes the state is no longer calculated correctly. HAS TO BE FIXED ASAP.
* NEW SEGFAULT AND NOBODY KNOWS WHY..
* also show subscribers
* make the show buttons more intelligent
* check why the translation is not working in the range boxes

reminder: state is no longer returned by the get_items_younger_than --> if any errors occur, look for this


## Final/Small Improvements
* interpolate the data with flowcharts or however the method is called
* hover texts have to be defined
* in the help button the api docs and the general docu (and our names should be shown)
* improve the model
* add licence data to every file
* check host/node etc regularily for timeouts --> get_time_of_last_update() --> im model regelmäßig abfragen und nach 65 sekunden löschen
* shrink the graphics to smaller size
* after a lot of time the programm doesn't terminate any longer
* on enter clicked --> automatically activate apply button
* add an expand_all button
* caching the filter results in the proxies for faster updates :) --> maybe also needed to fix the filter problems!!! (can return same result as long as the filter is not invalidated, but have to invalidate filter on every update)
* model is never closed
* find out why the translation is not used for generating the text of the range box in overview/ selection widget
* test code coverage with tools by trying out anything on the gui
* currently we do not support the delivered_msg part in topicstatistics --> may add support for it


## WiP
* filter is restricted to the upper plains --> if an word is searched and only a node but not its host contains the word <--> search every time through the whole subtreee and if any element returns true return true, for effiency use extensive caching --> caching now implemented, the rest still missing
* [mh]make locking a little better by using different locks for rated and non rated data :) --> show if this works :) <--> still a little work to do
* [mh]/[sk]in the filter field add some default text
* [sk]the flood of the data gets bigger and bigger, getting more and more data per each update - REASON UNKWON --> LOOK FOR BUFFERTHREAD AND THE INCOMING DATA IF THERE IS ALWAYS MORE AND MORE... IF NOT WE GONNA HAVE A PROBLEM
* [mh]race condition when delete_items_older_than and get_items_younger_than are executed paralelly
* [sk]solve funny graphs problem in selectionWidget (probably double plot or something similar)
* [mh]when plotting have to care for multidimensional entries --> simpel solution: don't add them right now --> currently not needed
* [mh]if the gui runs longer sometimes segfaults may appear - might be because of the race conditions that are about to be fixed
* [mh]cpu usage core ist nicht sinnvoll beim plotten --> currently simply ignored, has to be removed <--> ask alex if this makes sens at all --> IS NOT ANSWERING SINCE 3 DAYS --> now answered: can be removed --> remove in the code
* [sk]logging every error message occuring when updating an item <--> DAMAGED STATE CONVERSION BUT SEEMS TO BE WORKING
* [mh]drawing the graphs in selectionwidget --> copy and paste --> many errors still remaining to fix!!! 
* [mh]race conditions in overviewWidget on_graph_window_resized or similar whenever the resize method is called to fast... --> probably fixed, at least in most cases ^^
* [mh]saving the model for only 60 seconds (maybe more, simply try it) --> seems to work --> keep testing this feature!!! <--> race condition up there is happing because of this!!!
* [sk]/[mh]document everything
* [mh]RUNTIME_ERROR BECAUSE ELEMENT HAS BEEN DELETED --> Timer Fehler oder so... Schwer/nicht behebbar
* [mh]why are sometimes empty values given to the axis? --> for no plausible reason
* [sk] / [mh]host_statistics arrives in the gui only at the beginning --> then nothing comes any longer!!!


## Done
* [mh]the units in the graphs are still missing
* [mh]self.__last_update no longer needed
* [mh]CPU_TEMP_CORE IS UNNECESSARY AND SHOULD BE REMOVED BEFORE PUBLISHING!!!
* [sk] / [mh]host recognition for a host on the same pc  --> seems to work but only in "real" networks, obviously a Configuration problem <--> when restarting most times works again
* [mh]Fixed a bug that when the erroneous checkbox was on, the filter could not be applied correcty without removing the "errouneous" filter
* [sk] / [mh]after some time the overview widget does not show any data any more...
* [sk] / [mh]remove all the todos and code comments everywhere
* [mh]the storage of the data in node_item is somehow erroneous --> Fix!
* [mh]round the shown data in the gui --> round(number, 2) --> started but not finished yet
* [mh]replace "something is wrong" with good text!
* [mh]update the texts that are shown  --- the units in the overview are missing (maybe also some color if someone is too high --> not our )
* [mh]give the translatable items to somebody so that he can translate these items
* [mh]enter tr() everywhere for internationalisation --> still missing: the widget itself and the actual_values etc --> probably finished
* [mh]get overview widget to work --> still the bug with host items remains, don't know why
* [mh]SHORT_DATA CANNOT BE RECEIVED VIA GET_LATEST_DATA --> FIX --> works at many positions
* [mh]update get_erroneous_entries for supporting the new format of the rated data
* [mh]translation root_item
* [mh]disable action when not a nodeitem is selected --> check
* [mh]FINALLY FIX SEGMENTATION FAULT!!! - well at least it doesn't segfault any more^^
* [mh]adapt package.xmls for including pyqtgraph
* [mh]create to constants for minimum time and maximum number of elements in the model
* [mh]protect pyqtgraph imports with try/except
* [mh]locking the graph view to the views of the other so that the person sees the same range in every view --> done
* [mh]added stop/continue button to graphs so you have time to explore the data
* [sk]SelectionWidget: define services and check if working
* [mh]on close of one widget in detail gui, close the other! --> not possible
* [mh]lock updateGraphs --> non reentrant functions, otherwise segmentation faults occur!
* [mh]locking the updateModel from modifying the data while plotting the data in the gui --> shape errors
* [mh]unknown state errors
* [mh]make sure there are no more dict["time"] calls, they are invalid
* [mh]reformat the ROSModel to use the seuid helper class --> Helper class returns None values --> no longer using!!! --> wrongly used, maybe try another time!
* [sk]execute_action not implemented
* [mh]why are there runtimeerrors when closing the gui. and why are there these threading errors(might it be the timers?)
* [mh]GUI WANTS TO RECEIVE ITEMS THE TOPIC DOES NOT PROVIDE!!!
* [mh]fixed index errors when element was None
* [mh]reason for wrong return values: *args is never None, so if args is not None will always enter even though args might be empty!!!
* [mh]moved logging to own class

* [mh]changing the background of the updateGraphs
* [mh]change plotting to use one range_box or maybe two or three if there is enough space :) <--> make this dependent from the available space
* [sk]scrolling does not work because of the permanent updates...--> solution: if no data has changed, simply get_detailed_data simply returns None --> or not calling the update so often, every minute should suffice --> found better solution by simply asking the scrollbar
* [mh]fixed never ending recursion


=======
* selection.ui from log_tab_tree_widget to log_tab_tree_view.
* arni_gui_detail: moved __on_item_in_item_tree_view_double_clicked(self, item): from tree_widget to arni_gui_detail to handle the content change of the selection_widget
* indexerror in ros_model...