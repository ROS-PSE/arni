Changes
===

## Processing
### Configuration files
* Now uses seuid where possible
=> specifications now take **seuid**: {} instead of **type: name**: {} (see arni_processing/resources/testconfig.yaml)
* Now uses list of dictionaries against UserErrors from invalid parameter names

### Processing Data
* Added parameter `~aggregation_window (3)[seconds]` as length of time to aggregate topics,
* Created a new method to compare especially topics.
* Packages are rated as "alive" based on the parameters `~alive_interval` and `~alive_timer`.
* Data fields are not compared on the intended level thus leaving the Metadata class unnecessary.

### MetadataStorage
* automatic cleanup now depends on the rosparameters `~storage/auto_cleanup (True)`, `~storage/cleanup_timer (30)[seconds]` and `~storage/timeout (300)[seconds]`

* For more information on the new parameters see [The ROS wiki](http://wiki.ros.org/arni_processing#Used_Parameters).


## Arni GUI

* ***arni_gui_detail:*** added a Plugin-Class which holds the Widgets tree and selection
* moved **__on_item_in_item_tree_view_double_clicked(self, item)** from tree_widget to arni_gui_detail to handle the content change of the selection_widget
* ***tree_widget:*** changed the param type from **..check_box_state_changed** from bool to int
* added method **get_relative_font_size(self)**
* added method **set_relative_font_size(self, relative_font_size)**
* ***selection_widget:*** added method **create_graphs(self)**
* added method **get_current_tab(self)**
* added method **set_current_tab(self, index=0)**
* added method **get_range_combo_box_index(self)**
* added method **set_range_combo_box_index(self, index=0)**
* ***arni_gui_overview:*** added a Plugin-class which holds the Widget overview
* ***overview_widget:*** added method **create_graphs(self)**
* added method **get_current_tab(self)**
* added method **set_current_tab(self, index=0)**
* added method **get_range_combo_box_index(self)**
* added method **set_range_combo_box_index(self, index=0)**
* ***arni_gui:*** added class **connection_item_sub.py**
* added class **log_delegate.py**
* added class **lru_cache.py**
* added class **model_logger.py**
* added class **root_item.py**
* added class **topic_item_sub.py**
* ***ros_model:*** added method **get_overview_data_since(self, time=None)**
* changed **transform_data(..)** to **transform_(rated|topic|node|host)_statistics_item(self, item)**
* added method **get_item_by_seuid(self, seuid)**
* added method **get_log_model(self)**
* added method **get_logger(self):**
* added method **get_overview_text(self):**
* added method **get_root_item(self):**
* added method **get_translator(self):**
* removed method **get_instance(self)**
* removed method **add_log_item(self, list)**
* => added inner class **QAbstractItemModelSingleton(Singleton, type(QAbstractItemModel))**
* ***abstract_item:*** added method **get_type(self)**
* added method **get_seuid(self)**
* added method **get_seuid(self)**
* added method **set_state(self, state)**
* added method **get_state(self)**
* added method **_update_current_state(self)**
* added method **update_rated_data(self, data)**
* added method **child_count(self)**
* added method **column_count(self)**
* added method **row(self)**
* added method **get_amount_of_entries(self)**
* added method **get_rated_items_older_than(self, time)**
* added method **delete_rated_items_older_than(self, time)**
* added method **get_rated_items_younger_than(self, time, *args)**
* added method **get_detailed_data(self)**
* added method **get_plotable_items(self)**
* added method **get_erroneous_entries(self)**
* added method **get_erroneous_entries_for_log(self)**
* added method **can_execute_actions(self)**
* added method **get_short_data(self)**
* ***item_filter_proxy:*** added method **invalidateFilter(self)**
* added method **setFilterRegExp(self, string)**
* added method **show_subscribers(self, show_subscribers)**
* added method **set_filter_string(self, filter_string)**
* ***log_filter_proxy:*** added method **invalidateFilter(self)**
* added method **setFilterRegExp(self, string)**
* ***size_delegate:*** added method **initStyleOption(self, option, index)**
* removed method **paint(self, painter, option, index)**
* removed method **set_(bigger|smaller)_font_size(self)**

* the names of the services now are /get_statistic_history and /execute_node_reaction
 
## Final/Small Improvements --> maybe for after PSE...
* check why the translation is not working in the range boxes
* print the last update somewhere...
* interpolate the data with flowcharts or however the method is called
* hover texts have to be defined
* in the help button the api docs and the general docu (and our names should be shown)
* check host/node etc regularily for timeouts --> get_time_of_last_update() --> im model regelmäßig abfragen und nach 65 sekunden löschen
* find out why the translation is not used for generating the text of the range box in overview/ selection widget
* [mh]when plotting have to care for multidimensional entries --> simpel solution: don't add them right now --> currently not needed
* add state unkown-alive / unkown-dead etc.

## WiP
* [mh]topic aggregation not yet working!!! --> get the algorithm from matthias and build it in :) --> cannot be applied here --> check if everything is working
* [mh]make locking a little better by using different locks for rated and non rated data :) --> show if this works :) <--> still a little work to do
* [mh]race conditions in overviewWidget on_graph_window_resized or similar whenever the resize method is called to fast... --> probably fixed, at least in most cases ^^
* [sk]/[mh]document everything
* [mh]RUNTIME_ERROR BECAUSE ELEMENT HAS BEEN DELETED --> Timer Fehler oder so... Schwer/nicht behebbar
* [mh]why are sometimes empty values given to the axis? --> for no plausible reason

## Done
* [?]from some data in the overview widget the mean has to be calculated (or e.g. display the data from the last minute or something!!!) --> important
* [sk]also show subscribers
* [?]the translation has to be done again for adapting to the newly added items and texts
* [mh]fixed division error
* [sk]make the show buttons more intelligent
* [mh]currently we do not support the delivered_msg part in topicstatistics --> may add support for it
* [mh]test code coverage with tools by trying out anything on the gui
* [mh]caching the filter results in the proxies for faster updates :) --> maybe also needed to fix the filter problems!!! (can return same result as long as the filter is not invalidated, but have to invalidate filter on every update)
* [mh]shrink the graphics to smaller size
* [mh]after a lot of time the programm doesn't terminate any longer
* [mh]on enter clicked --> automatically activate apply button
* [mh]add an expand_all button
* [sk]delete items nolder than currently does **NOT** erase rated data!!! 
* [mh]NEW SEGFAULT AND NOBODY KNOWS WHY.. --> still don't really know the reason, but it doesn't appear any longer
* [mh]race condition when delete_items_older_than and get_items_younger_than are executed paralelly
* [mh]cpu usage core ist nicht sinnvoll beim plotten --> currently simply ignored, has to be removed <--> ask alex if this makes sens at all --> IS NOT ANSWERING SINCE 3 DAYS --> now answered: can be removed --> remove in the code --> removed
* [sk]logging every error message occuring when updating an item <--> DAMAGED STATE CONVERSION BUT SEEMS TO BE WORKING
* [mh]/[sk]in the filter field add some default text
* [mh]filter is restricted to the upper plains --> if an word is searched and only a node but not its host contains the word <--> search every time through the whole subtreee and if any element returns true return true, for effiency use extensive caching --> caching now implemented, the rest still missing --> now working
* [mh] / [sk]solve funny graphs problem in selectionWidget (probably double plot or something similar)
* [sk]the flood of the data gets bigger and bigger, getting more and more data per each update - REASON UNKWON --> LOOK FOR BUFFERTHREAD AND THE INCOMING DATA IF THERE IS ALWAYS MORE AND MORE... IF NOT WE GONNA HAVE A PROBLEM
* [mh]if the gui runs longer sometimes segfaults may appear - might be because of the race conditions that are about to be fixed
* [mh]drawing the graphs in selectionwidget --> copy and paste --> many errors still remaining to fix!!! 
* [mh]saving the model for only 60 seconds (maybe more, simply try it) --> seems to work --> keep testing this feature!!! <--> race condition up there is happing because of this!!!
* [sk] / [mh]host_statistics arrives in the gui only at the beginning --> then nothing comes any longer!!!
* [mh]/[sk]because of new changes the state is no longer calculated correctly. HAS TO BE FIXED ASAP.
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
* [mh]update the texts that are shown  --- the units in the overview are missing (maybe also some color if someone is too high)
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
* [mh]fixed segfault that occured because of using internalPointer() instead of data()
* [mh]fixed segfault that occured because of missing input sanitization (inputed dict instead of str)
* [mh]fixed bad singleton error in ROSModel --> normal singleton implementation didn't work because of the missing superclass object in PyQt/PySide <--> Fix: multiple inheritance for also using QObject
* [mh]extreme performance problem when plotting --> used the wrong method plot instead of setData, furthermore to clear graphs use clear(), a non api-documented method
