Change
===

* MyPlugin: added a PluginClass which holds the Widgets tree and selection
* TreeWidget: changed the param type from **..check_box_state_changed from bool to int**
* OverviewPlugin: added missing method **on_range_combo_box_index_changed(index)**

## Configuration files
* Now uses seuid where possible
=> specifications now take **seuid**: {} instead of **type: name**: {} (see arni_processing/resources/testconfig.yaml)

* AbstractItem needs a new method: get_identifier() --> this is needed in LogFilterProxy to determine the name of this element (at least I think so)
* getInstance no longer needed in ROSModel (add additional classes), the singleton effect is implemented with metaclasses
* *convert some functions from CamelCase to lower_case --> ROSModel.headerData (header_data), ROSModel.columnCount (column_count), ROSModel.rowCount to row_count* --> !!!ERROR!!! These are Qt functions and should not be renamed!!!!
* the transform_data(...) method in ROSModel have to be reduced to a single method cause python doesn't support function overloading :(
* same problem with __add_buffer_item() --> todo:find a good solution
* todo: network diagram is still missing in the design, also in the final versions --> add!
* found solution to method overloading in one case --> 4 new methods __add_rated_statistics_item(..), the old will be removed, are changed to *private* cause only needed internally --> 4 locks are needed, names adapated
* in ROSModel: Thread not longer needed --> Timer replaced its functionality, therefore also start() can deleted, only update_model is stillt needed as callback for the timer
* update_model in ROSModel no longer public
* AbstractItem needed some more methods to match the qt interfaces --> child_count, column_count, row, 
* execute_action no longer abstract!
* removed list from constructor of AbstractItem
* constructor/ __init__ from ROSModel should only get a parent item, nothing more --> data is no longer a parameter
* new private item in ROSModel: __mapping
* AbstractItem constructor changed to def __init__(self, parent_item=None, identifier, type, can_execute_actions)
*  **IMPORTANT**: should rethink the naming conventions in the gui part because of qt and python mixing the convetions can be fulfilled on the python/ros side...
* the transform_data method has been deleted, 4 other methods have taken its place **docu still missing**

* the names of the services now are /get_statistic_history and /execute_node_reaction. Network diagram etc have to be adapted.