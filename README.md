ARNI
=========
This is the codebase of the Advanced ROS Network Introspection.

*Please note that this is still a development version. If you find any bugs please report or open pull requests. We will be fixing them as soon as our time permits it.*

##What is arni?

Advanced ROS Network Introspection (ARNI) extends the /statistics features introduced with Indigo and completes the collected data with measurements about the hosts and nodes participating in the network. These are gathered from an extra node that has to run on each host machine. All statistics or metadata can be compared against a set of reference values using the monitoring_node. The rated statistics allow to run optional countermeasures when a deviation from the reference is detected, in order to remedy the fault or at least bring the system in a safe state.
Further information can be found at: http://wiki.ros.org/arni
Also there are a few to tutorials which cover most basics: http://wiki.ros.org/arni/Tutorials


## Note for ROS kinetic:
pyqtgraph has to be installed manually for this package to work correctly (don't ask me why):
```bash
git clone git@github.com:pyqtgraph/pyqtgraph.git
cd pyqtgraph; sudo python setup.py install
```


===

### Guidelines for developers:
- everything in this repository should be in english.
- every commit should be commented in english.
- python code has to follow the  [ROS](http://wiki.ros.org/PyStyleGuide) and the [pep8](http://legacy.python.org/dev/peps/pep-0008/)
style guide (please **actually** read them before coding)
- directories begin with a lower case character
- see code-examples for some examples on code guidelines
