"""
!IMPORTANT!
For executing this the enum class is needed.
Install via shell with "pip install enum34".
"""

try:
    from enum import Enum
except ImportError as e:
    print("An error occured trying to import enum from Enum. Please check your Python version (>=3.4) or make "
          "sure you have installed the enum classes via \"pip install enum34\".")
    raise


class RemoteAction(Enum):
    """
    Gives a predefinition for a remote interaction with hosts and nodes.
    """
    E_ACTION_STOP = 0
    E_ACTION_RESTART = 1