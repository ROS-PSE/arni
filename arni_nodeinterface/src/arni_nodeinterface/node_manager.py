import subprocess
import rosnode
import roslaunch

class NodeManager(object):
    """ 
    Can restart or stop nodes or execute a countermeasure. 
    """
    
    def __init__(self):
    
        super(NodeManager, self).__init__()
        
        
        
    def stop_node(self, node_id):
        """
        Stops the node with the given id.
        Returns a message about operation's success.
        
        :param node_id: id of the node to be stopped.
        :type node_id: String.
        :returns: String
        """
        success, fail = rosnode.kill_nodes(['node_id'])
        if node_id in success:
            return 'Node successfully stopped'
        else:
            return 'Node could not be stopped'
            
        
    def restart_node(self, node_id):
        """
        Restarts a node with the given id.
        Returns a message about operation's success.
        
        :param node_id: id of the node to be restarted.
        :type node_id: String.
        :returns: String
        """
        self.stop_node(node_id)

        #roslaunch.nodeprocess.create_node_process()
        
    def execute_command(self, args):
        """
        Executes a system call with the given arguments.
        Returns a message about operation's success.
        
        :param args: Arguments for the system call
        :type args: String
        :returns: String
        """
        proc = subprocess.Popen(args, stdout = subprocess.PIPE)
        
        print proc.communicate()
