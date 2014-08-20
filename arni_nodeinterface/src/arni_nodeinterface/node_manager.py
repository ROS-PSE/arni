from node_statistics_handler import NodeStatisticsHandler
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
            
        
    def restart_node(self, node):
        """
        Restarts a node with the given id.
        Returns a message about operation's success.
        
        :param node_id: id of the node to be restarted.
        :type node_id: NodeStatisticsHandler.
        :returns: String
        """
        pipe = subprocess('ps -p ' + str(node.get_pid()) + '-o cmd',
                        shell = True, stdout = subprocess.PIPE).stdout
        out = pipe.read()

        path = out.split('/',6)
        package = path[5]
        exe = path[6].split(' ')[0]
        arguments = path[6].split(' ',1)[1]

        new_node = roslaunch.core.Node(package, exe, args = arguments)
        
        try:
            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()
            node_proc = launch.launch(new_node)
        except RLException:
            return 'failed to launch new Node'



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
        proc.poll()
        return proc.returncode
