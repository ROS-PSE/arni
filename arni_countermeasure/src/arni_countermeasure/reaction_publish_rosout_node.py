from reaction import *


class ReactionPublishRosOutNode(Reaction):

    """A reaction that is able to publish a message on rosout."""

    def __init__(self, message):
        super(ReactionPublishRosOutNode, self).__init__()

        #: message to publis.
        self.message = message
