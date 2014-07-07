from Reaction import Reaction


class ReactionPublishRosOutNode(Reaction):

    """A reaction that is able to publish a message on rosout."""

    def __init__(self):
        super(ReactionPublishRosOutNode, self).__init__()
