# indent with 4 spaces (most editors can be set to
# put 4 spaces when hitting tab)
# if something is unclear, see
# http://wiki.ros.org/PyStyleGuide
# and
# http://legacy.python.org/dev/peps/pep-0008
# its recommended to check if the code follows the
# pep8 formatting with an linter.

# see http://legacy.python.org/dev/peps/pep-0257/
# for questions on documentation

# file has the same name as the class + .py
# each class has its own file.

#######
# NOTE:
# THIS CLASS IS COMPLETELY MISSING THE DOCUMENTATION
# GUIDLINE, TO BE ADDED SOON
#######


class SomeClass(object):
    """
    This is the description for the class

    Attributes: #only public attributes
        * size(int): the size
    """
    def __init__(self):
        """
        init method
        """
        super(SomeClass, self).__init__()

        # a public attribute
        self.size = 100

        # called non public in python
        # is equivalent to private in other languages
        self.__password = "im private"

        # _ as suffix stands for a protected attribute
        self._username = "im protected"

        # public attribute another_attribute
        # with the python version of getters and
        # setters.
        # acess is mySomeClass.width = 20
        # note that the attribute definition is with an
        # underscore.
        # only use this kind of defining the attribute
        # if the property or setter method really does
        # something.
        self._width = 20

    @property
    def another_attribute(self):
        return self._another_attribute

    @another_attribute.setter
    def another_attribute(self, value):
        """
        example method with random text :)

        :param value: The data to be published
        :type value: RatedStatistics
        :returns: MetadataStorageResponse
        :raise ValueError: If the key is null or empty.
        """
        if value > 40:
            value = 40
        self._another_attribute = value

    # if the parameters are to long put them in new lines
    # after the opening bracket. indent once.
    def long_method_name(
            self, var1, var2, var3,
            var4, some_really_long_parameter):
        pass

    def _im_a_protected_method(self):
        pass

    def __im_a_private_method(self, arg1, arg2):
        pass

    @staticmethod
    def i_am_a_function():
        pass
