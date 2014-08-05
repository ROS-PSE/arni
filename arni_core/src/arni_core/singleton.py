# class singleton:

#     """A decorator that makes an class a singleton."""

#     def __init__(self, cls):
#         self.cls = cls
#         self.instancex = None

#     def __call__(self, *args):
#         if self.instancex is None:
#             self.instancex = self.cls(*args)
#         return self.instancex


class Singleton(type):
    __instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls.__instances:
            cls.__instances[cls] = super(
                Singleton, cls).__call__(*args, **kwargs)
        return cls.__instances[cls]
