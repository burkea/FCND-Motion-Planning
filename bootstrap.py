
class BootStrap:
    def __init__(self):
        self.classes = {}

    def add(self, name, class_name):
        self.classes[name] = class_name

    def create(self, name):
        import importlib
        module_name, class_name = self.classes[name].rsplit(".", 1)
        myclass = getattr(importlib.import_module(module_name), class_name)
        return myclass