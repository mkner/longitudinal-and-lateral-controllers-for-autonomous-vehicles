#

#mk
# v0.01

class Localvars(object):
    
    def __init__(self):
        pass
    # creates a variable in class (object) local symbol table
    def create(self, var_name, value):
        if not var_name in self.__dict__:
            self.__dict__[var_name] = value

