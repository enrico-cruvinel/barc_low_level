from dataclasses import dataclass, field
import copy

@dataclass
class PythonMsg:
    def __setattr__(self,key,value):
        '''
        Overloads default atribute-setting functionality to avoid creating new fields that don't already exist
        This exists to avoid hard-to-debug errors from accidentally adding new fields instead of modifying existing ones
        
        To avoid this, use:
        object.__setattr__(instance, key, value)
        ONLY when absolutely necessary.
        '''
        if not hasattr(self,key):
            raise TypeError ('Cannot add new field "%s" to frozen class %s' %(key,self))
        else:
            object.__setattr__(self,key,value)

    
    def print(self, depth = 0, name = None):
        '''
        default __str__ method is not easy to read, especially for nested classes.
        This is easier to read but much longer
        
        Will not work with "from_str" method.
        '''
        print_str = ''
        for j in range(depth): print_str += '  '
        if name:
            print_str += name + ' (' + type(self).__name__ + '):\n'
        else:
            print_str += type(self).__name__ + ':\n'
        for key in vars(self):
            val = self.__getattribute__(key)
            if isinstance(val, PythonMsg):
                print_str += val.print(depth = depth + 1, name = key)
            else:
                for j in range(depth + 1): print_str += '  '
                print_str += str(key) + '=' + str(val)
                print_str += '\n'
        
        if depth == 0:
            print(print_str)
        else:
            return print_str
    
    
    def from_str(self,string_rep):
        '''
        inverts dataclass.__str__() method generated for this class so you can unpack objects sent via text (e.g. through multiprocessing.Queue)
        '''
        val_str_index = 0
        for key in vars(self):
            val_str_index = string_rep.find(key + '=', val_str_index) + len(key) + 1  #add 1 for the '=' sign
            value_substr  = string_rep[val_str_index : string_rep.find(',', val_str_index)]   #(thomasfork) - this should work as long as there are no string entries with commas

            if '\'' in value_substr:  # strings are put in quotes
                self.__setattr__(key, value_substr[1:-1])
            if 'None' in value_substr:
                self.__setattr__(key, None)
            else:
                self.__setattr__(key, float(value_substr))
                   
    def copy(self):
        return copy.deepcopy(self)

@dataclass
class VehicleState(PythonMsg):

    x :float = field(default = 0) # vehicle position (m)
    v :float = field(default = 0) # vehicle speed (m/s)
    a :float = field(default = 0) # vehicle acceleration (m/s^2)
    u :float = field(default = 0) # pwm input
    t :float = field(default = 0) # time (s)

@dataclass
class VehicleConfig(PythonMsg):

    delay      :float = field(default = 0)
    offset     :float = field(default = 1500)
    gain       :float = field(default = 0.002)
    sat_poly_3 :float = field(default = 0.1)
    sat_poly_5 :float = field(default = 0.1)
    roll_res   :float = field(default = 0.1)
    drag       :float = field(default = 0)
    damping    :float = field(default = 0)
    
    u_min      :float = field(default = 1000)
    u_max      :float = field(default = 2000)

    dt         :float = field(default = 0.01)