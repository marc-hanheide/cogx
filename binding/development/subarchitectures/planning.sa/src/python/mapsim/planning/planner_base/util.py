def any(S):
    for x in S:
        if x:
            return True
    return False

def all(S):
    for x in S:
        if not x:
            return False
    return True

def first_index(S):
    for i, x in enumerate(S):
        if x:
            return i
    return -1

def is_subdict_of(sub, sup):
    for key in sub:
        if not sup.has_key(key):
            return False
        if sub[key] != sup[key]:
            return False
    return True

def sign(x):
    if x > 0:
        return 1
    elif x == 0:
        return 0
    return -1

def is_string(val):
    try:
        val + " "
    except:
        return False
    return True

def is_seq(val):
    try:
        a = val[0]
    except:
        return False
    return True

def strlists2columns(lists, sep=3, width=None):
    if not width:
        width = sep
        for l in lists:
            for r in l:
                width = max(width, len(r)+sep)
    newlists = [[s.ljust(width) for s in l] for l in lists]
    transp = zip(*newlists)
    s = "\n".join(["".join(l) for l in transp])
    return s
    
def translate2python_obj(val):
    special_values = dict(true=True, false=False)
    if is_string(val):
        val = val.lower()
    if is_string(val) and val.isdigit():
        val = int(val)
    if val in special_values:
        val = special_values[val]
    return val

class MyConfiguration(object):
    def __init__(self, filename=None):
        self.conf_str = None   # a function providing a unique description of the configuration
        self.test_setting = None
        if filename:
            self.load(filename)
    def load(self, filename):
        self.__dict__.clear()
        for l in open(filename):
            l = l.strip()
            if not l or l.startswith('#'):
                continue
            var, val = l.split()
            self.set_config(var, val)
    def has_config(self, var):
        return self.__dict__.has_key(var)
    def get(self, var):
        return self.__dict__[var]
    def set_config(self, var, val):
        if is_string(val):
            val = val.lower()
        val = translate2python_obj(val)
        self.__dict__[var] = val
    def copy(self):
        n = MyConfiguration()
        n.conf_str = self.conf_str
        n.__dict__.update(self.__dict__)
        return n


class ListCursor:
    def __init__(self, l, ignore_blank_lines=True):
        if ignore_blank_lines:
            self.l = [line for line in l if line]
        else:
            self.l = l
        self.index = 0
    def next(self, n):
        nindex = self.index+n
        nl = self.l[self.index:nindex]
        self.index = nindex
        return nl
    def nextline(self):
        return self.next(1)[0]
    def has_next(self):
        return self.index < len(self.l)

def print_file(filename):
    for line in open(filename):
        print line,
            
if __name__ == "__main__":
    l = (1,2,3,4,5,6)
    print first_index(x == 3*3 for x in l)
            
