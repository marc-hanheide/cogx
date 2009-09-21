from collections import defaultdict

import task

# Question: for which variants (StateVariable,
# InstantiatedStateVariable, Fact) do we need to store "believers"?
# Working assumptions:  
#
# StateVariable: no need - this is a general
# description which can always be used with arbitrary believers 
#
# InstantiatedStateVariable: makes sense - the believers are
# specific (grounded) arguments that we want to keep separate
#
# Fact: no need - this is essentially just the combination of an
# InstantiatedStateVariable with a specific value. All info about
# the believers is in the InstantiatedStateVariable


class StateVariable(object):
    """

    """

    def __init__(self, name, parameters, value_type):
        """
        
        Arguments:
        - `name`:
        - `parameters`:
        - `type`:
        """
        self._name = name
        self._parameters = parameters
        self._value_type = value_type
        
    def is_equality_constraint(self):
        return self._name == "="



class InstantiatedStateVariable(object):
    """

    """

    def __init__(self, state_variable, arguments, believers, modality=""):
        self._state_variable = state_variable
        self._arguments = arguments
        self._believers = believers
        self._modality = modality
        if believers and not modality:
            self._modality = "K"

    def get_believers(self):
        return self._believers

    def as_mapl_str(self, parens=True, include_believers=True):
        elmts = [self._state_variable._name] + self._arguments
        mapl_str = " ".join(elmts)
        if include_believers:
            believers = self.get_believers()
            if believers:
                believers_str = " ".join(str(believer) for believer in believers)
                mapl_str = "%s %s (%s)" % (self.modality, believers_str, mapl_str)
        if parens:
            mapl_str = "(%s)" % mapl_str
        return mapl_str
        
    def is_equality_constraint(self):
        return self._state_variable.is_equality_constraint()


class Fact(object):
    """

    """

    def __init__(self, instantiated_state_var, value):
        """
        
        Arguments:
        - `instantiated_state_var`:
        - `value`:
        """
        assert isinstance(instantiated_state_var, InstantiatedStateVariable) 
        self._instantiated_state_var = instantiated_state_var
        if isinstance(value, tuple):
            assert len(value) == 1, "No support for value tuples in state variables any more! Sorry!"
            value = value[0]
        self._value = value

    def copy(self):
        return Fact(self._instantiated_state_var, self._value)  # when is a deep copy needed?
    
    def get_believers(self):
        return self._instantiated_state_var.get_believers()
        
    def as_tuple(self):
        return (self._instantiated_state_var, self.value)

    def as_mapl_str(self, parens=True):
        svar_str = self._instantiated_state_var.as_mapl_str(parens=False, include_believers=False)
        val_str = str(self._value)
        believers = self.get_believers()
        if believers:
            believers_str = " ".join(str(believer) for believer in believers)
            mapl_str = "B (%s) (%s)" % (believers_str, svar_str)
        else:
            mapl_str = "%s : %s" % (svar_str, val_str)
        if parens:
            mapl_str = "(%s)" % mapl_str
        return mapl_str

    def as_pddl_str(self, parens=True, use_direct_k=False, drop_boolean_values=False):
        svar_name = self._instantiated_state_var._state_variable._name
        args_str = " ".join(self._instantiated_state_var._arguments)
        val_str = str(self._value)
        modality = self._instantiated_state_var._modality
        believers = self.get_believers()
        if believers:
            if use_direct_k and modality == "kval":
                modality = "kvald"
            believers_str = " ".join(str(believer) for believer in believers)
            pddl_str = "%s__%s %s %s %s" % (modality, svar_name, believers_str, args_str, val_str)
        else:
            pddl_str = "%s %s %s" % (svar_name, args_str, val_str)
        if parens:
            pddl_str = "(%s)" % pddl_str
        return pddl_str
        

    @classmethod
    def ensure_fact(cls, fact_like):
        """fact_like is either a Fact already or a (key,value)
        tuple. Convert into a Fact if necessary."""
        if isinstance(fact_like, Fact):
            return fact_like
        fact = Fact(*fact_like)  # whether this conversion is possible is checked in Fact.__init__()
        return fact

    @classmethod
    def ensure_tuple(cls, fact_like):
        """fact_like is either a Fact already or a (key,value)
        tuple. Convert into a tuple if necessary."""
        if isinstance(fact_like, Fact):
            return fact_like._instantiated_state_var, fact_like._value
        assert len(fact_like) == 2 and isinstance(fact_like[0], InstantiatedStateVariable)
        key, value = fact_like
        return key, value

    def is_equality_constraint(self):
        return self._instantiated_state_var.is_equality_constraint()

        
class State(object):
    """

    """

    def __init__(self, atask=None, facts=[]):
        """
        
        Arguments:
        - `atask`: which planning task does this state belong to?
        - `facts`: initial facts
        """
        assert atask is None or isinstance(atask, task.Task)
        self._task = atask
        self._facts = {}  # what is the best data structure for efficient storage and retrieval?
        for fact in facts:
            self.add_fact(fact)
        # the following "views" should be computed initally instead of
        # stored. More costly but more open to changes while still in flux
#         self._dict = {}  # as key/value pairs
#         self._assignments = set()  # as Facts
#         self._belief_partitions = defaultdict(set)  #
#         self._evidence_partitions = {} 

    def add_fact(self, fact):
        """
        
        Arguments:
        - `fact`: either a Fact or an (InstantiatedStateVariable, value) tuple
        """
        key, value = Fact.ensure_tuple(fact)
        self._facts[key] = value
        # add to "views" directly here (later!)
    
    def get_facts(self):
        # get from "fact view" (later!)
        return (Fact(key, value) for (key, value) in self._facts.items())

    def __setitem__(self, key, value):
        assert isinstance(key, InstatiatedStateVariable)
        fact = Fact(key, value)
        self._facts.add(fact)
        self._belief_partitions[key.believers].add(fact)
        dict.__setitem__(self, key, value)
        
    def __delitem__(self, key):
        if key not in self:
            return
        value = self[key]
        fact = Fact(key, value)
        self._facts.remove(fact)
        self._belief_partitions[key.believers].remove(fact)
        dict.__delitem__(self, key)



