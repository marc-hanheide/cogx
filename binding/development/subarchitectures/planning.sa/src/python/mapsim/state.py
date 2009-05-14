#! /usr/bin/env python
# -*- coding: latin-1 -*-

import config
from config import log
import utils
from constants import *
from planning.mapl.predicates import Predicate

modalities = dict(k="k", kd="k")

def str_assignment(key, val, add_parens=True):
    key_str = str(key)
    if isinstance(val, tuple):
        val = " ".join(val)
    
    mapl_domain = config.get_mapl_domain()
    assert mapl_domain is not None
    
    # try to find out whether an explicit "true" is needed
    is_equality = key_str == "="
    is_k_pred = key_str.startswith(K_PREFIX+PREFIX_SEP) or key_str.startswith(DIRECT_K_PREFIX+PREFIX_SEP)
    if is_k_pred:
        assert "__" in key.name
        k_modality, svar = key.name.split("__", 1)
        agt = key.args[0]
        key_str = "K %s (%s %s)" % (agt, svar, " ".join(key.args[1:]))
    is_in_domain_pred = key_str.startswith(IN_DOMAIN_KW)
    omit_value = is_equality or is_in_domain_pred

    if omit_value:
        assert val in [TRUE_STRING, FALSE_STRING]
        if val == TRUE_STRING:
            result = key_str
        elif val == FALSE_STRING:
            result = "not (%s)" % key_str
    else:
        result = "%s : %s" % (key_str, val)
    if add_parens:
        result = "(%s)" % result

    if omit_value:
        #print result, "IDP: true"
        if val not in BOOLEAN_VALUE_STRINGS:
            raise
    return result




class Assignment(object):
    def __init__(self, svar, val):
        self.svar = svar
        self.val = val
    def to_mapl_str(self, add_parens=True):
        pass
    def to_pddl_str(self, add_parens=True):
        pass



class StateVariable(object):
    def __init__(self, args):
        self.name = args[0]
        self.args = tuple(args[1:])
    def __hash__(self):
        return hash((self.name, self.args))
    def __eq__(self, other):
        return (self.__class__ is other.__class__ and
                self.name == other.name and
                self.args == other.args)
    def split_modal_pddl_representation(self):
        # turns something like (kd__pos agt obj true) into "K, agt, (pos obj)"
        # returns both the new svar and the agents name
        if not PREFIX_SEP in self.name:
            return "", "", self
        prefix, suffix = self.name.split(PREFIX_SEP, 1)
        prefix = prefix.lower()
        if prefix not in modalities:
            return "", "", self
        prefix = modalities[prefix]
        agt_name = self.args[0]
        elmts = [suffix] + list(self.args[1:])
        new_svar = StateVariable(elmts)
        return prefix, agt_name, new_svar
    def in_domain_proposition(self, val):
        nname = Predicate.svar_in_domain_predicate_name(self.name)
        nargs = list(self.args) + [val]
        return StateVariable([nname]+nargs)
    def knowledge_proposition(self, agt):
        kprop_name = K_PREFIX + PREFIX_SEP + self.name
        return StateVariable([kprop_name, agt]+self.args)
    def is_knowledge_proposition(self):
        return self.name.startswith(K_PREFIX+PREFIX_SEP) or self.name.startswith(DIRECT_K_PREFIX+PREFIX_SEP)
    def is_own_knowledge(self, agt):
        """returns True if the variable is a knowledge proposition of "agt" """
        if self.name.startswith(K_PREFIX+PREFIX_SEP):
            return self.args[0].lower() == agt.lower()
        return False
    def is_in_domain(self):
        """returns True if the variable is a in_domain proposition"""
        return self.name.startswith(IN_DOMAIN_KW+PREFIX_SEP)
    def is_negative(self):
        return self.name[0] == "!"
    def invert(self):
        """returns the negative proposition"""
        if self.is_negative():
            nname = self.name[1:]
            return StateVariable((nname,) + self.args)
        else:
            nname = "!"+self.name
            return StateVariable((nname,) + self.args)
    def base_of_knowledge_proposition(self, agt):
        """returns the underlying proposition of a knowledge-proposition, e.g. pos object for k__pos agent object"""
        if not self.is_own_knowledge(agt):
            return None
        nname = self.name[len(K_PREFIX + PREFIX_SEP):]
        return StateVariable((nname,) + self.args[1:])
    def base_of_in_domain_proposition(self):
        """returns the underlying proposition of a in_domain proposition, e.g. pos object for in_domain__pos object value"""
        if not self.is_in_domain():
            return None
        nname = self.name[len(IN_DOMAIN_KW + PREFIX_SEP):]
        return StateVariable((nname,) + self.args[:-1])
    def __str__(self):
        return " ".join((self.name,) + self.args)


def mapl_str2assignment(mapl_str):
    """turns a MAPL assignment string, e.g. 'svar_name arg1 arg2 : val' into an (svar, val) tuple"""
    if ":" in mapl_str:
        svar_s, val = mapl_str.split(":")
        val = (val.strip(),)
    else:
        svar_s = mapl_str
        val = TRUE_TUP
    svar_l = [s.strip() for s in svar_s.split()]
    svar = StateVariable(svar_l)    
    return svar, val

def mapl_strings2state(alist):
    """turns a list of MAPL assignment strings, e.g. ['svar_name arg1 arg2
    : val'] into a list of (svar, val) tuples"""
    state = State()
    tups = [mapl_str2assignment(mapl_str) for mapl_str in alist]
    for svar, val in tups:
        state[svar] = vals
    return state
    

        
class State(dict):
    """ State: a time-stamped dict"""
    def __init__(self, assignments=[], memory_span=0, mapl_domain=None):
        self.time = 0
        self.memory_span = memory_span  # permanent memory by default
        self.key2time = {}
        self._mapl_domain = mapl_domain
        for svar, val in assignments:
            self[svar] = val
        
    def copy(self):
        s = State()
        s.time = self.time
        s.memory_span = self.memory_span
        s.key2time = self.key2time
        s._mapl_domain = self._mapl_domain
        for svar, val in self.items():
            self[svar] = val
        return s
    
    def __setitem__(self, key, value):
        assert isinstance(key, StateVariable)
        dict.__setitem__(self, key, value)
        self.key2time[key] = self.time
        
    def __getitem__(self, key):
        try:
            v = dict.__getitem__(self, key)
        except KeyError, e:
            log("couldn't find key '%s' of type %s" % (key, type(key)), vlevel=5)
            raise e
        return v

    def __delitem__(self, key):
        if key in self:
            dict.__delitem__(self, key)
            del self.key2time[key]

    def set_time(self, value):
        self.time = value
        self.remove_dated_values()

    def set_memory_span(self, value):
        self.memory_span = value
        self.remove_dated_values()
        
    def remove_dated_values(self):
        if not self.memory_span:
            return
        dated = []
        time = self.time
        for key in self:
            age = time - self.key2time[key]
            if age > self.memory_span:
                dated.append(key)
        for key in dated:
            del self[key]
            del self.key2time[key]

    def update(self, d):
        try:
            for k, v in d.items():
                self[k] = v
        except AttributeError:
            # d may also a be a sequence of key-value pairs
            try:
                for k, v in d:
                    self[k] = v
            except:
                print "the following should be a dict or (k,v) sequence, but is not:", d
                raise
            
    def values(self):
        return (self[k] for k in self.keys())

    def items(self):
        return ((k,self[k]) for k in self.keys())

    def get_mapl_domain():
        if self._mapl_domain is not None:
            return self._mapl_domain
        from_config = config.get_mapl_domain()
        if from_config:
            return from_config
        return None

    def items_by_svar(self, svar_name):
        for key, val in self.items():
            assert isinstance(key, StateVariable)
            if key.name == svar_name:
                yield key, val

    def mapl_assignment(self, key, add_parens=True):
        val = self[key]
        return str_assignment(key, val, add_parens)

    def to_mapl_factlist(self, add_parens=True):
        facts = [self.mapl_assignment(key, add_parens) for key in self]
        NOT_PREF = "(not ("
        if not add_parens:
            NOT_PREF = NOT_PREF[1:]
        facts = [f for f in facts if not f.startswith(NOT_PREF)]
        return facts

    def compute_difference(self, astate):
        svars = set(self.keys()).union(astate.keys())
        def gen():
            for svar in svars:
                val1 = self.get(svar)
                val2 = astate.get(svar)
                if val1 != val2:
                    yield (svar, val1, val2)
        return list(gen())

if __name__ == "__main__":
    pass
