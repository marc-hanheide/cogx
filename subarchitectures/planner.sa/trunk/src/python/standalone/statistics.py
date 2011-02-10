#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest
import time
import itertools


class Statistics(dict):
    def __init__(self, *args, **kwargs):
        self.defaults = kwargs.pop("defaults", {})
        self.update(self.defaults)
        dict.__init__(self, *args, **kwargs)

    def merge(self, other_stat):
        """ merge values of another stat into this one, but only if
            the relevant field is still set to the default value """
        assert isinstance(other_stat, Statistics)
        assert not set(self.keys()).intersection(other_stat.keys())
        return Statistics(itertools.chain(self.iteritems(), other_stat.iteritems()))

    def set_stat(self, name, value):
        assert name in self
        self[name] = value

    def increase_stat(self, name, inc=1):
        self[name] += inc

    def reset(self):
        self.clear()
        self.update(self.defaults)

    def sorted_repr(self, sort_order=[]):
        sort_dict = dict((v,i) for i,v in enumerate(sort_order))
        sorted_elems = sorted(self.iteritems(), key=lambda (x,_): sort_dict.get(x, 1000))
        return "{%s}" % (", ".join("%s: %s" % (repr(k), repr(v)) for k,v in sorted_elems))



def time_method_for_statistics(stats_name):
    """ decorator for timing methods and storing the duration into
    the 'statistics' field of the respective object.  

    usage: @time_method_for_statistics(stats_name)
    """
    def decorator(amethod):
        def decorated_method(self, *args, **kwargs):
            t0 = time.time()
            rval = amethod(self, *args, **kwargs)
            dur = time.time() - t0
            # the following assumes the decorated method belongs to a
            # class that has a 'statistics' field. Will throw an
            # AttributeError otherwise.
            self.statistics.increase_stat(stats_name, dur)
            return rval
        return decorated_method
    return decorator

class time_block_for_statistics(object):
    """context manager to time a block of statements into the
    'statistics' field of the 'obj' parameter.

    useage: with time_block_for_statistics(self, stats_name):
                code to measure
    """
    def __init__(self, obj, stats_name):
        self.obj = obj
        self.stats_name = stats_name
        
    def __enter__(self):
        self.t0 = time.time()

    def __exit__(self, type, value, traceback):
        dur = time.time() - self.t0
        self.obj.statistics.increase_stat(self.stats_name, dur)
            
class StatisticsTest(unittest.TestCase):

    def setUp(self):
        self.stats = Statistics(defaults=dict(some_stat=17))

    def test_get_set(self):
        stats = self.stats
        stats.set_stat("some_stat", 4)
        plans = stats["some_stat"]
        assert plans == 4

    def test_get_unknown_stat(self):
        def try_sth():
            replans = self.stats["unknown_stat"]
        self.assertRaises(KeyError, try_sth)

    def test_increase_stat(self):
        stats = self.stats
        stats.increase_stat("some_stat")

    def test_merge(self):
        other_stats = Statistics(defaults=dict(another_stat=42))
        stats = self.stats
        merged = stats.merge(other_stats)
        assert len(merged) == 2
        other_stats2 = Statistics(defaults=dict(some_stat=42))
        self.assertRaises(AssertionError, stats.merge, other_stats2)
        
if __name__ == '__main__':
    unittest.main()
