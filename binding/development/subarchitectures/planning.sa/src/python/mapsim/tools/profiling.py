#! /usr/bin/env python
# -*- coding: latin-1 -*-

import pstats

def analyze_stats(profile_stats):
    p = pstats.Stats(profile_stats)
    p = p.strip_dirs()
    p.sort_stats('cumulative').print_stats(20)
    p.sort_stats('time').print_stats(20)
   # p.sort_stats('cumulative').print_callers(20)



