#! /usr/bin/env python
# -*- coding: latin-1 -*-

from string import Template
from utils import Struct
import state
from constants_gridworld import *

def sdist(t1, t2):
    r1, c1 = t1
    r2, c2 = t2
    return max(abs(r1-r2), abs(c1-c2))

def mhdist(t1, t2):
    r1, c1 = t1
    r2, c2 = t2
    return abs(r1-r2) + abs(c1-c2)

def make_mapl_prob(prob, sensor_range):
    size = prob.size
    templ = Template(prob.mapl_prob)
    tuples = [(r,c) for r in xrange(size) for c in xrange(size)]
    tup2cell = {}
    for (r,c) in tuples:
        tup2cell[(r,c)] = "c%d%d" % (r,c)
    cells_def = ["  %s - gridcell" % cell for cell in tup2cell.values()]
    cartesian__tups = [(t1,t2) for t1 in tuples for t2 in tuples]
    conn_facts = []
    distance_facts = []
    for t1, t2 in cartesian__tups:
        sd = sdist(t1, t2)
        mh = mhdist(t1, t2)
        if mh == 1:
            conn_facts.append("  (connected %s %s)" % (tup2cell[t1], tup2cell[t2]))
        if sd <= sensor_range:
            distance_facts.append("  (in-sensing-distance %s %s)" % (tup2cell[t1], tup2cell[t2]))
    subs = dict(cells_def=cells_def, conn_facts=conn_facts, distance_facts=distance_facts)
    for k in subs:
        subs[k] = "\n".join(subs[k])
    return templ.safe_substitute(subs)

def parse_graphical_state(gstate, show_only_agents=False):
    for row, line in enumerate(gstate):
        for col, c in enumerate(line[1:-1]):  #ignore stars
            if c == ' ':
                c = "empty"
            elif c == OBSTACLE_CHR:
                c = "obstacle"
            else:
                c = "agt%s" % c
            if not show_only_agents or c.startswith("agt"):
                yield (c, col, row)

def load(fn):
    lines = [line.strip() for line in open(fn) if line]
    lines = [line for line in lines if line and not line.startswith("#")]
    size = (len(lines) - 4) // 2   # ignore boundaries
    if any(len(line) != size+2 for line in lines):
        print "No square grid!"
        sys.exit()
    init, goal = lines[1:size+1], lines[size+3:-1]
    itups = list(parse_graphical_state(init))
    gtups = list(parse_graphical_state(goal))
    agents = [name for name,_,_ in itups if name not in ("empty","obstacle")]
    return Struct(size=size, agents=agents, itups=itups, gtups=gtups)

def state2graphics(state, size):
    grid = [[" " for i in range(size)] for k in range(size)]
    for k,v in state.items():
        if isinstance(v, tuple) and len(v)==1:
            v = v[0]
        if k.name != "occupant":
            continue
        cell = k.args[0]
        x, y = int(cell[1]), int(cell[2])   # TODO: change format to allow sizes >10
        if v == "obstacle":
            sym = "#"
        elif v == "empty":
            sym = " "
        elif v.startswith("agt"):
            sym = v[-1]
        else:
            raise Exception("bad value %s for 'occupant' state variable!" % v)
        grid[y][x] = sym
    border = " " +(size+2)*"-"
    numbers = "  " + "".join(str(i) for i in xrange(size)) + " "
    lines = [numbers, border] + ["%s|%s|" % (numbers[i+2], "".join(grid[i])) for i in xrange(size)] + [border, numbers] 
    return lines


    
if __name__ == "__main__":
    import sys
    fn = sys.argv[1]
    prob = load_graphical_problem_rep(fn)
    prob = make_mapl_prob(prob, int(sys.argv[2]))
    print prob
