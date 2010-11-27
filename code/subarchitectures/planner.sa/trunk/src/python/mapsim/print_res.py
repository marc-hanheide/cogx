#! /usr/bin/env python
import sys, itertools
from collections import defaultdict

pfile = sys.argv[1]
if len(sys.argv) > 2:
    confs = sys.argv[2:]
else:
    confs = []

pf = open(pfile)
pf.readline()

configs = {}

def extract_dict(line):
    try:
        exec("sdict = %s" % line)
    except:
        print "couldn't parse: %s" % line
        raise
    return sdict

cf_order = confs[:]

while True:
    cfline = pf.readline()
    if not cfline:
        break
    if not cfline.startswith("config "):
        continue
    conf = cfline.split()[1]
    cfline = pf.readline()
    cheader, count = cfline.split()
    assert cheader == "count"
    count = int(count)
    for i in xrange(count):
        pf.readline()
        
    avg = extract_dict(pf.readline())
    if not confs or conf in confs:
        if not confs:
            cf_order.append(conf)
        configs[conf] = avg
    

if not confs:
    cf_order = sorted(cf_order)
    print map(str, cf_order)

print "results for", pfile

fields = [("cost", "total_plan_cost", "%5d"),
          ("actions", "sensor_actions_executed + physical_actions_executed", "%5d"),
          # ("reward", "100 * successful_runs - total_plan_cost", "%5d"),
          ("time", "planning_time + dt_planning_time", "%5.1f"),
          ("cp time", "planning_time", "%5.1f"),
          ("dt time", "dt_planning_time", "%5.1f"),
          ("success", "successful_runs", "%5.2f"),
          ("#", "sample_count", "%5d")]

def get_field(f, c, format):
    if c == '-':
        return "|"
    if not c in configs:
        return "--"
        
    locals().update(configs[c])
    return format % eval(f)

count = 0
for fname, f, format in fields:
    #for s_tup in itertools.izip_longest(*file_stats, fillvalue={}):
    #for c in cf_order:
    results = [get_field(f, c, format) for c in cf_order]
    print "%10s: " % fname + " ".join(results)
        
