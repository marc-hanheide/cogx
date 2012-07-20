#! /usr/bin/env python
import sys, itertools
from collections import defaultdict

pfile = sys.argv[1]
add_args = []
confs = []
for i,a in enumerate(sys.argv[2:]):
    if a == "--":
        add_args = sys.argv[i+2:]
        break
    confs.append(a)

pf = open(pfile)
pf.readline()

configs = {}
configs_common = {}
configs_succ = {}
configs_common_succ = {}

def extract_dict(line):
    try:
        exec("sdict = %s" % line)
    except:
        print "couldn't parse: %s" % line
        raise
    return sdict

cf_order = confs[:]

statsdict = {}

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
    stats = {}
    for i in xrange(count):
        seed = int(pf.readline().strip())
        stats[seed] = extract_dict(pf.readline())
        
    avg = extract_dict(pf.readline())
    avg_common = extract_dict(pf.readline())
    avg_succ = extract_dict(pf.readline())
    avg_common_succ = extract_dict(pf.readline())
    if not confs or conf in confs:
        if not confs:
            cf_order.append(conf)
        statsdict[conf] = stats
        configs[conf] = avg
        configs_common[conf] = avg_common
        configs_succ[conf] = avg_succ
        configs_common_succ[conf] = avg_common_succ

if not confs:
    cf_order = sorted(cf_order)
    print map(str, cf_order)

fields = [("cost", "total_plan_cost", "%5d"),
          ("actions", "sensor_actions_executed + physical_actions_executed", "%5d"),
          ("reward", "reward", "%5.1f"),
          ("time", "preprocess_time + translate_time + search_time + dt_planning_time", "%5.1f"),
          ("pptime", "planning_time + dt_planning_time", "%5.1f"),
          ("cp time", "preprocess_time + translate_time + search_time", "%5.1f"),
          ("dt time", "dt_planning_time", "%5.1f"),
          ("cp avg. time", "planning_time/planning_calls", "%5.1f"),
          ("dt avg. time", "dt_planning_time/dt_planning_calls", "%5.1f"),
          ("dt avg. time2", "dt_planning_time/dt_actions_received", "%5.1f"),
          ("success", "successful_runs * sample_count", "%5.2f"),
          ("failures", "failed_execution_attempts * sample_count", "%5.2f"),
          ("#", "sample_count", "%5d")]

def calc_averages(stats, filters, common_filters):
    def eval_dict(d, f):
        locals().update(d)
        return eval(f)

    no_data_confs = set()
    common_seeds = None
    if common_filters:
        for conf, s in stats.iteritems():
            seeds = set()
            for seed, d in s.iteritems():
                if all(eval_dict(d, f) for f in common_filters):
                    seeds.add(seed)
            if not seeds:
                no_data_confs.add(conf)
            elif common_seeds is None:
                common_seeds = seeds
            else:
                common_seeds &= seeds

    # common_seeds = set(range(0,30))

    cavgs = {}
    for conf, s in stats.iteritems():
        if conf in no_data_confs:
            cavgs[conf] = {}
            continue
        averages = defaultdict(lambda: 0)
        count = 0
        for seed, d in s.iteritems():
            if common_seeds and seed not in common_seeds:
                continue
            if all(eval_dict(d, f) for f in filters):
                for key, value in d.iteritems():
                    averages[key] += value
                count += 1
        if count == 0:
            cavgs[conf] = {'sample_count' : 0}
            continue
        
        for key, value in averages.iteritems():
            averages[key] = float(value) / count
        averages['sample_count'] = count
        cavgs[conf] = averages
    return cavgs

def get_field(f, c, format, d):
    if c == '-':
        return "|"
    if not c in d:
        return "--"
        
    locals().update(d[c])
    try:
        return format % eval(f)
    except:
        return format % 0

def print_set(fields, filters=[], common_filters=[]):
    cavgs = calc_averages(statsdict, filters, common_filters)
    name_format = "%%%ds: " % max(len(fname) for fname, _,_ in fields)
    
    for fname, f, format in fields:
        #for s_tup in itertools.izip_longest(*file_stats, fillvalue={}):
        #for c in cf_order:
        results = [get_field(f, c, format, cavgs) for c in cf_order]
        print name_format % fname + " ".join(results)

def print_set_gnuplot(fields, filters=[], common_filters=[]):
    cavgs = calc_averages(statsdict, filters, common_filters)
    for c in cf_order:
        print c, " ".join(get_field(f, c, format, cavgs) for fname, f, format in fields)

if "gnu" in add_args:
    # fields = [("cp time", "preprocess_time + translate_time + search_time", "%5.1f"),
    fields = [("cp time", "preprocess_time + translate_time + search_time", "%5.1f"),
              ("dt time", "dt_planning_time", "%5.1f"),
              ("success", "(successful_runs - failed_execution_attempts) * sample_count", "%5.4f"),
              ("cost", "total_plan_cost", "%5d"),
              ]
    print_set_gnuplot(fields)
elif "gnupomdp" in add_args:
    fields = [("cp time", "preprocess_time + translate_time + search_time", "%5.1f"),
              ("dt time", "dt_planning_time", "%5.1f"),
              ("success", "(successful_runs - failed_execution_attempts) * sample_count", "%5.4f"),
              ("reward", "reward", "%5d"),
              ]
    print_set_gnuplot(fields)
else:
    print "results for", pfile
    print_set(fields)
    print
    print_set(fields, common_filters=["True"])
    print_set(fields, filters=["successful_runs > 0"])
    # print
    # print_set(fields, common_filters=["successful_runs == 0"])
        
# print
# for fname, f, format in fields:
#     #for s_tup in itertools.izip_longest(*file_stats, fillvalue={}):
#     #for c in cf_order:
#     results = [get_field(f, c, format, configs_common) for c in cf_order]
#     print "%10s: " % fname + " ".join(results)

# print
# for fname, f, format in fields:
#     #for s_tup in itertools.izip_longest(*file_stats, fillvalue={}):
#     #for c in cf_order:
#     results = [get_field(f, c, format, configs_succ) for c in cf_order]
#     print "%10s: " % fname + " ".join(results)

# print
# for fname, f, format in fields:
#     #for s_tup in itertools.izip_longest(*file_stats, fillvalue={}):
#     #for c in cf_order:
#     results = [get_field(f, c, format, configs_common_succ) for c in cf_order]
#     print "%10s: " % fname + " ".join(results)

