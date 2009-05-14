#! /usr/bin/env python

# This is a program to run benchmarks.

# Usage: benchmark.py <time> <memory> <command> [<logfile>]

# <time> is the maximal allowable execution time in seconds.
# <memory> is the maximal allowable space consumption in megabytes.
# <command> is the command to be run (use quotes if the command should include
#           arguments).
# <logfile> is the name for the log file (optional).

from collections import defaultdict

import os
import resource
import sys

import utils
import config
from config import log
from constants import *


last_output_fn = None
last_call = None
tmpcount = 0

def setup_stats():
    global stats
    call_types = "planning actions monitor total".split()
    stats_types = "calls max_duration total_duration timeouts".split()
    def make_stats():
        for call_type in call_types:
            for stat_name in stats_types:
                yield "sys_%s_%s" % (call_type, stat_name), 0
    stats = utils.Struct(**dict(make_stats()))
    config.statistics["syscall"] = stats

setup_stats()

def update_stats(call_type, duration, timed_out):
    base_specific = "sys_%s" % call_type
    base_total = "sys_total"
    for base in (base_specific, base_total):
        stats.__dict__[base+"_calls"] += 1
        stats.__dict__[base+"_total_duration"] += duration
        maxdur = stats.__dict__[base+"_max_duration"]
        if duration > maxdur:
            stats.__dict__[base+"_max_duration"] = duration
        if timed_out:
            stats.__dict__[base+"_timeouts"] += 1
    
def get_temporary_filename(suffix=None):
    global tmpcount
    fn = "tmp"
    if config.current_agent:
        fn += "_%s" % config.current_agent.name
    if suffix:
        fn += "_%s" % suffix
    tmpcount += 1
    fn += "_%s" % str(tmpcount).zfill(3)
    tmpfn = os.path.join(config.current_data_dir, fn)
    return tmpfn

all_calls = defaultdict(int)
all_durations = defaultdict(float)

def run(cmd, input=None, timeout=None, memory=1000, verbose=False, fn_suffix=None):
    global last_output_fn, last_call
    """Runs a command using os.system(), restricting time and space
    resources, preventing core dumps and redirecting the output
    (both stdout and stderr) into a file.

    Parameters:
      cmd     - shell command to be executed
      timeout - timeout in CPU seconds
      memory  - maximum heap size allowed in Megabytes
      verbose - If true, also print the heap and time restrictions,
                the return code of the program and elapsed time.
                If false, this info is logged if there is a log,
                but not printed.
    """

    log_syscalls = False
    if config.debug:
        log_syscalls = True
    log("calling: %s" % cmd, info=INFO_SYSCALLS)
    cmdhead = " ".join(cmd.split()[:2])
    all_calls[cmdhead] += 1
    last_call = cmd
    tmpfn = get_temporary_filename(fn_suffix)
    log("writing to: %s" % tmpfn, info=INFO_SYSCALLS)
    if os.path.exists(tmpfn):
        os.remove(tmpfn)
    redirected_cmd = "(%s) >> %s 2>&1" % (cmd, tmpfn)
    if input is not None:
        sys.exit("No input can be sent to processes currently")

    time_passed_before = os.times()[2] + os.times()[3]

    pid = os.fork()
    if not pid:
        resource.setrlimit(resource.RLIMIT_CPU, (timeout, timeout))
        resource.setrlimit(resource.RLIMIT_DATA, (memory, memory))
        resource.setrlimit(resource.RLIMIT_RSS, (memory, memory))
        #resource.setrlimit(resource.RLIMIT_AS, (memory, memory))
        #resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
        signal = os.system(redirected_cmd)
        if signal % 256 == 0:
            signal = signal // 256
        else:
            signal = signal % 256
        os._exit(signal)

    signal = os.waitpid(pid, 0)[1]
    if signal % 256 == 0:
        signal = signal // 256
    else:
        signal = signal % 256
    execution_error = signal != 0
    if execution_error:
        err_msg = "\nError: the following syscall returned with signal %d:\n%s\nOutput was redirected to:\n%s" % (signal, cmd, tmpfn)
        if config.verbosity == 0:
            raise RuntimeError(err_msg + "\nFor better error output increase --verbosity.")
        print err_msg
        tail_cmd = "tail %s" % tmpfn
        print "\nCalling '%s':" % tail_cmd
        os.system(tail_cmd)
        sys.exit(signal)
        
    timed_out = False
    duration = (os.times()[2] + os.times()[3]) - time_passed_before
    if timeout is not None:
        timed_out = duration >= timeout

    update_stats(fn_suffix, duration, timed_out)

    all_durations[cmdhead] += duration
    for cmdhead in all_calls:
        calls = all_calls[cmdhead]
        dur = all_durations[cmdhead]
        log("call: %s : %.2fs / %d = %.2fs" % (cmdhead, dur, calls, dur/calls), info=INFO_SYSCALLS)


    log("syscall took %.2f secs.  Total time for system calls %.1f secs." % (duration, stats.sys_total_total_duration), info=INFO_SYSCALLS)
    last_output_fn = tmpfn
    result = open(tmpfn).read()
    return timed_out, result




if __name__ == "__main__":
    timeout = int(sys.argv[1])
    memory = int(sys.argv[2])
    command = sys.argv[3]

    log = None
    if len(sys.argv) > 4:
        log = Log(sys.argv[4])

    run(command, timeout, memory, log)
