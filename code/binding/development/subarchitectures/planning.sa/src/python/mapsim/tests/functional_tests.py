from __future__ import with_statement
from configobj import ConfigObj
import os, os.path, sys
import time

import support
import config
import utils
import main
import reporter

test_files_path = os.path.join(config.mapsim_dir, config.test_files_path)
test_log = "functional_test.log"
log_file = None

standard_params = "-v 0"

class TestCatalogue(object):
    config = {}
    __config_obj = None
    
    def __init__(self, fname):
        config = ConfigObj(fname, unrepr=True)
        if 'tests' not in config:
            raise KeyError, "%s does not contain any tests" % fname
        if 'config' in config:
            self.config = config['config']
        self.__config_obj = config

    def __get_all_tests(group, prefix):
        result = []
        for key, value in group.iteritems():
            if isinstance(value, dict):
                result += TestCatalogue.__get_all_tests(value, prefix + "." + key)
            else:
                result.append( (prefix + "." +key, value) )
        return result
    __get_all_tests = staticmethod(__get_all_tests)
    
    def get_tests_from_name(self, name):
        path = name.split(".")
        if path and not path[0]:
            path = None

        tests = []
            
        current = self.__config_obj['tests']
        prefix = ""
        while path:
            if path[0] not in current:
                raise NameError, "%s does not exist in section %s" % (path[0], prefix)
            current = current[path[0]]
            prefix += "." + path[0]
            path.pop(0)

        if isinstance(current, dict):
            tests = self.__get_all_tests(current, prefix)
        else:
            tests = [(prefix, current)]

        return tests
    
    def contains(self, name):
        try:
            tests = self.get_tests_from_name(name)
            return True
        except:
            return False

    def add(self, name, value):
        path = name.split(".")
        if path and not path[0]:
            path = None

        previous = None
        current = self.__config_obj['tests']
        current_name = ""
        while path:
            if path[0] not in current:
                if len(path) == 1:
                    current[path[0]] = value
                    self.__config_obj.write()
                    return True
                else:
                    current[path[0]] = {}
            previous = current
            current = current[path[0]]
            current_name = path.pop(0)
        
        if isinstance(current, basestring):
            previous[current_name] = value
            return True

        print "The entry %s does already exist" % name
        return False

    indent_depth = 4
    def output(self):
        def print_items(catalogue, indent_level):
            for key, value in catalogue.iteritems():
                if isinstance(value, dict):
                    print " "*indent_level*self.indent_depth + "[%s]" % key
                    print_items(value, indent_level+1)
                else:
                    print " "*indent_level*self.indent_depth + key
                    
        
        print_items(self.__config_obj['tests'], 0)    
        

def output(*tup):
    if isinstance(tup, tuple):
        short, verbose = tup
    else:
        short, verbose = "", tup  # tup is a message in this case
    print >>log_file, verbose
    if options.verbose:
        print verbose
    else:
        sys.stdout.write(short)


def clean(lines):
    def gen(lines):
        for line in lines:
            line = line.split("#", 1)[0].strip()  # remove comments and whitespace
            if line.startswith("Runtime"):
                continue
            line = reporter.remove_line_number(line)
            if not line:
                continue
            yield line
    return list(gen(lines))


def reports_are_different(actual_report, expected_report):
    """returns a failure report if reports are different, False if they are equivalent."""
    actual_report = clean(actual_report)
    expected_report = clean(expected_report)
    for num, (line1, line2) in enumerate(utils.izip_longest(actual_report, expected_report, fillvalue="<empty line>")):
        if line1 != line2:
            return "Line %d differs:\nexpected: %s\nfound:    %s" % (num, line2, line1)
    return ""

def verify(actual_report_fn, expected_results):
    # currently expected_results_fn is a report.log file.
    # TODO: extend to .stat files (ie enable comparison of statistics)
    actual_report = open(actual_report_fn).readlines()
    diff_msg = reports_are_different(actual_report, expected_results)
    short_result = "F" if diff_msg else "."
    return (short_result, diff_msg)

def run_and_verify(ex_result, name, filename ):
    sys.stdout.flush()
    output("", "Running test %s (File: %s)" % (name, filename))
    try:
        config.verbosity=0
        main.testmain()
    except Exception, e:
        if options.debug:
            raise
        output("E", "An error occured during test %s\n" % name)
        return False, "An error occured during test %s" % name
    
    actual_report_fn = os.path.join(config.mapsim_dir, config.reporter_file_name)
    short_result, diff_msg = verify(actual_report_fn, ex_result)
    if diff_msg:
        verbose_msg = "TEST %s FAILED! MAPSIM could not replicate report in file %s\nFailure message: %s\n" % \
            (name, filename, diff_msg)
    else:
        verbose_msg = "passed.\n"
    output(short_result, verbose_msg)

    return (not diff_msg), diff_msg

# return tuple (config, result)
def load_testfile(fname):
    config_part = []
    result_part = None
    for line in open(fname):
        if line.strip() == "[result]":
            result_part = []
            continue
        if result_part == None:
            config_part += [line.strip()]
        else:
            result_part += [line.strip()]

    config = ConfigObj(config_part, unrepr=True)

    return config.get('config'), result_part

def compare(test):
    (name, file) = test
    (cfg, result) = load_testfile(os.path.join(config.test_files_path, file))
    config.add_configuration(cfg)

    output("", "Running test %s (File: %s)" % (name, file))
    try:
        main.testmain()
    except Exception, e:
        if options.debug:
            raise
        print "An error occured during simulation %s" 
        return

    print "The following report was expected:"
    for line in result:
        print line
    
    actual_report_fn = os.path.join(config.mapsim_dir, config.reporter_file_name)
    short_result, diff_msg = verify(actual_report_fn, result)
    
    if not diff_msg:
        diff_msg = "no differences"
    
    print "\nVerification result:", diff_msg

options = None
def run_all_tests(the_options):
    global options
    options = the_options

    catalogue = TestCatalogue(config.functional_tests_file)
    
    if options.list:
        catalogue.output()
        return
    
    if options.file:
        tests = [(os.path.basename(options.file), os.path.abspath(options.file))]
    else:
        tests = catalogue.get_tests_from_name(options.test)
        
    if options.compare and len(tests) != 1:
        print "--compare can only be used with one test"
        return
    elif options.compare:
        config.add_configuration(catalogue.config)
        compare(tests[0])
        return
        
    global log_file
    with open(test_log, "w") as log_file:
        print "Starting %d functional tests now." % len(tests)
        start_time = time.time()
        
        passed = 0
        results = []
        for name, file in tests:
            (cfg, result) = load_testfile(os.path.join(config.test_files_path, file))
            config.add_configuration(catalogue.config)
            config.add_configuration(cfg)
            (success, report) = run_and_verify(result, name, file)
            results.append( (name, file, success, report) )
            if success:
                passed += 1

        duration = time.time() - start_time

        print "\n\n%d of %d tests passed." % (passed, len(tests))
        if passed < len(tests) :
            print "The following tests failed:"
            for name, file, success, report in results:
                if not success:
                    print "%s: %s" % (name, file)
        print "Completed functional testing in %.1f secs." % round(duration,1)
    

###############################################

def prepare_args(args):
    if args[0].endswith("/"):
        args[0] = args[0][:-1]
    def gen_new_args ():
        lastarg = None
        for arg in args[1:]:
            if not arg.startswith("-"):
                assert lastarg, "no option specified for parameter %s" % arg
                yield " ".join((lastarg, arg))
                lastarg = None
            else:
                if lastarg:
                    yield lastarg
                lastarg = arg
        if lastarg:
            yield lastarg
    def filter_pred(arg):
        filter_out = ["-c", "-v", "--compare2test"]
        return arg.split(" ")[0] not in filter_out
    newargs = sorted(arg for arg in gen_new_args() if filter_pred(arg))
    args[1:] = newargs
    return args

config_diff_ignorelist = ['create_functional_test', 'test_file', 'reporter', 'test_files_path',
    'functional_tests_file']

def add_run_to_functional_tests(args):
    assert args[1] == "-t", "future functional tests must run called with 'mapsim -t'."
    args = args[args.index("-t")+1:]
    args = prepare_args(args)
    cmd_line_args = " ".join(args)

    catalogue = TestCatalogue(config.functional_tests_file)

    diff = config.get_config_diff(catalogue.config)
    conf = ConfigObj(unrepr=True)
    for key, value in diff.iteritems():
        if key not in config_diff_ignorelist:
            conf[key] = value

    is_new = True
    if catalogue.contains(config.create_functional_test):
        if len(catalogue.get_tests_from_name(config.create_functional_test)) == 1:
            is_new = False
        else:
            default_name = os.path.basename(config.test)
            count = 0
            while catalogue.contains(config.create_functional_test+"."+default_name):
                count += 1
                default_name = "%s-%d" % (os.path.basename(config.test), count)
            config.create_functional_test += "." + default_name
            
    if not config.test_file:
        config.test_file = config.create_functional_test +".rep"
    else:
        config.test_file = os.path.abspath(config.test_file)
        
    fname = os.path.join(config.test_files_path, config.test_file)
            
    catalogue.add(config.create_functional_test, config.test_file)

    f = open(fname, "w")
    print >>f, "# This run should be replicable by running:"
    print >>f, "# mapsim -t %s" % cmd_line_args
    print >>f, "[config]"
    for line in conf.write():
        print >>f, line
        
    print >>f, "[result]"
    for line in config.reporter.report:
        print >>f, line
    f.close()

    if is_new:
        print "Added new functional test '%s'" % config.create_functional_test
    else:
        print "Updated functional test '%s'" % config.create_functional_test

