#!/usr/bin/env python
import sys, errno, time
import argparse

import xml.etree.cElementTree as etree

levels = ['ERROR', 'WARNING', 'INFO', 'DEBUG', 'TRACE']

parser = argparse.ArgumentParser(description='Filter log4j xml files and print resulting xml to standard output.')
parser.add_argument('-p', '--prefix', nargs='?',
                    help='comma-separated list of logger prefixes to filter for')
parser.add_argument('-l', '--level', type=str.upper, default='TRACE', choices = levels, nargs='?',
                    help='filter out loglevels lower than this')
parser.add_argument('file', nargs='?', type=argparse.FileType('r'),
                    default=sys.stdin,
                    help='input filename')

event_str = """<log4j:event logger="%(logger)s" timestamp="%(timestamp)s" level="%(level)s" thread="%(thread)s">
<log4j:message><![CDATA[%(message)s]]></log4j:message>
</log4j:event>"""

def get_levels(max_level):
    for l in levels:
        yield l
        if l == max_level.upper():
            return

        
args =  parser.parse_args()

use_levels = set(get_levels(args.level))
try:
    loggers = [l.lower() for l in args.prefix.split(",")]
except AttributeError:
    loggers = []

etree.register_namespace('log4j', 'http://jakarta.apache.org/log4j/')

def handle_event(elem):
    if elem.get("level") not in use_levels:
        return False
    if not loggers:
        return True
    if any(elem.get("logger").lower().startswith(l) for l in loggers):
        return True
    
    return False

def print_header(elem):
    user = elem.get("user")
    time = elem.get("time")
    print '<?xml version="1.0" encoding="UTF-8"?>'
    print '<log4j:logsequence xmlns:log4j="http://jakarta.apache.org/log4j/" user="%s" time="%s">' % (user, time)

root = None

read = 0
written = 0

t0 = time.time()

try:
    for event, elem in etree.iterparse(args.file, events=("start", "end")):
        if event == "start" and elem.tag == "{http://jakarta.apache.org/log4j/}logsequence":
            print_header(elem)
            root = elem

        elif event == "end" and elem.tag == "{http://jakarta.apache.org/log4j/}event":
            read += 1
            if handle_event(elem):
                # can't use the etree.tostring() method incerementally as it add xmlns:log4j attributes everywhere
                # so do it the quick and dirty way
                message_elem = iter(elem).next()
                elem.attrib['message'] = message_elem.text
                written += 1
                print event_str % elem.attrib
                
            root.clear()

            # if read % 10001 == 10000:
            #     print >>sys.stderr, "read %d, written %d, time:%.2f" % (read, written, time.time()-t0)
            #     t0=time.time()

            

except IOError as e:
    if e.errno == errno.EPIPE:
        exit(0)
    else:
        raise e

except etree.ParseError:
    pass

print '</log4j:logsequence>'





# parser = etree.XMLTreeBuilder()

# def end_tag_event(tag):
#     node = self.parser._end(tag)
#     print node

# parser._parser.EndElementHandler = end_tag_event


# with open(sys.argv[1]) as f:
#     for line in f:
#         parser.feed(line)
