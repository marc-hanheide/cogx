#! /usr/bin/env python2.5
# -*- coding: utf-8 -*-

import os.path
import re


class Expander(object):
    def __init__(self, regex, filename_func):
        self.regex = re.compile(regex)
        self.filename_func = filename_func

    def expand_line(self, line):
        match = self.regex.search(line)
        if not match:
            return False, line
        if "%" in line.rstrip().rstrip("%"):
            raise SystemExit("Don't give me a headache: %r." % line)
        input_filename = self.filename_func(match)
        if not os.path.splitext(input_filename)[1]:
            input_filename += ".tex"
        input_text = open(input_filename).read()
        return True, line[:match.start()] + input_text + line[match.end():]
        

def expand(expanders, text):
    result = []
    for line in text.splitlines():
        line += "\n"
        for expander in expanders:
            expanded, line = expander.expand_line(line)
            if expanded:
                line = expand(expanders, line)
        result.append(line)
    return "".join(result)


def main(input_filename, bib_filename):
    # TODO:
    # This doesn't work with \includefigure etc. that are used in,
    # for example, the AAAI 2008 NECTAR paper. These aren't simple
    # include macros, but also do other stuff, so dealing with these
    # would be quite a bit tougher. Ignore them for now.
    
    expanders = [
        Expander(r"\\input\s*\{([^#]*?)\}", lambda match: match.group(1)),
        Expander(r"\\bibliography\{.*?\}", lambda match: bib_filename),
        ]

    input_text = open(input_filename).read()
    output_text = expand(expanders, input_text)
    sys.stdout.write(output_text)


if __name__ == "__main__":
    import sys
    main(*sys.argv[1:])
