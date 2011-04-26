#! /usr/bin/env python2.5
# -*- coding: utf-8 -*-

import sys

for line in sys.stdin:
    if line.strip() == "*File List*":
        for line in sys.stdin:
            if line.strip() == "***********":
                break
            print line.split()[0]

