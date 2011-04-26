#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys

for line in sys.stdin:
    if line.strip() == "*File List*":
        for line in sys.stdin:
            if line.strip() == "***********":
                break
            print line.split()[0]

