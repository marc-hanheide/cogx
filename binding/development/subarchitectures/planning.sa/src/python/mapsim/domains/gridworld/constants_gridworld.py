#! /usr/bin/env python
# -*- coding: latin-1 -*-

OCC = "occupant"

OBSTACLE_CHR = '#'

PREF2TYPE_ALL_AGENTS = dict(empty=None, obstacle=None,agt="agent",c="gridcell")
PREF2TYPE_ONE_AGENT =  dict(empty=None, obstacle=None,agt="gridcontent",c="gridcell")

PROB_TEMPLATE = """;; Created automatically by MAPSIM
(define (problem gridworld_prob)
(:domain gridworld)
(:objects
$obj_def
)   ;; end objects
(:init
;; static facts
$static_facts
;; dynamic facts
$facts
$additional_facts
)   ;; end init
(:goal (and
$goal
))) ;; end problem
"""

