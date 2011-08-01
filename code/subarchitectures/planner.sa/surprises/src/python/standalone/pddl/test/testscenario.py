#! /usr/bin/env python
# -*- coding: latin-1 -*-
#from __future__ import absolute_import

import unittest
import tempfile
import os

import predicates, parser, domain, scenario, problem
from parser import Parser, ParseError

dora_domain = \
"""
(define (domain cogx)

        (:requirements :mapl :adl :durative-actions :object-fluents :action-costs)
	
	(:types 
		place - object
		room - object
		robot - agent
		place_id - object  ;; make this a number at some point 
		room_id - object  ;; make this a number at some point 
		place_category - object
	)
	
	(:constants
        default_place - place_category
        R2D2 - robot
	)

    (:predicates
		(connected ?n1 - place ?n2 - place)
                (located ?r -  agent ?n - place)
		(contains ?r - room ?n - place)
		;; special
		(has-goal ?a - agent)
		(commited ?a - agent)
    )	

	(:functions
		(place_id ?n - place) - place_id
		(explored ?n - place) - boolean
		(place_category ?n - place) - place_category
		(areaclass ?r - room) - place_category
 	)
	

;;; actions ;;;


	(:action explore_place
	 :agent (?a - agent)
	 :parameters (?loc - place)
	 :precondition (and
	 	(located ?a ?loc)
		)
     :effect (assign (explored ?loc) true)
    )

	(:action categorize_place
	 :agent (?a - agent)
	 :parameters (?loc - place)
	 :precondition (and
	 	(located ?a ?loc))
     :sense (place_category ?loc)
    )


	(:action categorize_room
	 :agent (?a - agent)
	 :parameters (?r - room)
	 :variables (?loc - place)
	 :precondition (and
	 	(located ?a ?loc)
		(contains ?r ?loc)
		)
     :sense (areaclass ?r)
    )


	(:action move
	 :agent (?a - agent)
	 :parameters (?to - place)
	 :variables (?from - place)
	 :precondition (and
		(located ?a ?from) 
		(connected ?from ?to)
		)
	 :effect (and
		(not (located ?a ?from))
		(located ?a ?to)
		(connected ?to ?from)   ;; now we assume we can also move back
	))
	

)
"""

dora_scenario = \
"""
(define (scenario cogxscen) (:domain cogx)

(:common

(:objects
	place1 place2 place3 - place
	pid1 pid2 pid3 - place_id
    kitchen - place_category
)

(:init
	(= (place_id place1) pid1)
	(= (place_id place2) pid2)
	(= (place_id place3) pid3)
	(connected place1 place2)
	(connected place2 place1)
	(connected place2 place3)
	(connected place3 place2)
	(located R2D2 place1)
)
)

(:world

(:init
	(= (place_category place3) default_place)
	(= (place_category place2) kitchen)
))

(:agent R2D2

(:objects
	place10 place11 - place
)

(:goal (and
	   (forall (?p - place) (= (explored ?p) true)))
)

(:metric minimize (total-cost))

)

)
"""

emptyworld = \
"""
(define (scenario cogxscen) (:domain cogx)

(:agent R2D2
(:goal (and
	   (forall (?p - place) (= (explored ?p) true))
))

)
)
"""

dora_multiagent = \
"""
(define (scenario cogxscen) (:domain cogx)

(:common

(:objects
	place1 place2 - place
	pid1 pid2 - place_id
        kitchen - place_category
        c3po - robot
        michael - agent
)

(:init
	(= (place_id place1) pid1)
	(= (place_id place2) pid2)
	(connected place1 place2)
	(connected place2 place1)
	(located R2D2 place1)
)
)

(:agent R2D2
(:goal (and
	   (forall (?p - place) (= (explored ?p) true))
))
)

(:agent C3PO
(:goal (and
	   (forall (?p - place) (= (explored ?p) false))
))
)

(:agent Michael
(:goal (and
	   (located r2d2 place2)
))
)


)
"""

unknownagent = \
"""
(define (scenario cogxscen) (:domain cogx)

(:common

(:objects
	place1 place2 place3 - place
))

(:agent R2D3
(:goal (and
	   (forall (?p - place) (= (explored ?p) true))
))
)

)
"""

unknownagent2 = \
"""
(define (scenario cogxscen) (:domain cogx)

(:common

(:objects
	place1 place2 place3 - place
))

(:agent R2D3
(:objects
	r2d3 - robot
)

(:goal (and
	   (forall (?p - place) (= (explored ?p) true))
))
)

)
"""

duplicateagent = \
"""
(define (scenario cogxscen) (:domain cogx)

(:common

(:objects
	place1 place2 place3 - place
))

(:agent r2d2
(:goal (and
	   (forall (?p - place) (= (explored ?p) true))
))
)

(:agent r2d2
(:goal (and
	   (forall (?p - place) (= (explored ?p) true))
))
)

)
"""


class ScenarioTest(unittest.TestCase):
    
    def testLoading(self):
        """Testing dora scenario"""
        
        p = Parser(dora_domain.split("\n"))
        dom = domain.Domain.parse(p.root)
        p = Parser(dora_scenario.split("\n"))
        scen = scenario.MapsimScenario.parse(p.root, dom)

        self.assert_("r2d2" in scen.agents)
        worldprob = scen.world
        agentprob = scen.agents["r2d2"]

        self.assertEqual(len(worldprob.init), 10)
        self.assertEqual(len(agentprob.init), 8)

        self.assertEqual(len(worldprob.objects), 7)
        self.assertEqual(len(agentprob.objects), 9)

        self.assertEqual(agentprob.optimization, "minimize")
        
    def testMultiagent(self):
        """Testing multi agent scenario"""
        
        p = Parser(dora_domain.split("\n"))
        dom = domain.Domain.parse(p.root)
        p = Parser(dora_multiagent.split("\n"))
        scen = scenario.MapsimScenario.parse(p.root, dom)

        self.assert_("r2d2" in scen.agents)
        self.assert_("c3po" in scen.agents)
        self.assert_("michael" in scen.agents)
        worldprob = scen.world
        r2prob = scen.agents["r2d2"]
        c3prob = scen.agents["c3po"]
        mprob = scen.agents["michael"]

        self.assertEqual(len(worldprob.init), 5)
        self.assertEqual(len(r2prob.init), 5)
        self.assertEqual(len(c3prob.init), 5)
        self.assertEqual(len(mprob.init), 5)

        self.assertEqual(len(worldprob.objects), 7)
        self.assertEqual(len(r2prob.objects), 7)
        self.assertEqual(len(c3prob.objects), 7)
        self.assertEqual(len(mprob.objects), 7)

    def testEmptyWorld(self):
        """Testing handling of empty world state"""
        
        p = Parser(dora_domain.split("\n"))
        dom = domain.Domain.parse(p.root)
        p = Parser(emptyworld.split("\n"))
        try:
            scen = scenario.MapsimScenario.parse(p.root, dom)
        except ParseError, e:
            self.assertEqual(e.token.string, "(")
            self.assertEqual(e.token.line,  2)
            return
        self.fail("Empty world state triggered no error")

    def testUnknownAgent(self):
        """Testing handling of unknown agents"""
        
        p = Parser(dora_domain.split("\n"))
        dom = domain.Domain.parse(p.root)
        p = Parser(unknownagent.split("\n"))
        try:
            scen = scenario.MapsimScenario.parse(p.root, dom)
        except ParseError, e:
            self.assertEqual(e.token.string, "r2d3")
            self.assertEqual(e.token.line,  10)
            return
        self.fail("Unknown agent triggered no error")

    def testUnknownAgent2(self):
        """Testing handling of unknown agents to the world state"""
        
        p = Parser(dora_domain.split("\n"))
        dom = domain.Domain.parse(p.root)
        p = Parser(unknownagent2.split("\n"))
        try:
            scen = scenario.MapsimScenario.parse(p.root, dom)
        except ParseError, e:
            self.assertEqual(e.token.string, "(")
            self.assertEqual(e.token.line,  2)
            return
        self.fail("Unknown agent triggered no error")

    def testDuplicateAgent(self):
        """Testing handling of duplicate agents"""
        
        p = Parser(dora_domain.split("\n"))
        dom = domain.Domain.parse(p.root)
        p = Parser(duplicateagent.split("\n"))
        try:
            scen = scenario.MapsimScenario.parse(p.root, dom)
        except ParseError, e:
            self.assertEqual(e.token.string, ":agent")
            self.assertEqual(e.token.line,  16)
            return
        self.fail("Duplicate agent triggered no error")
        
        
if __name__ == '__main__':
    unittest.main()    
        
