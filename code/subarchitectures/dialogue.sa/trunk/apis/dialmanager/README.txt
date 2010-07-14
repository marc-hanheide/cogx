

************************************
* LIBRARIES FOR DIALOGUE MANAGEMENT
************************************


This branch contains development code for the dialogue manager used in various DFKI projects (CogX, ALIZ-E).  

At the moment, the dialogue manager is based on a policy represented by a finite-state machine, associated with the
definitions of the dialogue actions (nodes) and observations (edges).

A dialogue policy is typically constructed with 3 complementary files:
- a file for the policy itself, describing the structure of the interaction
- a file for the observations, i.e. the edges of the policy
- a file for the actions, i.e. the nodes of the policy.

For an illustrative example, cf. the files [policyExampleDora.txt, actionsDora.txt, observationsDora.txt].


INSTALLATION:
	- make sure that your Java classpath include the following dependencies:
		Ice.jar
		ant-ice.jar
	- simply type "ant" on the command line to compile the code	
	- to ensure the libraries work properly, you can run the test suite: "ant test"


CONTACT PERSON:
	Pierre Lison (plison@dfki.de)

=========================
==== DIALOGUE POLICY ====
=========================

The finite state machine is specified in the standard AT&T format:

	state1 state2 edge1
	state2 state3 edge2
	...
	finalstate1
	finalstate2

By convention, the first state specified in the FSA is the starting state, whereas the last lines represent the final states.



============================
======= OBSERVATIONS =======
============================


The observations are encoded the following format:

	observationSymbol = observation (optional probability range)

where the observation can be either:
- a shallow observation, written in double quotes: "hi robow how are you doing"
- an (attributed) intention observation, written as I[intentional content]
- an event observation, written as E[event content]

The content for both intentions and events can be arbitrary formulae in propositional modal logic.  In addition, two special characters 
can be entered: "?", which represents an unknown value, and "*", which represents an underspecified value.



============================
========= ACTIONS ==========
============================

The actions are encoded in the following format:

	actionSymbol = action
	
where the action can be either:
- a shallow action, written in double quotes: "I'm fine, thank you"
- a (private) intentional action, written as I[intentional content]

