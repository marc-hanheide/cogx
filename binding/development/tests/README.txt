AUTOMATIC TEST GENERATION
-------------------------

In order to extend more easily the test suite for ComSys, an automatic
*test generation* script is now available.  Based on a set of XML files
describing the test parameters, a full test suite is automatically 
generated (XML configuration file for the test suite, CAST configuration
files, fake vision parameters, etc.).

To generate and run the test suite, use the following script:
	./runComSysTests.sh 


TEST SUITE SPECIFICATION
-------------------------

1) the file "ComSysMetaConfig.xml" contains the meta-configuration for
the test suite, i.e. the different *test types* to be performed and their
associated parameters.

Most importantly, it specifies:
- a XML file describing all the parameters for each test (see below);
- as well as a *skeleton* for the individual CAST configuration files.

2) For each test test type, there is a XML file describing all the
need parameters. These parameters are specific to each test type:

Here are the requirements for each test type:

* Parsing test:
	a) an utterance ;
	b) a set of expected logical forms.

* Packing test:
	a) an utterance.

* Dialogue move test:
	a) an utterance ;
	b) an expected dialogue move, or "none".

* Discourse referent binding test:
	a) a discourse (set of utterances)
	b) the expected referent binding (a referring expression and its
	   associated referent, if any)

* Visual grounding test:
	a) a NP expression;
	b) a set of objects describing the visual scene;
	c) the expected grounding for the NP expression.

* Text-to-Speech test:
	a) an utterance to synthesise.

* Event structure interpretation:
	a) an utterance
	b) an event nucleus, defined by the event type and the state type.

* SDRS structure:
	a) a discourse (set of utterances)

* Ascription proxy factories:
	a) an utterance
	b) number of binding unions

