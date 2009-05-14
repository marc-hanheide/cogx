Trevor Benjamin, April 1st (ha ha) 2008

This file contains 
	1) information concerning the contents of this folder
    2) a general overview of the layout & organization of the grammar.
    3) instructions for adding words, or more substantial grammatical alterations
	4) instructions for compiling the grammar & testing it.
	
********** Folder Contents ****************

-The complete Dot-CCG grammar file:     moloko.ccg
-The compiled Open-CCG grammar files:   grammar.xml, rules.xml, etc.

!!!IMPORTANT: These are all generated files. Do not alter them directly. Changes will not be saved !!!

-The folder ccg-files containing:
		-all of the Dot-CCG component files forming the grammar (see below) 
		-a perl script merge.pl used to combine these .ccg files to create moloko.ccg
	

*********** Grammar Layout ***********

The Moloko Grammar, contained in /ccg-files is divided into 5 parts:
    
    1) The grammar signature (i.e. all of the syntactic categories, semantics, lexical families, hierarchies and rules)
	2) The dictionary (this contains the words and their assignment to lexical families, i.e. entries)
	3) A list of examples used in testing the grammar, and illustrating its features

	   
1) The Grammar Signature

   The grammar proper is divided into a number of different files of the form #_description-of-contents.ccg.
   
   The reason the files are numbered like this is because the Dot-CCG compiler is ordered, and 
   consequently any macro-definitions that are used must be defined prior to that.
   
   The first two files types-ontology.ccg and types-features.ccg contain hierarchies used in the grammar,
   and in the case of the ontology, for outside inferencing as well. The ontology is a hierarchy of semantic
   sorts, similar in spirit to the Upper Generalized Model. It contains subdivisions for the three
   main semantic 'objects', i.e., entities, events and modifiers. It also includes subdivisions for 
   discourse markers and other 'less' semantics-like (or ideational) meaning. The feature hierarchy contains
   both syntactic and semantic feature categories and a listing of their possible (hierarchically organiyed) 
   values. These definitions also function as 'macros' (in the old OpenCCG sense of the word), i.e., values that
   can be 'used' within the grammar & dictionary (i.e. lexically specified). 
      
	  For a syntactic example, it contains a feature category VFORM for verbal forms, including values like
      ing, base, past-particple, infinitive, etc). In the grammar, this is used, for example, to create the family
      v+verb-inf, i.e., the family of verbs which take a verbal compliment in infinitive form (ex. I want to go home)
      It is also used in dictionary entries to specify, e.g., that the past-partiple form of 'sing' is 'sung'.
   
      For a semantic example, it contains the feature <Aspect> with values perfect and imperfect. These values are attached
      to events via auxillary verb entries. 
  
   The remainder of the files in the Grammar Signature contain the syntactic and semantic categories, families, rules and
   dictionary entry creating macros. Syntactic and semantic categories are combinined to make lexical families, 
   like ditranstive verbs. They are also used in rules which are used for a variety of purposes in the grammar.
   Dictionary-entry creating macros are a new component of the grammar. They utilize the power of the Dot-CCG language 
   to massively simplify the task of adding new words to the grammar. 
         
		 For example, here are some examples of noun entries from dictionary-open.ccg (see below for more details)
	       noun-irr(man, men, person,)
           noun(box, thing,) 
		   name(GJ, person,)

   These grammar components have been divided among the files according to two general organizational principles.
   First, encapsulation, e.g. #_adj.ccg contains everything particular to adjectives, wo if you want to find the 
   families, rules, and dictionary macros for adjectives, see this file. (Note: but the adjective words themselves
   are in the dictionary files). Second, generality and efficiency: minimize overlap and put generalized components
   together, e.g., adjectives, prepositions and adverbs overlap a lot in their grammatical information (like
   semantics and their syntactic categories). Thus, this common info is contained in #_modifiers.ccg. 

   Within any these files, there is an order to the components. 
		
		1st: syntactic and semantic categories. 
		2nd: def-macros used for simplifying the building of lexical families
		3rd: the various families themselves
		4th: associated type-changing rules
		5th: dictionary-entry macros
		     
   The syntatic and semantic categories created via def-macros are typically those used frequently within
   its file, and those used externally. For example, the noun file contains a syntactic category 
   n() which corresponds to 'any old generic noun', i.e. a non-bound n . It can be further specified by
   adding feature values to its parameters, e.g. n(3rd s-sg). ENTITY() is the corresponding (both share index)
   semantic representation. Again, features & arguments can be further specified by adding them to the parameters.
   			 		 
2) The Dictionary

   The dictionary, i.e. the words in the grammar, are divided into 'closed' and 'open' class entries, 
   located in X_dictionary-open.ccg & X_dictionary-closed.ccg respectively. Each of these files
   contains a sorted listing of all the entries currently contained in the grammar. The vast majority 
   are simply 'calls' to (instances of) the various dictionary entry macros specified in the grammar 
   signiture files. 
   
		e.g.  noun(box, thing,)  
		      verb(give, giving, gave, given, m-class-1, action-non-motion,          tv dtv dtv-to ) 
			  pronoun(I,    1st, sg, I    , me   , my   , mine   , person,)
				 
   Some of the more irregular or singular words (specifically closed class function words) are simply
   build using the default Dot-CCG syntax, i.e.  word form: Family: {other-forms: args;}
   
	    e.g   word most : Most-adj: superlative;

3) The Testbed

   The testbe, contained in X_testbed.ccg, is a listing of sample sentences used to test and to 'showcase' 
   the grammar. Each line in the testbed consists of a sentence and the expected number of parses. This listing can
   be tested for both parsing and generation using the command ccg-test (see below)
   
   I have organized these test-sentences in an attempt to illustrate the capability of the grammar.
   See the file itself for more comments.
    
*********** Modifying the Grammar ***********

*********** Compiling & Testing the Grammar ***********

   Compiling the grammar actually involves two steps:
       1) Merging the .ccg files to create moloko.ccg using merge.pl
	   2) Converting moloko.ccg into the .xml files used by the OpenCCG compiler & realizer
   These two steps have been automated in the 