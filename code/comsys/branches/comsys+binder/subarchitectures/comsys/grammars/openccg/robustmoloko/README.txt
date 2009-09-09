Trevor Benjamin, April 1st (ha ha) 2008

This file contains 
    1) information concerning the contents of this folder
    2) a general overview of the layout & organization of the grammar.
    3) instructions for adding words, or more substantial grammatical alterations
    4) instructions for compiling the grammar & testing it.
	
********** Folder Contents ****************

Generated Files
    !!!IMPORTANT: Do not alter them directly. Changes will not be saved !!!

-The complete Dot-CCG grammar file:     moloko.ccg
-The compiled Open-CCG grammar files:   grammar.xml, rules.xml, etc.

Non-Generated Files

-a batch file for compiling the grammar called build-moloko (see below)
    (this should be placed in your openccg bin directory)
-The folder ccg-files containing:
		-all of the Dot-CCG component files forming the grammar (see below) 
		-a perl script merge.pl used to combine these .ccg files to create moloko.ccg
	

*********** Grammar Layout ***********

The Moloko Grammar, contained in /ccg-files is divided into 3 parts:
    
     1) The grammar signature (i.e. all of the syntactic categories, semantics, lexical families, hierarchies and rules)
     2) The dictionary (this contains the words and their assignment to lexical families, i.e. entries)
     3) The testbed: a list of examples used in testing the grammar, and illustrating its features
	   
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
   build using the default Dot-CCG syntax, i.e.  word form: Family: {other forms: args;}
   
        e.g   word most : Most-adj: superlative;
		
   Furthermore, some 'higher level' dictionary macros are located within the dictionary files themselves.
   They combine a series of other 'base level' macros to create the appropriate lexical entries.
   For example, the macro for number entries (one, two, etc), takes the ordinal and cardinal forms  
   and then 'calls' a large number of other macros, like adjective (those two balls), context-n (those two),
   subset-selector (two of the balls) etc. to accountfor all of the 'uses' of numbers. 
   This eliminates redundany by collecting all of these entriesinto a meaningful conceptual category, 
   instead of having them spread throughout the grammar. 
      
3) The Testbed

   The testbed, contained in X_testbed.ccg, is a listing of sample sentences used to test and to 'showcase' 
   the grammar. Each line in the testbed consists of a sentence and the expected number of parses. This listing can
   be tested for both parsing and generation using the command ccg-test (see below)
   
   I have organized these test-sentences in an attempt to illustrate the capability of the grammar.
   See the file itself for more comments.
    
*********** Modifying the Grammar ***********

1) Adding New Words (i.e. dictionary entries) to the grammar

   The simplest and most common way of modifying the grammar is to add new words using the 
   current dictionary-entry building macros. In most cases, simply parroting or mimicing one
   of the currently existing entries will suffice. However, as discussed above, these macros 
   are defined throughout the grammar and for a full description of the parameters involved, 
   you'll have to look into these files.
   
   If none of the existing dictionary macros fit your exact needs, this does not necessarily
   mean that you underlying lexical families, rules and features cannot support what you want. 
   In some cases, you can overload one of the more general dictionary macros by adding extra 
   feature values in the argument list. 
   If this still won't work you may be able to combine these appropriately by resorting to 
   the Dot-CCG word entry syntax:
   
	word form: families (class, pred): {other forms: feature-values;}

2) Changing the Ontological Hierarchy
   
   The semantic sorts/classes/types attached to the semantic objects produced in the semantic
   representations used in the grammar are specified in X_types-ontology.ccg
   
   If you wish to modify this hierarchy by either collapsing, expanding, or altering its
   components and their relations, in some cases, this can be done by simply modifying this file. 
   In other cases, however, this will require further changes within other parts of the grammar.

   See X_types-ontology.ccg for specifics

3) Dependency Relation Names

   If you don't like the dependency relation names used in the grammar you will have to search for 
   appropriate semantic def-macroand change its form, e.g. The def-macro corresponding to <Actor>
   is called ACTOR() and, being an event role,it is located in verb.ccg. Going there and changing 
        def ACTOR()  { <Actor>(X) }    to   def ACTOR()  { <Arg1>(X) } 
   will perculate these changes throughout the whole grammar (hopefully!).
   If you would like to make more fined grained distinctions inside the grammar (i.e. turn into 
   a more frame-semantic like grammar) you will of course need to carry these changes 
   through into the various lexical families and rules. For example. 
        def ACTOR()  { <Actor>(X) }    to    def ACTOR()       { <Actor>(X)       }     
                                           & def EXPERIENCER() { <Experiencer>(X) }
   will require, for each verbal argument structure family, 2 forms.
   Of course, you could no doubt use def-macros to simplify this massively.
   
4) Other changes (features, new lexical families, rules, etc)

   I have tried to organized the grammar in such a way that it can easily be navigated and
   extended. Moreover, some of the grammar files include instructions on extending the 
   more 'open' elements of the grammar. Good Luck!
   
    
*********** Compiling & Testing the Grammar ***********

   Compiling the grammar actually consists of 2 steps:
   
      1) merging all of the .ccg files to create moloko.ccg  (using the merge.pl script)
      2) converting moloko.ccg into the OpenCCG .xml files   (using the Dot-CCG Parser ccg2xml)
  
   These have been combined into a single batch file called build-moloko. 
   This file should be moved to your openccg bin directory
   
   There are two ways of testing the grammar. These are identical to the old OpenCCG methods.
	  1) Command line parsing & generation using the tccg tool
	  2) Running through the testbed using the ccg-test tool
	  
	  