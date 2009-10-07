<?xml version="1.0"?>
<!--
Copyright (C) 2005 DFKI GmbH (Geert-Jan M. Kruijff gj@dfki.de)

This file contains the cats and families for slots for Mutli-word-expressions

About MWE categories and families themselves:

1) they are contained in the appropriate files
For example, "take_a_look", as in "take_a_look at the ball" is in 
verb-other-categories.xsl


2) they are of the form:
    head_W_W_W..
where head is the normal category or family for the result type, and for each
additional word in the MWE a _W is added

for example, "take_a_look" has family
		tv.at-obj_W_W
which means that the head (take) is followed by 2 words (a, look)

Here is a sample MWE category, note that it is a simple case of extending the head category

<xsl:variable name="verb.tv.at-obj.ind_W_W">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb.tv.at-obj.ind)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="*"/>
        <xsl:copy-of select="$word.2"/>
	    <slash dir="/" mode="*"/>
	    <xsl:copy-of select="$word.1"/>   
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

NOTE: The addition of MWE categories could in theory be automated


3) in the dictionary entry, they must choose their slots using macros of the form:
		@mwe#-wform
   So for our example, @mwe1-a @mwe2-look

Here is a sample MWE dictionary entry:

<entry pred="look" stem="take_a_look-ind" pos="V" class="perception">
   <word form="take" macros="@nonfin @mwe1-a @mwe2-look"/>
   <word form="take" macros="@finite @pres @ind @mwe1-a @mwe2-look"/>
   <word form="took" macros="@finite @past @ind @mwe1-a @mwe2-look"/>
   <member-of family="tv.at-obj.ind_W_W"/>
</entry>  

		
All words to be used as slots in MWE (e.g. 'a' and 'look' above)
need dictionary entries of POS WORD, where they will automatically be given
the family mwe-arg (see below) as it is "open."
Also, the corresponding w-form macro must be added  each slot
position in which it appears (word1, word2, word3, etc)

Here is a sample: 

<entry pred="look" stem="look"  pos="WORD" class="??????">
  <word form="look"/>
</entry>
<macro name="@mwe1-look"> <fs id="51"><feat attr="wform" val="look"/> </fs> </macro>
<macro name="@mwe2-look"> <fs id="52"><feat attr="wform" val="look"/> </fs> </macro>
<macro name="@mwe3-look"> <fs id="53"><feat attr="wform" val="look"/> </fs> </macro>



THIS IS DONE AUTOMATICALLY BY ADDED THE MWE-ARGS (ie. WORDS) TO DICT.LIST

-->

<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">

 
<!-- ================================================ -->  
<!-- BASIC CATEGORY DEFINITIONS                       -->
<!-- ================================================ -->   
   
<!-- Generic Category -->

  <xsl:variable name="word.generic">  
	<atomcat type="word">
      <fs id="50">
	    <feat attr="wform" val="[*DEFAULT*]"/>
      </fs>
    </atomcat>
  </xsl:variable>

<!-- Slot Categories for MWE heads to choose 
     The actual wforms are specified via macros -->

  <xsl:variable name="word.1">  
	<atomcat type="word">
      <fs id="51">
  	  <!-- <feat attr="wform"><featvar name="WORD1:WORD"/></feat> -->
	  </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="word.2">  
	<atomcat type="word">
      <fs id="52">
  	  <!-- <feat attr="wform"><featvar name="WORD2:WORD"/></feat> -->
	  </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="word.3">  
	<atomcat type="word">
      <fs id="53">
  	  <!-- <feat attr="wform"><featvar name="WORD3:WORD"/></feat> -->
	  </fs>
    </atomcat>
  </xsl:variable>
 

 
  <!-- ================================================ -->  
  <!-- FAMILY         DEFINITIONS                       -->
  <!-- ================================================ -->  


  <xsl:template name="add-mwe-families">

   <family name="mwe-arg" pos="WORD" indexRel= "*NoSem*">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($word.generic)/*"/>
      </xsl:call-template>
    </entry>
  </family>
	 
  
  </xsl:template>
</xsl:transform>