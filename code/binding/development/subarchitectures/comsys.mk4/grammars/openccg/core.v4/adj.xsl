<?xml version="1.0"?>
<!--
Copyright (C) 2005 DFKI GmbH (Geert-Jan M. Kruijff gj@dfki.de)


UNARY RULE TURNS BASIC CAT INTO n/n 

MACROS ADD DEGREE INFORMATION:  syntactic atr: aform
                                semantic :<degree>  


-->

<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">


<!-- ==================================================================================== -->
<!-- CATEGORY DEFINITIONS                                                                 -->
<!-- ==================================================================================== -->


      
<!--  Adj cat

	The basic type for adjectives is as a basic-cat. To get the combinatorially  complex
	cat n/n, a unary rule is used (see unary-rule.xsl)  
	This is necessary to handle adj modifiers like "really"

  -->	

 <xsl:variable name="adj.basic-cat">
	<atomcat type="adj">
      <fs id="25">
        <feat attr="index"><lf><nomvar name="M"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>
 
 <xsl:variable name="adj.basic-cat.comparative">
	<atomcat type="adj">
      <fs id="25">
        <feat attr="index"><lf><nomvar name="M"/></lf></feat>
		<feat attr="aform" val="comparative"/>
      </fs>
    </atomcat>
  </xsl:variable>

<!--  used for things like "proud of", "tired of", etc. 
	  should be moved to Mod-Roles 
-->
 
 <xsl:variable name="adj-of-np">
    <complexcat>
      <xsl:copy-of select="$adj.basic-cat"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$of.mod-arg"/> 
    </complexcat>
  </xsl:variable>

 <xsl:variable name="adj-with-np">
    <complexcat>
      <xsl:copy-of select="$adj.basic-cat"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$with.mod-arg"/> 
    </complexcat>
  </xsl:variable>



<!-- allowed to -->

 <xsl:variable name="adj.inf-verb">
    <complexcat>
      <xsl:copy-of select="$adj.basic-cat"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$vp.mod-arg.inf"/> 
    </complexcat>
  </xsl:variable>


 
 <!-- these are the complex n/n forms 
	  they may be used externally -->
 
 <xsl:variable name="adj.mod-n">
    <complexcat>
      <xsl:copy-of select="$n.from-generic"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$n.generic"/>	
    </complexcat>
  </xsl:variable>

  


<!-- ==================================================================================== -->
<!-- LEXICAL-MEANING DEFINITIONS                                                          -->
<!-- ==================================================================================== -->
 

  <xsl:variable name="PREDICATIVE">
    <lf>
       <satop nomvar="M">
	     <prop name="[*DEFAULT*]"/>
	   </satop>
    </lf>
  </xsl:variable>
 
  
  <xsl:variable name="PREDICATIVE.ARG">
    <lf>
       <satop nomvar="M">
	     <prop name="[*DEFAULT*]"/>
		 <xsl:copy-of select="$ARG"/> 
	   </satop>
    </lf>
  </xsl:variable>

  <xsl:variable name="PREDICATIVE.ECOMP">
    <lf>
       <satop nomvar="M">
	     <prop name="[*DEFAULT*]"/>
		 <xsl:copy-of select="$M-ECOMP"/> 
	   </satop>
    </lf>
  </xsl:variable>
 
	
<!-- ==================================================================================== -->
<!-- LEXICAL-FAMILY DEFINITIONS                                                           -->
<!-- ==================================================================================== -->

 <xsl:template name="add-adj-families">

  <!-- standard adjective family -->
	
  <family name="adj.property" pos="ADJ" closed="true">
     <entry name="predicative">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adj.basic-cat)/*"/>
        <xsl:with-param name="ext" select="$PREDICATIVE"/>
      </xsl:call-template>
    </entry>
  </family> 
  

 <!-- proud of, tired of, etc-->
  
 <family name="adj.of-np.property" pos="ADJ" closed="true">
    <entry name="predicative">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adj-of-np)/*"/>
        <xsl:with-param name="ext" select="$PREDICATIVE.ARG"/>
      </xsl:call-template>
    </entry>
 </family> 
 
 <family name="adj.with-np.property" pos="ADJ" closed="true">
    <entry name="predicative">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adj-with-np)/*"/>
        <xsl:with-param name="ext" select="$PREDICATIVE.ARG"/>
      </xsl:call-template>
    </entry>
 </family> 


 <!-- "they are allowed to play in the living room" , etc-->
  
 <family name="adj.to-verb.property" pos="ADJ" closed="true">
    <entry name="predicative">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adj.inf-verb)/*"/>
        <xsl:with-param name="ext" select="$PREDICATIVE.ECOMP"/>
      </xsl:call-template>
    </entry>
 </family> 
 
  
  
  </xsl:template>


</xsl:transform>