<?xml version="1.0"?>
<!--
Copyright (C) 2005 DFKI GmbH (Geert-Jan M. Kruijff gj@dfki.de)


-->

<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">
 
 
  <!-- ================================================ -->  
  <!-- CATEGORY       DEFINITIONS                       -->
  <!-- ================================================ -->  



<!-- ================================================ -->  
<!-- INFINITIVE TO                                    -->
<!-- ================================================ -->  
 
  <xsl:variable name="non-fin-vp.becomes.inf-vp">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($vp.from-generic.inf)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
		<complexcat>	 
	      <xsl:copy-of select="$s.generic.non-fin"/>
		  <slash dir="\" mode="&lt;"/>
          <xsl:copy-of select="$np.subj"/>
		</complexcat> 
      </xsl:with-param>	   
   </xsl:call-template>
 </xsl:variable>

<!-- ================================================ -->  
<!-- SENTENCE MARKING (that)                          -->
<!-- ================================================ -->  

<!-- no promiscous, i.e. dont mark. Must add, also for if, whether etc in "I want to know whether"-->

 <xsl:variable name="s-marker">
    <complexcat>
        <xsl:copy-of select="$s.generic"/>	
		<slash dir="/" mode="&gt;"/>
	    <xsl:copy-of select="$s.generic"/>
    </complexcat> 
  </xsl:variable>
 
  <xsl:variable name="MOD-E.MARKER">
  <lf>
    <satop nomvar="E0">
		<diamond mode="Marker">
	      <nomvar name="MARKER"/>
	      <prop name="[*DEFAULT*]"/> 
        </diamond>
	</satop>
  </lf>
  </xsl:variable>



<!-- ================================================ -->
<!--    NEBEN SATZ- introduce a dependent clause
	    Example:  I will be happy "if" you kiss me  -->
<!-- ================================================ -->  

 <xsl:variable name="dep-clause-marker.backward">
   <complexcat>
	 <xsl:copy-of select="$s.generic.fin"/>
	 <slash dir="/" mode="&gt;"/>
     <xsl:copy-of select="$s.generic.fin"/>
 	 <slash dir="/" mode="&gt;"/>
     <xsl:copy-of select="$s.mod-arg.fin"/>
   </complexcat>
 </xsl:variable>

  <xsl:variable name="dep-clause-marker.forward">
   <complexcat>
	 <xsl:copy-of select="$s.generic.fin"/>
	 <slash dir="/" mode="&gt;"/>
     <xsl:copy-of select="$s.mod-arg.fin"/>
     <slash dir="\" mode="&lt;"/>
     <xsl:copy-of select="$s.generic.fin"/>
   </complexcat>
 </xsl:variable>

 <xsl:variable name="dep-clause-marker-with-non-fin-vp.backward">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($vp.generic)/*"/>
      <xsl:with-param name="ext">	 
	    <slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$vp.mod-arg.subj-controlled.non-fin"/>
        <slash dir="\" mode="&lt;"/>
	    <xsl:copy-of select="$vp.generic"/>
     </xsl:with-param>	   
   </xsl:call-template>
 </xsl:variable>


  <!-- ================================================ -->  
  <!-- LEXICAL FAMILY DEFINITIONS                       -->
  <!-- ================================================ -->  

  <xsl:template name="add-marker-families">

  <family name="infinitive-to" pos="MARKER" closed="true" indexRel= "*NoSem*">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($non-fin-vp.becomes.inf-vp)/*"/>
      </xsl:call-template>
     </entry>
  </family>

 <family name="sentence-marker" pos="MARKER" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($s-marker)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.MARKER"/>
      </xsl:call-template>
     </entry>
  </family>

  <family name="dep-clause-marker.purpose" pos="MARKER" indexRel="Purpose" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($dep-clause-marker-with-non-fin-vp.backward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.PURPOSE"/>
	  </xsl:call-template>
     </entry>
  </family>

  <family name="dep-clause-marker.condition" pos="MARKER" indexRel="Condition" closed="true">
     <entry name="backward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($dep-clause-marker.backward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.CONDITION"/>
	  </xsl:call-template>
   </entry>
   <entry name="forward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($dep-clause-marker.forward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.CONDITION"/>
	  </xsl:call-template>
   </entry>
  </family>

  <family name="dep-clause-marker.cause" pos="MARKER" indexRel="Cause" closed="true">
     <entry name="backward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($dep-clause-marker.backward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.CAUSE"/>
	  </xsl:call-template>
   </entry>
   <entry name="forward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($dep-clause-marker.forward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.CAUSE"/>
	  </xsl:call-template>
   </entry>
  </family>

  <family name="dep-clause-marker.result" pos="MARKER" indexRel="Result" closed="true">
     <entry name="backward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($dep-clause-marker.backward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.RESULT"/>
	  </xsl:call-template>
   </entry>
   <entry name="forward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($dep-clause-marker.forward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.RESULT"/>
	  </xsl:call-template>
   </entry>
  </family>
 
   <family name="dep-clause-marker.time" pos="MARKER" indexRel="Time" closed="true">
     <entry name="backward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($dep-clause-marker.backward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.TIME"/>
	  </xsl:call-template>
   </entry>
   <entry name="forward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($dep-clause-marker.forward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.TIME"/>
	  </xsl:call-template>
   </entry>
  </family>
 
    </xsl:template>
</xsl:transform>
