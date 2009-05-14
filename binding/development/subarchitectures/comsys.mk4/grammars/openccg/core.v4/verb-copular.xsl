<?xml version="1.0"?>
<!--
Copyright (C) 2005 DFKI GmbH (Geert-Jan M. Kruijff gj@dfki.de)

This file contains the cats, semantics and families of those words (and constructions)
which don't have their own files (yet?)

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


<!-- Bound wh-np argument used by copular verbs for quesions --> 

 <xsl:variable name="whnp-subj-pos">
    <atomcat type="np">
      <fs id="2">
	    <feat attr="nform"><featvar name="WH:WHNP"/></feat> 
        <feat attr="index"><lf><nomvar name="X"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

 <!-- ================================================ -->  
 <!-- VERB with ADJECTIVE COMP	(COPULAR)				-->
 <!-- ================================================ -->  


 <!-- "john is a boy" & "is john a boy" & "who/what is a ball"-->

 <xsl:variable name="copular-np">
     <xsl:copy-of select="$verb.obj"/>
 </xsl:variable>
 
 <xsl:variable name="copular-np.NO-SUBJ">
     <xsl:copy-of select="$verb.obj.NO-SUBJ"/>
 </xsl:variable>


<xsl:variable name="copular-np.inverted">
    <complexcat> 
	  <xsl:copy-of select="$s.main"/> 
	    <slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$np.obj"/> 
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$np.subj"/> 
	</complexcat>  
  </xsl:variable>

<xsl:variable name="copular-np.whnp">
    <complexcat> 
	  <xsl:copy-of select="$s.main"/> 
	    <slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$np.obj"/> 
      <slash dir="\" mode="&gt;"/>
      <xsl:copy-of select="$whnp-subj-pos"/> 
	</complexcat>  
  </xsl:variable>


<!-- "john is big" & "is john big" "who/what is big" -->
	 
  <xsl:variable name="copular-adj">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$adj.mod-n"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  
  <xsl:variable name="copular-adj.NO-SUBJ">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$adj.mod-n"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

 <xsl:variable name="copular-adj.inverted">
    <complexcat> 
	  <xsl:copy-of select="$s.main"/> 
	    <slash dir="/" mode="*"/>
		<xsl:copy-of select="$adj.mod-n"/> 
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$np.subj"/> 
	</complexcat>  
  </xsl:variable>

 <xsl:variable name="copular-adj.whnp">
    <complexcat> 
	  <xsl:copy-of select="$s.main"/> 
	    <slash dir="/" mode="*"/>
		<xsl:copy-of select="$adj.mod-n"/> 
      <slash dir="\" mode="&gt;"/>
      <xsl:copy-of select="$whnp-subj-pos"/> 
	</complexcat>  
  </xsl:variable>

<!-- "John is in the room"  "is john in the room " "where is john"
	PLUS - location + action as in "I am in my office working" or "Is he in his room sleeping" -->

  <xsl:variable name="copular-pp">
     <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
        <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$prep.mod-n.saturated"/>
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

 <!-- 
 <xsl:variable name="copular-pp.progr-verb">
     <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
        <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$vp.vcomp.subj-controlled.progr"/>
	    <slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$prep.mod-n.saturated"/>
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable> 
  -->
     
  <xsl:variable name="copular-pp.NO-SUBJ">
     <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$prep.mod-n.saturated"/>
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

 <xsl:variable name="copular-pp.inverted">
    <complexcat> 
	  <xsl:copy-of select="$s.main"/> 
	    <slash dir="/" mode="*"/>
		<xsl:copy-of select="$prep.mod-n.saturated"/> 
      <slash dir="/" mode="*"/>
      <xsl:copy-of select="$np.subj"/> 
	</complexcat>  
  </xsl:variable>

 <xsl:variable name="copular-pp.whnp">
    <complexcat> 
	  <xsl:copy-of select="$s.main"/> 
	    <slash dir="/" mode="*"/>
		<xsl:copy-of select="$prep.mod-n.saturated"/> 
      <slash dir="\" mode="*"/>
      <xsl:copy-of select="$whnp-subj-pos"/> 
	</complexcat>  
  </xsl:variable>


<!-- ==================================================================================== -->
<!-- SEMANTICS                                                                            -->
<!-- ==================================================================================== -->

<xsl:variable name="COP.RESTR.SCOPE">
  <lf>
    <satop nomvar="E">
	    <prop name="[*DEFAULT*]"/>
		<diamond mode="Restr"><nomvar name="X"/></diamond>
		<diamond mode="Scope"><nomvar name="Y"/></diamond>
    </satop>
  </lf> 
 </xsl:variable>  

<xsl:variable name="COP.RESTR.SCOPE2">
  <lf>
    <satop nomvar="E">
	    <prop name="[*DEFAULT*]"/>
		<diamond mode="Restr"><nomvar name="X"/></diamond>
		<diamond mode="Scope"><nomvar name="T"/><prop name="restricted-entity"/></diamond>
    </satop>
  </lf> 
 </xsl:variable>  

<!-- <prop name="restricted-entity"/>
<xsl:variable name="COP.RESTR.SCOPE-WITH-ECOMP">
  <lf>
    <satop nomvar="E">
	    <prop name="[*DEFAULT*]"/>
		<diamond mode="Restr"><nomvar name="X"/></diamond>
		<diamond mode="Scope">
			<nomvar name="T"/><prop name="restricted-entity"/>
			<xsl:copy-of select="$VCOMP"/> 
		</diamond>
    </satop>
  </lf> 
 </xsl:variable> -->


<!-- ==================================================================================== -->
<!-- FAMILIES                                                                              -->
<!-- ==================================================================================== -->

<xsl:template name="add-verb-copular-families">

  <family name="copular" pos="V" closed="true">
    <entry name="adj">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($copular-adj)/*"/>
        <xsl:with-param name="ext" select="$COP.RESTR.SCOPE2"/>
      </xsl:call-template>
    </entry>
	<entry name="pp">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($copular-pp)/*"/>
        <xsl:with-param name="ext" select="$COP.RESTR.SCOPE2"/>
      </xsl:call-template>
    </entry>
	<!-- <entry name="pp-progrcomp">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($copular-pp.progr-verb)/*"/>
        <xsl:with-param name="ext" select="$COP.RESTR.SCOPE-WITH-ECOMP"/>
      </xsl:call-template>
    </entry> -->
	<entry name="np">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($copular-np)/*"/>
        <xsl:with-param name="ext" select="$COP.RESTR.SCOPE"/>
      </xsl:call-template>
    </entry>
  </family> 
  
  <family name="copular.NO-SUBJ" pos="V" closed="true">
    <entry name="adj">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($copular-adj.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$COP.RESTR.SCOPE2"/>
      </xsl:call-template>
    </entry>
	<entry name="pp">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($copular-pp.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$COP.RESTR.SCOPE2"/>
      </xsl:call-template>
    </entry>
	<entry name="np">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($copular-np.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$COP.RESTR.SCOPE"/>
      </xsl:call-template>
    </entry>
  </family> 

  
  <family name="copular.inverted" pos="V" closed="true">
    <entry name="adj">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($copular-adj.inverted)/*"/>
        <xsl:with-param name="ext" select="$COP.RESTR.SCOPE2"/>
      </xsl:call-template>
    </entry>
	 <entry name="pp">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($copular-pp.inverted)/*"/>
        <xsl:with-param name="ext" select="$COP.RESTR.SCOPE2"/>
      </xsl:call-template>
    </entry>
	<entry name="np">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($copular-np.inverted)/*"/>
        <xsl:with-param name="ext" select="$COP.RESTR.SCOPE"/>
      </xsl:call-template>
    </entry>
  </family> 

 <family name="copular.fronted-question" pos="V" closed="true">
    <entry name="adj">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($copular-adj.whnp)/*"/>
        <xsl:with-param name="ext" select="$COP.RESTR.SCOPE2"/>
      </xsl:call-template>
    </entry>
	<entry name="pp">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($copular-pp.whnp)/*"/>
        <xsl:with-param name="ext" select="$COP.RESTR.SCOPE2"/>
      </xsl:call-template>
    </entry>
	<entry name="np">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($copular-np.whnp)/*"/>
        <xsl:with-param name="ext" select="$COP.RESTR.SCOPE"/>
      </xsl:call-template>
    </entry>
  </family> 


    
</xsl:template>

</xsl:transform>

