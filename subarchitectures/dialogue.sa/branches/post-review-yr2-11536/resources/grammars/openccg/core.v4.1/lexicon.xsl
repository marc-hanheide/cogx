<?xml version="1.0"?>
<!--
Copyright (C) 2005 DFKI (Geert-Jan M. Kruijff gj@dfki.de)
-->

<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">

  <!-- ================================================ -->  
  <!-- INCLUDES                                         --> 
  <!-- ================================================ -->  

  <xsl:include href="templates.xsl"/>
  <xsl:include href="cats-sentence-vp.xsl"/>
  <xsl:include href="coord.xsl"/>
  <xsl:include href="adj.xsl"/>
  <xsl:include href="adv.xsl"/>
  <xsl:include href="det.xsl"/>
  <xsl:include href="noun.xsl"/>
  <xsl:include href="marker.xsl"/>
  <xsl:include href="preposition-and-oblique.xsl"/>

  <xsl:include href="other.xsl"/>
  <xsl:include href="mwe.xsl"/>   
  <xsl:include href="cue-and-discourse.xsl"/>   
  
  <xsl:include href="mod-cats.xsl"/>
  <xsl:include href="mod-roles.xsl"/>
  <xsl:include href="mod-modifier.xsl"/>
  <xsl:include href="mod-event-semantics.xsl"/>
  <xsl:include href="mod-entity-semantics.xsl"/>
  

  <xsl:include href="verb-aux.xsl"/>
  <xsl:include href="verb-copular.xsl"/>
  <xsl:include href="verb-other-families.xsl"/>
  <xsl:include href="verb-other-categories.xsl"/>
  <xsl:include href="verb-semantics.xsl"/> 
  <xsl:include href="verb-roles.xsl"/>


 
  <!-- ================================================ -->    
  <!-- ADD: 
       Each include file [X.xsl] defines a template 
       "add-X-families" that adds the families          -->
  <!-- ================================================ -->    
    
  <xsl:template name="add-core-families">
  
	<xsl:call-template name="add-adj-families"/>
    <xsl:call-template name="add-adv-families"/>
	<xsl:call-template name="add-coord-families"/>
	<xsl:call-template name="add-det-families"/>
	<xsl:call-template name="add-marker-families"/>
	<xsl:call-template name="add-other-families"/>
	<xsl:call-template name="add-mwe-families"/>
	<xsl:call-template name="add-mod-modifier-families"/>
    <xsl:call-template name="add-preposition-and-oblique-families"/>
    <xsl:call-template name="add-n-families"/>

	<xsl:call-template name="add-aux-families"/>
	<xsl:call-template name="add-verb-copular-families"/>
   	<xsl:call-template name="add-verb-other-families"/>

   	<xsl:call-template name="add-cue-and-discourse-families"/>
 
  </xsl:template>

</xsl:transform>
