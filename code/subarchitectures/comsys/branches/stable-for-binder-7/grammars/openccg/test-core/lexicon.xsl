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
  <xsl:include href="feature-bundles.xsl"/>
  <xsl:include href="mod-roles and sem.xsl"/>
  
  <xsl:include href="noun.xsl"/>
  <xsl:include href="prep.xsl"/> 
  <xsl:include href="adj.xsl"/>
  <xsl:include href="det.xsl"/>

  <xsl:include href="verb-roles.xsl"/>
  <xsl:include href="verb-cats.xsl"/>
  <xsl:include href="verb-semantics.xsl"/>
  <xsl:include href="verb-families.xsl"/>
   
  <!-- ================================================ -->    
  <!-- ADD: 
       Each include file [X.xsl] defines a template 
       "add-X-families" that adds the families          -->
  <!-- ================================================ -->    
    
  <xsl:template name="add-core-families">
  
    <xsl:call-template name="add-noun-families"/>
    <xsl:call-template name="add-prep-families"/> 
    <xsl:call-template name="add-adj-families"/> 
    <xsl:call-template name="add-det-families"/>
    <xsl:call-template name="add-verb-families"/>  
	 
  </xsl:template>

</xsl:transform>
