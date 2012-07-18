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


<!-- ==================================================================================== -->
<!-- BASE MODIFIER FOR EACH ENTITY TYPE												   -->
<!-- ====================================================================================  -->


  <xsl:variable name="T-GENERIC">
    <satop nomvar="T:entity"></satop>
  </xsl:variable>

  <xsl:variable name="T-PHYSICAL">
    <satop nomvar="T:physical"></satop>
  </xsl:variable>


<!-- ==================================================================================== -->
<!-- SEMANTICS: ENTITY MODIFYING														   -->
<!-- ====================================================================================  -->


<!-- GENERIC ENTITIES -->

<xsl:variable name="MOD-T.ARG">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($T-GENERIC)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$MODIFIER"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<xsl:variable name="MOD-T.PROPERTY">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($T-GENERIC)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$PROPERTY"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<xsl:variable name="MOD-T.PROPERTY-WITH-ARG">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($T-GENERIC)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$PROPERTY-WITH-ARG"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<xsl:variable name="MOD-T.COMPARISON-ENTITY">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($T-GENERIC)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$COMPARISON-ENTITY"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>   

<xsl:variable name="MOD-T.ACCOMPANIMENT">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($T-GENERIC)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$ACCOMPANIMENT"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>

<xsl:variable name="MOD-T.BENEFACTOR">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($T-GENERIC)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$M-BENEFACTOR"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>

<xsl:variable name="MOD-T.OWNER">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($T-GENERIC)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$OWNER"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>

<xsl:variable name="MOD-T.PURPOSE">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($T-GENERIC)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$PURPOSE"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>


<xsl:variable name="MOD-T.RESTR"> <!-- used in restrictive relative clauses -->
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($T-GENERIC)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$M-RESTR"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    



<!-- PHYSICAL ENTITIES -->

<xsl:variable name="MOD-T-PHYSICAL.ARG">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($T-PHYSICAL)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$MODIFIER"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>

<xsl:variable name="MOD-T-PHYSICAL.ACCOMPANIMENT">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($T-PHYSICAL)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$ACCOMPANIMENT"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>

<xsl:variable name="MOD-T.LOCATION-WITH-ANCHOR">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($T-GENERIC)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$LOCATION-WITH-ANCHOR"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable> 

<xsl:variable name="MOD-T.LOCATION">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($T-GENERIC)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$LOCATION"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable> 
 
<xsl:variable name="MOD-T-PHYSICAL.LOCATION-WITH-ANCHOR">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($T-PHYSICAL)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$LOCATION-WITH-ANCHOR"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable> 


</xsl:transform>