<?xml version="1.0"?>
<!-- 
Copyright (C) 2003 University of Edinburgh (Michael White)
$Revision: 1.1 $, $Date: 2005/06/07 08:10:27 $ 

This transformation converts a node-rel graph to a hybrid logic 
dependency semantics representation.
-->
<xsl:transform 
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform" 
  version="1.0"
  xmlns:xalan2="http://xml.apache.org/xslt">

  <xsl:output indent="yes" xalan2:indent-amount="2"/> 
  <xsl:strip-space elements="*"/>

  
  <!-- convert root nodes to satops --> 
  <xsl:template match="lf/node | node/node">
    <satop nom="{@id}">
      <!-- add pred -->
      <prop name="{@pred}"/>
      <!-- convert sem features -->
      <xsl:call-template name="convert-feats"/>
      <!-- do rest -->
      <xsl:apply-templates/>
    </satop>
  </xsl:template>
  
  <!-- convert dependent nodes with initial nominals --> 
  <xsl:template match="rel/node[@id]">
    <nom name="{@id}"/>
    <!-- add pred -->
    <prop name="{@pred}"/>
    <!-- convert sem features -->
    <xsl:call-template name="convert-feats"/>
    <!-- do rest -->
    <xsl:apply-templates/>
  </xsl:template>
  
  <!-- convert dependent node references to nominals --> 
  <xsl:template match="rel/node[@idref]">
    <nom name="{@idref}"/>
  </xsl:template>
  
  <!-- convert sem features of current node -->
  <xsl:template name="convert-feats">
    <xsl:for-each select="@*[name(.) != 'id' and name(.) != 'pred']">
      <diamond mode="{name(.)}"><prop name="{.}"/></diamond>
    </xsl:for-each>
  </xsl:template>
  
  <!-- convert rels -->
  <xsl:template match="rel">
    <diamond mode="{@name}">
      <xsl:apply-templates/>
    </diamond>
  </xsl:template>
  
  
  <!-- Copy -->
  <xsl:template match="@*|node()">
    <xsl:copy>
      <xsl:apply-templates select="@*|node()"/>
    </xsl:copy>
  </xsl:template>

</xsl:transform>

