<?xml version="1.0"?>
<!-- 
Copyright (C) 2003 University of Edinburgh (Michael White)
$Revision: 1.1 $, $Date: 2005/06/07 08:10:26 $ 

This transformation converts a hybrid logic dependency semantics representation
to a node-rel graph.
-->
<xsl:transform 
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform" 
  version="1.0"
  xmlns:xalan2="http://xml.apache.org/xslt">

  <xsl:output indent="yes" xalan2:indent-amount="2"/> 
  <xsl:strip-space elements="*"/>

  
  <!-- convert satops to nodes --> 
  <xsl:template match="satop">
    <node id="{@nom}">
      <xsl:apply-templates/>
    </node>
  </xsl:template>

  <!-- add pred -->
  <xsl:template match="prop">
    <xsl:attribute name="pred"><xsl:value-of select="@name"/></xsl:attribute>
  </xsl:template>  
  
  <!-- convert sem features -->
  <xsl:template match="diamond[not(nom)]">
    <xsl:attribute name="{@mode}"><xsl:value-of select="prop/@name"/></xsl:attribute>
  </xsl:template>
  
  <!-- convert modal rels to rels with nodes -->
  <xsl:template match="diamond[nom]">
    <rel name="{@mode}">
      <node>
        <xsl:apply-templates/>
      </node>
    </rel>
  </xsl:template>
  
  <!-- convert initial nominals to id's --> 
  <xsl:template match="nom[following-sibling::*]">
    <xsl:attribute name="id"><xsl:value-of select="@name"/></xsl:attribute>
  </xsl:template>
  
  <!-- convert solo nominals to idref's --> 
  <xsl:template match="nom[not(following-sibling::*)]">
    <xsl:attribute name="idref"><xsl:value-of select="@name"/></xsl:attribute>
  </xsl:template>

  
  <!-- Copy -->
  <xsl:template match="@*|node()">
    <xsl:copy>
      <xsl:apply-templates select="@*|node()"/>
    </xsl:copy>
  </xsl:template>

</xsl:transform>

