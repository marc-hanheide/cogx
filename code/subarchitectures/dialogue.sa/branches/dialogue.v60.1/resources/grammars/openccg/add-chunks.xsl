<?xml version="1.0"?>
<!-- 
Copyright (C) 2003-4 University of Edinburgh (Michael White)
$Revision: 1.1 $, $Date: 2005/06/07 08:10:25 $ 

This transformation adds LF chunks (to be realized separately) to the 
HLDS representations.
-->
<xsl:transform 
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform" 
  version="1.0"
  xmlns:xalan2="http://xml.apache.org/xslt">

  <xsl:output indent="yes" xalan2:indent-amount="2"/> 
  <xsl:strip-space elements="*"/>

  
  
  <!-- default: chunk all children when there is at least one relational child --> 
  <xsl:template match="*[diamond[nom]]">
    <xsl:call-template name="chunk-all"/>
  </xsl:template>

  
  <!-- override: don't chunk w/in scopal body -->
  <xsl:template match="diamond[@mode='Body']">
    <xsl:call-template name="copy"/>
  </xsl:template>


  <!-- or a root-level satop containing a mood feature -->
  <xsl:template match="lf/satop[diamond[@mode='mood']]" priority="1.0">
    <xsl:call-template name="copy"/>
  </xsl:template>

  
  <!-- or a tuple -->
  <xsl:template match="*[prop[@name='tup']]" priority="1.0">
    <xsl:call-template name="copy"/>
  </xsl:template>

  
  <!-- or under a First rel containing a tuple item (for gapping) -->
  <xsl:template match="diamond[@mode='First'and diamond[@mode='Item' and prop[@name='tup']]]" priority="1.0">
    <xsl:call-template name="copy"/>
  </xsl:template>
  

  
  <!-- chunk all children -->
  <xsl:template name="chunk-all">
    <xsl:copy>
      <xsl:apply-templates select="@*"/>
      <chunk>
        <xsl:apply-templates select="node()"/>
      </chunk>
    </xsl:copy>
  </xsl:template>
  
  
  
  <!-- Copy -->
  <xsl:template match="@*|node()">
    <xsl:copy>
      <xsl:apply-templates select="@*|node()"/>
    </xsl:copy>
  </xsl:template>

  <xsl:template name="copy">
    <xsl:copy>
      <xsl:apply-templates select="@*|node()"/>
    </xsl:copy>
  </xsl:template>

</xsl:transform>

