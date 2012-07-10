<?xml version="1.0"?>
<!--
Copyright (C) 2005 DFKI GmbH (Geert-Jan M. Kruijff, gj@dfki.de) 

See ../core/lexicon.xsl for comments concerning the basic categories,
LF structures, and families.

no log information provided

-->
<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  xmlns:redirect="org.apache.xalan.lib.Redirect"
  extension-element-prefixes="redirect"
  exclude-result-prefixes="xalan xalan2">

  <!-- ================================================ -->    
  <!-- IMPORTS, OUTPUT OPTIONS                          -->
  <!-- ================================================ -->    

  <xsl:import href="../test-core/lexicon.xsl"/>
  <xsl:output indent="yes" xalan2:indent-amount="2"/>
  <xsl:strip-space elements="*"/>

  <!-- ================================================ -->    
  <!-- GLOBAL GRAMMAR TEMPLATE ENVIRONMENT              -->
  <!-- ================================================ -->    

  <xsl:template match="/">
  <ccg-lexicon name="moloko"
    xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="../lexicon.xsd"
  >

  <!-- ================================================ -->    
  <!-- ADD THE FAMILIES DEFINED IN THE CORE MODULE      -->
  <!-- ================================================ -->    

  <xsl:call-template name="add-core-families"/>

  <!-- ================================================ -->    
  <!-- AND THOSE IN DEFINED IN THIS MODULE              -->
  <!-- ================================================ -->    

  </ccg-lexicon>


  </xsl:template>

</xsl:transform>