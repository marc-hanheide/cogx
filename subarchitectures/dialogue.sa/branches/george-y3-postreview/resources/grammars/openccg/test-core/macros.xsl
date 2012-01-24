<?xml version="1.0"?>
<!--
Copyright (C) 2005 DFKI (Geert-Jan M. Kruijff, gj@dfki.de) 

-->
<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">

  <!-- ================================================ -->  
  <!-- Lexical macros                                   -->
  <!-- ================================================ -->  

  <xsl:template name="add-macros">

  
  <!--  Number, syntactic and semantic  -->
    
  <macro name="@num.sg">
    <fs id="30" attr="num" val="sg"/>
    <lf>
      <satop nomvar="T">
        <diamond mode="num"><prop name="sg"/></diamond>
      </satop>
    </lf>
  </macro>
  <macro name="@num.pl">
    <fs id="30" attr="num" val="pl"/>
    <lf>
      <satop nomvar="T">
        <diamond mode="num"><prop name="pl"/></diamond>
      </satop>
    </lf>
  </macro>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- Macros: case.30									-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <macro name="@nom">
	 <fs id="30" attr="case" val="nom"/>
  </macro>
  <macro name="@acc">
	 <fs id="30" attr="case" val="acc"/>
  </macro>

  </xsl:template>

</xsl:transform>