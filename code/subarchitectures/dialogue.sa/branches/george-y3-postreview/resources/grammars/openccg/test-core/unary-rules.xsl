<?xml version="1.0"?>

<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">

  <xsl:import href="../test-core/lexicon.xsl"/>  <!-- allows access to xsl:variables -->
  
<xsl:template name="add-unary-rules">

<typechanging name="prep-s"> 
	<xsl:copy-of select="$comp-type.prep-s"/>
</typechanging>

<typechanging name="prep-n"> 
	<xsl:copy-of select="$comp-type.prep-n"/>
</typechanging>

<typechanging name="adj"> 
	<xsl:copy-of select="$comp-type.adj"/>
</typechanging>

<typechanging name="det"> 
	<xsl:copy-of select="$comp-type.det"/>
</typechanging>


</xsl:template>
</xsl:transform>

