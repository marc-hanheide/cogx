<?xml version="1.0"?>

<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">


 <!-- Noun Bundles -->
 
 <xsl:variable name="NUM.PERS">  
    <feat attr="num"><featvar name="NUM:num-vals"/></feat>
    <feat attr="pers"><featvar name="PERS:pers-vals"/></feat>
</xsl:variable>	
 <xsl:variable name="NUM.PERS.CASE">  
    <feat attr="num"><featvar name="NUM:num-vals"/></feat>
    <feat attr="pers"><featvar name="PERS:pers-vals"/></feat>
    <feat attr="case"><featvar name="CASE:case-vals"/></feat>
 </xsl:variable>
 <xsl:variable name="NUM.PERS.acc">  
    <feat attr="num"><featvar name="NUM:num-vals"/></feat>
    <feat attr="pers"><featvar name="PERS:pers-vals"/></feat>
	<feat attr="case" val="acc"/>
 </xsl:variable>
 <xsl:variable name="NUM.PERS.nom">  
    <feat attr="num"><featvar name="NUM:num-vals"/></feat>
    <feat attr="pers"><featvar name="PERS:pers-vals"/></feat>
    <feat attr="case" val="nom"/>
 </xsl:variable>

</xsl:transform>