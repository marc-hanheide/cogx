<?xml version="1.0"?>

<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">

<!-- Basic meaning -->

<xsl:variable name="E-BASE">
  <satop nomvar="E">
      <prop name="[*DEFAULT*]"/>
  </satop>
</xsl:variable>

<xsl:variable name="E.ACTOR">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-BASE)/*"/>
    <xsl:with-param name="ext">	
       <xsl:copy-of select="$ACTOR"/>
    </xsl:with-param>
  </xsl:call-template>
  </lf>
</xsl:variable>

<xsl:variable name="E.ACTOR.PATIENT">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-BASE)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$ACTOR"/>
	  <xsl:copy-of select="$PATIENT"/>
    </xsl:with-param>
  </xsl:call-template>
  </lf>
</xsl:variable>    

<xsl:variable name="E.ACTOR.PATIENT.RECIPIENT">
  <lf>
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($E-BASE)/*"/>
      <xsl:with-param name="ext">	
        <xsl:copy-of select="$ACTOR"/> 	
	    <xsl:copy-of select="$PATIENT"/>
		<xsl:copy-of select="$RECIPIENT"/>
     </xsl:with-param>
   </xsl:call-template>
  </lf> 
 </xsl:variable>    

 <xsl:variable name="E.ACTOR.PATIENT.RESULTANT">
  <lf>
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($E-BASE)/*"/>
      <xsl:with-param name="ext">
	    <xsl:copy-of select="$ACTOR"/>	
	    <xsl:copy-of select="$PATIENT"/>
		<xsl:copy-of select="$RESULTANT"/>
     </xsl:with-param>
   </xsl:call-template>
   </lf>
 </xsl:variable>    

 <xsl:variable name="E.ACTOR.PATIENT.WHEREFROM.WHERETO">
  <lf>
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($E-BASE)/*"/>
      <xsl:with-param name="ext">
	    <xsl:copy-of select="$ACTOR"/>	
	    <xsl:copy-of select="$PATIENT"/>
		<xsl:copy-of select="$WHEREFROM"/>
		<xsl:copy-of select="$WHERETO"/>
     </xsl:with-param>
   </xsl:call-template>
   </lf>
 </xsl:variable>    
 
 <xsl:variable name="E.ACTOR.PATIENT.WHERETO">
  <lf>
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($E-BASE)/*"/>
      <xsl:with-param name="ext">
	    <xsl:copy-of select="$ACTOR"/>	
	    <xsl:copy-of select="$PATIENT"/>
		<xsl:copy-of select="$WHERETO"/>
     </xsl:with-param>
   </xsl:call-template>
   </lf>
 </xsl:variable>    
 
 <xsl:variable name="E.RESTR.SCOPE">
  <lf>
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($E-BASE)/*"/>
      <xsl:with-param name="ext">
       <xsl:copy-of select="$RESTR"/>
       <xsl:copy-of select="$SCOPE"/>
     </xsl:with-param>
   </xsl:call-template>
  </lf> 
 </xsl:variable>   

</xsl:transform>