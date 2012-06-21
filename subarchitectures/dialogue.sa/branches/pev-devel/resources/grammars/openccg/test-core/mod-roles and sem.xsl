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



 <!-- BOUND CATS -->

 <xsl:variable name="np.mod-arg">  
	<atomcat type="np">
      <fs id="30">
        <feat attr="num"><featvar name="NUMMA:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERSMA:pers-vals"/></feat>
		<feat attr="case"><featvar name="CASEMA:case-vals"/></feat>
        <feat attr="index"><lf><nomvar name="TMA"/></lf></feat>
		<feat attr="nform" val="basic"/>
      </fs>
    </atomcat>
  </xsl:variable>



<!-- ROLES -->

   <xsl:variable name="ARG">
       <diamond mode="Arg"><nomvar name="TMA"/></diamond>
   </xsl:variable>

  <xsl:variable name="MODIFIER">
       <diamond mode="Modifier"><nomvar name="MM"/><prop name="[*DEFAULT*]"/></diamond>
   </xsl:variable>

  <xsl:variable name="NEGATIVE">
     <diamond mode="Polarity"><prop name="-"/></diamond>
  </xsl:variable>

<!-- SEMANTICS -->

 <xsl:variable name="MOD">
    <lf>
      <satop nomvar="M">
		<prop name="[*DEFAULT*]"/>
      </satop>
    </lf>
 </xsl:variable>

  <xsl:variable name="MOD.ARG">
    <lf>
      <satop nomvar="M">
		<prop name="[*DEFAULT*]"/>
        <xsl:copy-of select="$ARG"/>
      </satop>
    </lf>
  </xsl:variable>

  <xsl:variable name="MOD-MODIFIER">
	<lf>
	  <satop nomvar="M">
	     <xsl:copy-of select="$MODIFIER"/>
      </satop>
	</lf>
  </xsl:variable>

 <xsl:variable name="MOD-NEGATION">
	<lf>
	  <satop nomvar="M">
	     <xsl:copy-of select="$NEGATIVE"/>
      </satop>
	</lf>
  </xsl:variable>

 <xsl:variable name="MOD-NEGATION2">
	<lf>
	  <satop nomvar="M">
		<prop name="[*DEFAULT*]"/>
        <diamond mode="Scope"><nomvar name="M2"/></diamond>
		
      </satop>
	</lf>
  </xsl:variable>


<!-- Where should I go ?? -->

  <xsl:variable name="FIRST.NEXT">
    <lf>
      <satop nomvar="CR">
        <prop name="[*DEFAULT*]"/>
        <diamond mode="First">
          <nomvar name="L1"/>
          <prop name="elem"/>
          <diamond mode="Item"><nomvar name="C1"/></diamond>
          <diamond mode="Next">
            <nomvar name="L2"/>
            <prop name="elem"/>
            <diamond mode="Item"><nomvar name="C2"/></diamond>
          </diamond>
        </diamond>
      </satop>
    </lf>
  </xsl:variable>
  



</xsl:transform>

