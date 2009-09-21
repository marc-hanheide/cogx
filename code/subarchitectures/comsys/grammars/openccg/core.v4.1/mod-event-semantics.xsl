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
<!-- BASE MODIFIER FOR EACH EVENT TYPE 											      -->
<!-- ==================================================================================== -->
 
  <xsl:variable name="E-DYNAMIC">
    <satop nomvar="E0:m-dynamic"></satop>
  </xsl:variable>

  <xsl:variable name="E-LOCATION">
    <satop nomvar="E0:m-location"></satop>
  </xsl:variable>

  <xsl:variable name="E-MANNER">
    <satop nomvar="E0:m-manner"></satop>
  </xsl:variable>

  <xsl:variable name="E-TIME">
    <satop nomvar="E0:m-time"></satop>
  </xsl:variable>

  <xsl:variable name="E-FREQUENCY">
    <satop nomvar="E0:m-frequency"></satop>
  </xsl:variable>

  <xsl:variable name="E-PROBABIILTY">
    <satop nomvar="E0:m-probability"></satop>
  </xsl:variable>

  <xsl:variable name="E-ACCOMPANIMENT">
    <satop nomvar="E0:m-accompaniment"></satop>
  </xsl:variable>
  
  <xsl:variable name="E-BENEFACTOR">
    <satop nomvar="E0:m-benefactor"></satop>
  </xsl:variable>

 <xsl:variable name="E-COMMENT">
    <satop nomvar="E0:m-comment"></satop>
 </xsl:variable>

 <xsl:variable name="E-COMPARISION">
    <satop nomvar="E0:m-comparison"></satop>
 </xsl:variable>

 <xsl:variable name="E-RELATIONAL">
    <satop nomvar="E0:m-relational"></satop>
 </xsl:variable>

 <xsl:variable name="E-PURPOSE">
    <satop nomvar="E0:m-purpose"></satop>
 </xsl:variable>


 <xsl:variable name="E-NEGATION">
    <satop nomvar="E0:m-negation"></satop>
 </xsl:variable>


<!-- ==================================================================================== -->
<!-- SEMANTICS: EVENT MODIFYING															  -->
<!-- ==================================================================================== -->

<!-- NEGATION -->

<!-- Addeed Nom-var with Default Prop to make realization work. 
		Should most likely be using scope for this... -->
		
 <xsl:variable name="MOD-E.NEGATION">
    <lf>
      <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($E-NEGATION)/*"/>
      <xsl:with-param name="ext">	
    	      <diamond mode="Polarity"><prop name="-"/></diamond>
	      </xsl:with-param>
      </xsl:call-template>
    </lf>
 </xsl:variable>


<!-- FREQUENCY -->
<xsl:variable name="MOD-E.FREQUENCY">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-FREQUENCY)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$MODIFIER"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<!-- PROBABILITY -->

<xsl:variable name="MOD-E.PROBABILITY">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-PROBABIILTY)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$MODIFIER"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<!-- MANNER -->
<xsl:variable name="MOD-E.MANNER">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-MANNER)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$MODIFIER"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<!-- TIME -->
<xsl:variable name="MOD-E.TIME">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-TIME)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$MODIFIER"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<!-- COMMENT -->
<xsl:variable name="MOD-E.COMMENT">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-COMMENT)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$MODIFIER"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<!-- STATIC LOCATION -->

<xsl:variable name="MOD-E.LOCATION">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-LOCATION)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$LOCATION"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<xsl:variable name="MOD-E.LOCATION-WITH-ANCHOR">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-LOCATION)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$LOCATION-WITH-ANCHOR"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<!-- DYNAMIC EVENTS -->

  <xsl:variable name="MOD-E.WHERETO">
     <lf>
       <xsl:call-template name="extend">
       <xsl:with-param name="elt" select="xalan:nodeset($E-DYNAMIC)/*"/>
       <xsl:with-param name="ext">	
         <xsl:copy-of select="$WHERETO"/>
       </xsl:with-param>
       </xsl:call-template>
    </lf>
  </xsl:variable>

  <xsl:variable name="MOD-E.WHICHWAY">
     <lf>
       <xsl:call-template name="extend">
       <xsl:with-param name="elt" select="xalan:nodeset($E-DYNAMIC)/*"/>
       <xsl:with-param name="ext">	
         <xsl:copy-of select="$WHICHWAY"/>
       </xsl:with-param>
       </xsl:call-template>
    </lf>
  </xsl:variable>

  <xsl:variable name="MOD-E.WHEREFROM">
     <lf>
       <xsl:call-template name="extend">
       <xsl:with-param name="elt" select="xalan:nodeset($E-DYNAMIC)/*"/>
       <xsl:with-param name="ext">	
         <xsl:copy-of select="$WHEREFROM"/>
       </xsl:with-param>
       </xsl:call-template>
    </lf>
  </xsl:variable>


  <xsl:variable name="MOD-E.THROUGH">
     <lf>
       <xsl:call-template name="extend">
       <xsl:with-param name="elt" select="xalan:nodeset($E-DYNAMIC)/*"/>
       <xsl:with-param name="ext">	
         <xsl:copy-of select="$THROUGH"/>
       </xsl:with-param>
       </xsl:call-template>
    </lf>
  </xsl:variable>

<!-- ACCOMPIANIMENT -->

<xsl:variable name="MOD-E.ACCOMPANIMENT">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-ACCOMPANIMENT)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$ACCOMPANIMENT"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<xsl:variable name="MOD-E.BENEFACTOR">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-BENEFACTOR)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$M-BENEFACTOR"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<!-- Comparision -->

<xsl:variable name="MOD-E.COMPARISION-ENTITY">
     <lf>
       <xsl:call-template name="extend">
       <xsl:with-param name="elt" select="xalan:nodeset($E-COMPARISION)/*"/>
       <xsl:with-param name="ext">	
         <xsl:copy-of select="$COMPARISON-ENTITY"/>
       </xsl:with-param>
       </xsl:call-template>
    </lf>
  </xsl:variable>


<!-- RELATIONAL -->
<!-- Neben-Satz Like -->

<xsl:variable name="MOD-E.RESULT">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-RELATIONAL)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$RESULT"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<xsl:variable name="MOD-E.CAUSE">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-RELATIONAL)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$CAUSE"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<xsl:variable name="MOD-E.CONDITION">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-RELATIONAL)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$CONDITION"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<xsl:variable name="MOD-E.PURPOSE">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-PURPOSE)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$PURPOSE"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<xsl:variable name="MOD-E.TIME">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-TIME)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$TIME"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<xsl:variable name="MOD-E.PURPOSE-ENTITY">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-RELATIONAL)/*"/>
    <xsl:with-param name="ext">	
      <xsl:copy-of select="$PURPOSE-ENTITY"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    


	 
</xsl:transform>