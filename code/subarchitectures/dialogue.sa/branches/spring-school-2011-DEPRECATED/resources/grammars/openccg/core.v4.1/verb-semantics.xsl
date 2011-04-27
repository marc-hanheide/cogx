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

 <!-- Currently not being used, if want to need to determine nom-var for part-->
 <xsl:variable name="E.ACTOR.PARTICLE">
   <lf>
     <xsl:call-template name="extend">
     <xsl:with-param name="elt" select="xalan:nodeset($E-BASE)/*"/>
     <xsl:with-param name="ext">
       <xsl:copy-of select="$ACTOR"/>	
       <xsl:copy-of select="$PARTICLE"/>
     </xsl:with-param>
   </xsl:call-template>
   </lf> 
 </xsl:variable>


<!-- ==================================================================================== -->
<!-- With just <PATIENT>                                                                  -->
<!-- ==================================================================================== -->

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
 
 <xsl:variable name="E.ACTOR.PATIENT-PHYSICAL">
  <lf>
  <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($E-BASE)/*"/>
    <xsl:with-param name="ext">
	   <xsl:copy-of select="$ACTOR"/>	
       <xsl:copy-of select="$PATIENT-PHYSICAL"/>
    </xsl:with-param>
  </xsl:call-template>
  </lf>
</xsl:variable>    

 <xsl:variable name="E.ACTOR.PATIENT-ANIMATE">
  <lf>
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($E-BASE)/*"/>
      <xsl:with-param name="ext">
	    <xsl:copy-of select="$ACTOR"/>	
        <xsl:copy-of select="$PATIENT-ANIMATE"/>
      </xsl:with-param>
   </xsl:call-template>
   </lf> 
 </xsl:variable>    


 <xsl:variable name="E.ACTOR.PATIENT.PARTICLE">
   <lf>
   <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($E-BASE)/*"/>
      <xsl:with-param name="ext">
	    <xsl:copy-of select="$ACTOR"/>	
	    <xsl:copy-of select="$PATIENT"/>
		<xsl:copy-of select="$PARTICLE"/>
     </xsl:with-param>
   </xsl:call-template>
   </lf>
 </xsl:variable>

<!-- ==================================================================================== -->
<!-- With <PATIENT> and <RECIPIENT> / <BENEFACTOR>                                        -->
<!-- ==================================================================================== -->

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

<xsl:variable name="E.ACTOR.PATIENT.BENEFACTOR">
  <lf>
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($E-BASE)/*"/>
      <xsl:with-param name="ext">	
        <xsl:copy-of select="$ACTOR"/> 	
	    <xsl:copy-of select="$PATIENT"/>
		<xsl:copy-of select="$BENEFACTOR"/>
     </xsl:with-param>
   </xsl:call-template>
  </lf> 
 </xsl:variable>    


<xsl:variable name="E.ACTOR.PATIENT.RECIPIENT-ANIMATE">
  <lf>
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($E-BASE)/*"/>
      <xsl:with-param name="ext">
	    <xsl:copy-of select="$ACTOR"/>	
	    <xsl:copy-of select="$PATIENT"/>
		<xsl:copy-of select="$RECIPIENT-ANIMATE"/>
     </xsl:with-param>
   </xsl:call-template>
   </lf>
 </xsl:variable>    

 
<!-- ==================================================================================== -->
<!-- With <PATIENT> and <RESULTANT>                                                      -->
<!-- ==================================================================================== -->

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



<!-- ==================================================================================== -->
<!-- With OTHERS                                                                          -->
<!-- ==================================================================================== -->

<xsl:variable name="E.ACTOR.VCOMP">
  <lf>
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($E-BASE)/*"/>
      <xsl:with-param name="ext">
	   <xsl:copy-of select="$ACTOR"/> 	
       <xsl:copy-of select="$VCOMP"/>
     </xsl:with-param>
   </xsl:call-template>
  </lf> 
 </xsl:variable>   

<xsl:variable name="E.ACTOR.PATIENT.VCOMP">
  <lf>
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($E-BASE)/*"/>
      <xsl:with-param name="ext">
	   <xsl:copy-of select="$ACTOR"/>
   	   <xsl:copy-of select="$PATIENT"/> 	
       <xsl:copy-of select="$VCOMP"/>
     </xsl:with-param>
   </xsl:call-template>
  </lf> 
 </xsl:variable>   

<!-- ==================================================================================== -->
<!-- RESTR SCOPE                                                                          -->
<!-- ==================================================================================== -->

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

<!-- ==================================================================================== -->
<!-- REFERENT ANNOUNCING                                                                  -->
<!-- ==================================================================================== -->

<xsl:variable name="E.EXISTENCE">
  <lf>
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($E-BASE)/*"/>
      <xsl:with-param name="ext">
       <xsl:copy-of select="$EXISTS"/>
     </xsl:with-param>
   </xsl:call-template>
  </lf> 
 </xsl:variable>   



</xsl:transform>