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
  
<!--
   <xsl:variable name="adv-to-adv">
    <complexcat>
        <xsl:copy-of select="$adv.basic-cat"/>	
		<slash dir="/" mode="^"/>
	    <xsl:copy-of select="$adv.basic-cat"/>
    </complexcat> 
   </xsl:variable> -->
  
  
  <xsl:variable name="adj-to-adj">
    <complexcat>
      <xsl:copy-of select="$adj.basic-cat"/>
      <slash dir="/" mode="&gt;"/>
	  <xsl:copy-of select="$adj.basic-cat"/>
    </complexcat>
  </xsl:variable>

<!-- used in "big enough to kick" adj/vp-missing-obj\adj -->

 <xsl:variable name="adj-to-adj.inf-verb">    
    <complexcat>
      <xsl:copy-of select="$adj.basic-cat"/>
      <slash dir="/" mode="&gt;"/>
	  <complexcat>
 			<xsl:copy-of select="$s.mod-arg.inf"/>
         	<slash dir="\" mode="&gt;"/>
	        <xsl:copy-of select="$np.subj"/>
	        <slash dir="/" mode="&gt;"/>
			<xsl:copy-of select="$np.generic"/> 		 
	  </complexcat>
	  <slash dir="/" mode="&gt;"/>
	  <xsl:copy-of select="$adj.basic-cat"/>
   </complexcat>
 </xsl:variable>

 <xsl:variable name="adj-to-adj.forward">
    <complexcat>
      <xsl:copy-of select="$adj.basic-cat"/>
      <slash dir="\" mode="&gt;"/>
	  <xsl:copy-of select="$adj.basic-cat"/>
    </complexcat>
  </xsl:variable>

<!-- used in "too big to kick" -->

 <xsl:variable name="adj-to-adj.forward.inf-verb">    
    <complexcat>
      <xsl:copy-of select="$adj.basic-cat"/>
      <slash dir="/" mode="&gt;"/>
      <complexcat>
 			<xsl:copy-of select="$s.mod-arg.inf"/>
         	<slash dir="\" mode="&gt;"/>
	        <xsl:copy-of select="$np.subj"/>
	        <slash dir="/" mode="&gt;"/>
			<xsl:copy-of select="$np.generic"/> 		 
	  </complexcat>
	  <slash dir="\" mode="&gt;"/>
	  <xsl:copy-of select="$adj.basic-cat"/>
    </complexcat>
 </xsl:variable>
   
   <xsl:variable name="MOD-MODIFIER">
	<lf>
	  <satop nomvar="M">
	     <diamond mode="Modifier"><nomvar name="MM"/><prop name="[*DEFAULT*]"/></diamond>
      </satop>
	</lf>
   </xsl:variable>

   <xsl:variable name="MOD-MODIFIER-WITH-ECOMP">
	<lf>
	  <satop nomvar="M">
	     <diamond mode="Modifier">
			<nomvar name="MM"/><prop name="[*DEFAULT*]"/>
			<xsl:copy-of select="$M-ECOMP"/>
		 </diamond>
      </satop>
	</lf>
   </xsl:variable>


  <xsl:template name="add-mod-modifier-families">

<!--
   <family name="adv-modifier" pos="ADV" closed="true">
     <entry name="initial">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv-to-adv)/*"/>
        <xsl:with-param name="ext" select="$MOD-MODIFIER"/>
      </xsl:call-template>
     </entry>
   </family>
-->

   <!-- REALLY big -->

  <family name="adj-modifier" pos="ADV" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adj-to-adj)/*"/>
        <xsl:with-param name="ext" select="$MOD-MODIFIER"/>
      </xsl:call-template>
     </entry>
  </family>

   <!-- TOO big to kick-->

  <family name="adj-modifier.to-verb" pos="ADV" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adj-to-adj.inf-verb)/*"/>
        <xsl:with-param name="ext" select="$MOD-MODIFIER-WITH-ECOMP"/>
      </xsl:call-template>
     </entry>
  </family>

 <family name="adj-modifier.forward" pos="ADV" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adj-to-adj.forward)/*"/>
        <xsl:with-param name="ext" select="$MOD-MODIFIER"/>
      </xsl:call-template>
     </entry>
  </family>

   <!-- big ENOUGH to kick-->

 <family name="adj-modifier.forward.to-verb" pos="ADV" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adj-to-adj.forward.inf-verb)/*"/>
        <xsl:with-param name="ext" select="$MOD-MODIFIER-WITH-ECOMP"/>
      </xsl:call-template>
     </entry>
  </family>

   
   </xsl:template>
 </xsl:transform>

