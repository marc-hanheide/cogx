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

  <xsl:variable name="det">  
	<atomcat type="det">
      <fs id="25">
        <feat attr="index"><lf><nomvar name="M"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>


<!-- determiner modifiers -->
 <xsl:variable name="mod-right.det">
   <complexcat>
      <xsl:copy-of select="$det"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$det"/>
   </complexcat>
 </xsl:variable>   


<!-- COMPLEX FORMING RULES -->

 <xsl:variable name="det.complex">
    <complexcat>
  	  <xsl:copy-of select="$np"/>
	  <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$n"/>
    </complexcat>
 </xsl:variable>   

<xsl:variable name="comp-type.det">
   <arg> 
      <xsl:copy-of select="$det"/>
   </arg> 
   <result> 
     <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.complex)/*"/>
        <xsl:with-param name="ext">
	     <lf>
          <satop nomvar="T">
	         <diamond mode="Det"><nomvar name="M"/></diamond> 
	       </satop>
         </lf> 	
	   </xsl:with-param>	 
     </xsl:call-template>
   </result> 
 </xsl:variable> 

<xsl:template name="add-det-families">

 <family name="det" pos="DET" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($det)/*"/>
        <xsl:with-param name="ext" select="$MOD"/>
      </xsl:call-template>
    </entry>
 </family>

 <family name="mod-right.det" pos="MOD"  closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-right.det)/*"/>
        <xsl:with-param name="ext" select="$MOD-MODIFIER"/>
      </xsl:call-template>
    </entry>
 </family>
 
 </xsl:template>

</xsl:transform>

