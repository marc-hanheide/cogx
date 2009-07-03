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

<!-- CATS -->

 <xsl:variable name="adj">  
	<atomcat type="adj">
      <fs id="25">
        <feat attr="index"><lf><nomvar name="M"/></lf></feat>
    <!--   <feat attr="dep-rel"><featvar name="DR:dr-vals"/></feat> -->
        <feat attr="modifiable" val="+"/>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="adj2">  
	<atomcat type="adj">
      <fs id="26">
        <feat attr="index"><lf><nomvar name="M2"/></lf></feat>
    <!--   <feat attr="dep-rel"><featvar name="DR:dr-vals"/></feat> -->
        <feat attr="modifiable" val="+"/>
      </fs>
    </atomcat>
  </xsl:variable>


<!-- COMPLEX FORMING RULES -->

 <xsl:variable name="adj.mod-n">
    <complexcat>
  	  <xsl:copy-of select="$n.from"/>
	  <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$n"/>
    </complexcat>
 </xsl:variable>   

<xsl:variable name="comp-type.adj">
   <arg> 
      <xsl:copy-of select="$adj"/>
   </arg> 
   <result> 
     <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adj.mod-n)/*"/>
        <xsl:with-param name="ext">
	     <lf>
          <satop nomvar="T">
	         <diamond mode="Property"><nomvar name="M"/></diamond> 
	       </satop>
         </lf> 	
	   </xsl:with-param>	 
     </xsl:call-template>
   </result> 
 </xsl:variable> 

   <!-- MODIFIER -->

 <xsl:variable name="mod-right.adj">
   <complexcat>
      <xsl:copy-of select="$adj"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$adj"/>
   </complexcat>
 </xsl:variable>   

 <xsl:variable name="mod-right.adj2">
   <complexcat>
      <xsl:copy-of select="$adj"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$adj2"/>
   </complexcat>
 </xsl:variable>   
 
 <!-- Adj Coordination  -->

<xsl:variable name="adj.1st">
    <atomcat type="adj">
      <fs id="21">
	     <feat attr="index"><lf><nomvar name="C1"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="adj.2nd">
    <atomcat type="adj">
      <fs id="22">
        <feat attr="index"><lf><nomvar name="C2"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

<xsl:variable name="adj.res">
    <atomcat type="adj">
      <fs id="23">
        <feat attr="index"><lf><nomvar name="CR"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="coord.adj">
    <complexcat>
      <xsl:copy-of select="$adj.res"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$adj.2nd"/>
      <slash dir="\" mode="&lt;"/>
      <xsl:copy-of select="$adj.1st"/>
    </complexcat>
  </xsl:variable>



<xsl:template name="add-adj-families">

 <family name="adj" pos="ADJ" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($adj)/*"/>
        <xsl:with-param name="ext" select="$MOD"/>
      </xsl:call-template>
    </entry>
 </family>

 <family name="coord.adj" pos="COORD"  indexRel="First" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($coord.adj)/*"/>
        <xsl:with-param name="ext" select="$FIRST.NEXT"/>
      </xsl:call-template>
    </entry>
 </family>

<family name="negation.adj2" pos="MOD" indexRel="Polarity"  closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-right.adj2)/*"/>
        <xsl:with-param name="ext" select="$MOD-NEGATION2"/>
      </xsl:call-template>
    </entry>
 </family>

<family name="negation.adj" pos="MOD" indexRel="Polarity"  closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-right.adj)/*"/>
        <xsl:with-param name="ext" select="$MOD-NEGATION"/>
      </xsl:call-template>
    </entry>
 </family>


<family name="mod-right.adj" pos="MOD"  closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-right.adj)/*"/>
        <xsl:with-param name="ext" select="$MOD-MODIFIER"/>
      </xsl:call-template>
    </entry>
 </family>


 
 </xsl:template>

</xsl:transform>

 



 

