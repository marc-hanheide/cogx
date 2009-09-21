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

 <xsl:variable name="pp">  
	<atomcat type="pp">
      <fs id="25">
        <feat attr="index"><lf><nomvar name="M"/></lf></feat>
    <!--   <feat attr="dep-rel"><featvar name="DR:dr-vals"/></feat> -->
        <feat attr="modifiable" val="+"/>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="prep">
   <complexcat>
      <xsl:copy-of select="$pp"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$np.mod-arg"/>
   </complexcat>
 </xsl:variable>   



<!-- COMPLEX FORMING RULES 
		now just built direct into rule below

 <xsl:variable name="prep.mod-n">
    <complexcat>
  	  <xsl:copy-of select="$n"/>
	  <setarg>
        <slash dir="\" mode="*"/>
        <xsl:copy-of select="$n"/>
        <slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$n.mod-arg"/>
	  </setarg>
    </complexcat>
  </xsl:variable>   



<xsl:variable name="prep.mod-n">
    <complexcat>
  	  <xsl:copy-of select="$n"/>
	  <setarg>
        <slash dir="\" mode="*"/>
        <xsl:copy-of select="$n"/>
        <slash/>
	    <dollar name="1"/>
  	  </setarg>
    </complexcat>
  </xsl:variable>   

-->

<xsl:variable name="comp-type.prep-n">
   <arg> 
    <complexcat>
	   <xsl:copy-of select="$pp"/>
       <slash/>
       <dollar name="1"/>
    </complexcat>
   </arg> 
   <result>
      <complexcat>
	      <xsl:copy-of select="$n.from"/>
		  <slash/>
          <dollar name="1"/>
          <slash dir="\" mode="*"/>
		  <xsl:copy-of select="$n"/>         
	      <lf>
          <satop nomvar="T">
	         <diamond mode="Property"><nomvar name="M"/></diamond> 
	       </satop>
         </lf> 	
      </complexcat>
   </result> 
 </xsl:variable> 

<xsl:variable name="comp-type.prep-s">
   <arg> 
    <complexcat>
	   <xsl:copy-of select="$pp"/>
       <slash/>
       <dollar name="1"/>
    </complexcat>
   </arg> 
   <result>
      <complexcat>
	      <xsl:copy-of select="$s.from"/>
		  <slash/>
          <dollar name="1"/>
          <slash dir="\" mode="*"/>
		  <xsl:copy-of select="$s"/>         
	      <lf>
          <satop nomvar="E0">
	         <diamond mode="Property"><nomvar name="M"/></diamond> 
	       </satop>
         </lf> 	
      </complexcat>
   </result> 
 </xsl:variable> 


<!-- MODIFIER -->

 <xsl:variable name="mod-right.pp">
   <complexcat>
      <xsl:copy-of select="$pp"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$pp"/>
   </complexcat>
 </xsl:variable>   


<!-- COORDINATION -->

<xsl:variable name="pp.1st">
    <atomcat type="pp">
      <fs id="21">
	     <feat attr="index"><lf><nomvar name="C1"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="pp.2nd">
    <atomcat type="pp">
      <fs id="22">
        <feat attr="index"><lf><nomvar name="C2"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

<xsl:variable name="pp.res">
    <atomcat type="pp">
      <fs id="23">
        <feat attr="index"><lf><nomvar name="CR"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

<!-- stack over any args -->
<xsl:variable name="coord.pp">
    <complexcat>
      <xsl:copy-of select="$pp.res"/>
      <slash/>
	  <dollar name="1"/>
	  <slash dir="/" mode="&gt;"/>
      <complexcat>
         <xsl:copy-of select="$pp.2nd"/>
	     <slash/>
	     <dollar name="1"/>  
      </complexcat>
      <slash dir="\" mode="&lt;"/>
      <complexcat>
         <xsl:copy-of select="$pp.1st"/>
	     <slash/>
	     <dollar name="1"/>  
      </complexcat>
   </complexcat>
</xsl:variable>  


<xsl:template name="add-prep-families">

 <family name="prep" pos="PREP" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($prep)/*"/>
        <xsl:with-param name="ext" select="$MOD.ARG"/>
      </xsl:call-template>
    </entry>
 </family>

<family name="pro-prep" pos="PREP" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($pp)/*"/>
        <xsl:with-param name="ext" select="$MOD"/>
      </xsl:call-template>
    </entry>
 </family>

 <family name="coord.prep" pos="COORD"  indexRel="First" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($coord.pp)/*"/>
        <xsl:with-param name="ext" select="$FIRST.NEXT"/>
      </xsl:call-template>
    </entry>
 </family>

<family name="negation.prep" pos="MOD"  indexRel="Polarity" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-right.pp)/*"/>
        <xsl:with-param name="ext" select="$MOD-NEGATION"/>
      </xsl:call-template>
    </entry>
</family>

<family name="mod-right.prep" pos="MOD"  closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-right.pp)/*"/>
        <xsl:with-param name="ext" select="$MOD-MODIFIER"/>
      </xsl:call-template>
    </entry>
 </family>



</xsl:template>

</xsl:transform>
