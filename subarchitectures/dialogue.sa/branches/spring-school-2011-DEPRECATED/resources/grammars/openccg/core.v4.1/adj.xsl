<?xml version="1.0"?>
<!--
Copyright (C) 2005 DFKI GmbH (Geert-Jan M. Kruijff gj@dfki.de)


UNARY RULE TURNS BASIC CAT INTO n/n 

MACROS ADD DEGREE INFORMATION:  syntactic atr: aform
                                semantic :<degree>  


-->

<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">


<!-- ==================================================================================== -->
<!-- CATEGORY DEFINITIONS                                                                 -->
<!-- ==================================================================================== -->


      
<!--  Adj cat

	The base type for adjectives is as a base-cat. To get the combinatorially  complex
	cat n/n, a unary rule is used (see unary-rule.xsl)  
	This is necessary to handle adj modifiers like "really"

  -->	

 <xsl:variable name="adj">
	<atomcat type="adj">
      <fs id="25">
        <feat attr="index"><lf><nomvar name="M"/></lf></feat>
        <feat attr="modifiable" val="+"/>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="adj.base">
	<atomcat type="adj">
      <fs id="25">
        <feat attr="index"><lf><nomvar name="M"/></lf></feat>
		<feat attr="aform" val="aform-base"/>
      </fs>
    </atomcat>
  </xsl:variable>
 
 <xsl:variable name="adj.comparative">
	<atomcat type="adj">
      <fs id="25">
        <feat attr="index"><lf><nomvar name="M"/></lf></feat>
		<feat attr="aform" val="comparative"/>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="adj.superlative">
	<atomcat type="adj">
      <fs id="25">
        <feat attr="index"><lf><nomvar name="M"/></lf></feat>
		<feat attr="aform" val="superlative"/>
      </fs>
    </atomcat>
  </xsl:variable>

<xsl:variable name="from-adj.superlative">
	<atomcat type="adj">
      <fs inheritsFrom="25">
         <feat attr="aform" val="superlative"/>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="from-adj.comparative">
	<atomcat type="adj">
      <fs inheritsFrom="25">
         <feat attr="aform" val="comparative"/>
      </fs>
    </atomcat>
 </xsl:variable>


<!--  used for things like "proud of", "tired of", etc. 
	  should be moved to Mod-Roles 
-->
 
 <xsl:variable name="adj-of-np">
    <complexcat>
      <xsl:copy-of select="$adj"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$of.mod-arg"/> 
    </complexcat>
  </xsl:variable>

 <xsl:variable name="adj-with-np">
    <complexcat>
      <xsl:copy-of select="$adj"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$with.mod-arg"/> 
    </complexcat>
  </xsl:variable>



<!-- allowed to -->

 <xsl:variable name="adj.inf-verb">
    <complexcat>
      <xsl:copy-of select="$adj"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$vp.mod-arg.inf"/> 
    </complexcat>
  </xsl:variable>
 
 <!-- these are the complex n/n forms 
	  they may be used externally -->
 
 <xsl:variable name="adj.mod-n">
    <complexcat>
      <xsl:copy-of select="$n.from-generic"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$n.generic"/>	
    </complexcat>
  </xsl:variable>



<!-- MODIFIER 

sort out problem with expansion!!!  We don't want extra reading for copular

-->



<xsl:variable name="from-adj">
	<atomcat type="adj">
      <fs inheritsFrom="25">
          <feat attr="modifiable" val="+"/>
      </fs>
    </atomcat>
  </xsl:variable>
  
  
 <xsl:variable name="adj.no-expand">
	<atomcat type="adj">
      <fs id="25">
        <feat attr="index"><lf><nomvar name="M"/></lf></feat>
        <feat attr="modifiable" val="-"/>
      </fs>
    </atomcat>
  </xsl:variable>
 

<!--generic
	
		adj/adj.no-expand    the big, the big ball won't parse
		from-adj.expand/adj.no-expand  the big OK, the big ball GIVES ONLY BAD READING (waiting for adj)

		PROBLEM WAS IN COMPLEX CAT, positioning of dollar (see mod.cats)

-->
 <xsl:variable name="mod-right.adj">
   <complexcat>
      <xsl:copy-of select="$from-adj"/>
      <slash dir="/" mode="&gt;"/> <!-- pre 7/11 WA ^ -->
      <xsl:copy-of select="$adj"/>
   </complexcat>
 </xsl:variable>   

<!-- more and most -->

 <xsl:variable name="mod-right.adj.to-superlative">
   <complexcat>
      <xsl:copy-of select="$from-adj.superlative"/>
      <slash dir="/" mode="^"/>
      <xsl:copy-of select="$adj.base"/>
   </complexcat>
 </xsl:variable>   


 <xsl:variable name="mod-right.adj.to-comparative">
   <complexcat>
      <xsl:copy-of select="$from-adj.comparative"/>
      <slash dir="/" mode="^"/>
      <xsl:copy-of select="$adj.base"/>
   </complexcat>
 </xsl:variable>   






<!-- COORDINATION -->

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

<!-- stack over any args -->
<xsl:variable name="coord.adj">
    <complexcat>
      <xsl:copy-of select="$adj.res"/>
      <xsl:copy-of select="$with-dollar"/>
	  <slash dir="/" mode="&gt;"/>
      <complexcat>
         <xsl:copy-of select="$adj.2nd"/>
	     <xsl:copy-of select="$with-dollar"/>
      </complexcat>
      <slash dir="\" mode="&lt;"/>
      <complexcat>
         <xsl:copy-of select="$adj.1st"/>
         <xsl:copy-of select="$with-dollar"/>
      </complexcat>
   </complexcat>
</xsl:variable>  

 
	
<!-- ==================================================================================== -->
<!-- LEXICAL-FAMILY DEFINITIONS                                                           -->
<!-- ==================================================================================== -->

 <xsl:template name="add-adj-families">

  <!-- standard adjective family -->
	
  <family name="adj" pos="ADJ" closed="true">
     <entry name="predicative">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adj)/*"/>
        <xsl:with-param name="ext" select="$MOD"/>
      </xsl:call-template>
    </entry>
  </family> 
  

 <!-- proud of, tired of, etc-->
  
 <family name="adj.of-np" pos="ADJ" closed="true">
    <entry name="predicative">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adj-of-np)/*"/>
        <xsl:with-param name="ext" select="$MOD.ARG"/>
      </xsl:call-template>
    </entry>
 </family> 
 
 <family name="adj.with-np" pos="ADJ" closed="true">
    <entry name="predicative">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adj-with-np)/*"/>
        <xsl:with-param name="ext" select="$MOD.ARG"/>
      </xsl:call-template>
    </entry>
 </family> 


 <!-- "they are allowed to play in the living room" , etc-->
  
 <family name="adj.to-verb" pos="ADJ" closed="true">
    <entry name="predicative">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adj.inf-verb)/*"/>
        <xsl:with-param name="ext" select="$MOD.ECOMP"/>
      </xsl:call-template>
    </entry>
 </family> 
 
 <!-- COORDINATION -->

 <family name="coord.adj" pos="COORD"  indexRel="First" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($coord.adj)/*"/>
        <xsl:with-param name="ext" select="$FIRST.NEXT"/>
      </xsl:call-template>
    </entry>
 </family>

 <!-- MODIFICATION -->

<family name="negation.adj" pos="MOD"  indexRel="Polarity" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-right.adj)/*"/>
        <xsl:with-param name="ext" select="$MOD-NEGATION"/>
      </xsl:call-template>
    </entry>
</family>

<family name="more-adj" pos="MOD"  closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-right.adj.to-comparative)/*"/>
        <xsl:with-param name="ext" select="$MOD.COMPARATIVE"/>
      </xsl:call-template>
    </entry>
 </family>


<family name="most-adj" pos="MOD"  closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-right.adj.to-superlative)/*"/>
        <xsl:with-param name="ext" select="$MOD.SUPERLATIVE"/>
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


<!-- TYPE CHANGING RULES -->



 <xsl:variable name="mod-n.right.TEST">
	<complexcat>
	      <xsl:copy-of select="$n.from-generic"/>
          <slash dir="/" mode="^"/>
		  <xsl:copy-of select="$n.generic"/>         
		  <xsl:copy-of select="$with-dollar"/>
     </complexcat>
 </xsl:variable>    
	
<xsl:template name="add-adj-rules">

<typechanging name="adj"> 
   <arg> 
     <complexcat>
	   <atomcat type="adj">
        <fs id="26">
	      <feat attr="index"><lf><nomvar name="M0"/></lf></feat>
          <feat attr="modifiable" val="+"/>
		  <feat attr="mod-type" val="s-dynamic"/>
		  <feat attr="complexcat-type" val="s-left"/>
        </fs>
       </atomcat>
       <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result> 
	    <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-n.right)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-T-XPROPERTY"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>


 </xsl:template>



</xsl:transform>