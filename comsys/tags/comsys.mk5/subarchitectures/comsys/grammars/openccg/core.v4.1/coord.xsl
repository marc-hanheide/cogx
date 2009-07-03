<?xml version="1.0"?>
<!--

Currently handles basic np, n, adj and s (and vp) coordination
using standard list. 

No fancy stuff

Copyright (C) 2005 DFKI GmbH (Geert-Jan M. Kruijff gj@dfki.de)
-->

<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">



<!-- ================================================ -->  
<!-- CATEGORY DEFINITIONS                             -->
<!-- ================================================ --> 

<!-- Coordination over a category X involves the following
		X.1st  fs = 21    index = C1
		X.2nd  fs = 22    index = C2
		X.res  fs = 23    index = CR

		and  X.res\X.2nd/X.1st  with or without $ stacking

	 Note, unlike almost all other standard acounts of coord in OpenCCG, 
	 I do not use inheritance, i.e., the 3 cats are separate. 
	 	 
	 This is problematic in a number of cases:
			for example "the boys and girls" will lose its semantic type
	 but useful in others:
			I did not go and will not go. (2 conjuncts are of different tenses for example)

     This decision is based on the grammars "promiscous" tenents, but could be easily changed

-->
		
		

<!-- NP Coordination - result is plural marked in "and" and unconstrained in "or" -->

<xsl:variable name="np.1st">
    <atomcat type="np">
      <fs id="21">
	     <feat attr="index"><lf><nomvar name="C1"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="np.2nd">
    <atomcat type="np">
      <fs id="22">
        <feat attr="index"><lf><nomvar name="C2"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

<xsl:variable name="np.res">
    <atomcat type="np">
      <fs id="23">
        <feat attr="index"><lf><nomvar name="CR"/></lf></feat>
        <feat attr="pers"><featvar name="PERS:pervals"/></feat>
		<feat attr="nform" val="basic"/>
      </fs>
    </atomcat>
  </xsl:variable>

<xsl:variable name="np.res.pl-result">
    <atomcat type="np">
      <fs id="23">
        <feat attr="index"><lf><nomvar name="CR"/></lf></feat>
		<feat attr="num" val="pl"/>
		<feat attr="pers" val="3rd"/>
		<feat attr="nform" val="basic"/>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="np.coord.pl-result">
    <complexcat>
      <xsl:copy-of select="$np.res.pl-result"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$np.2nd"/>
      <slash dir="\" mode="&lt;"/>
      <xsl:copy-of select="$np.1st"/>
   </complexcat>
</xsl:variable>

<xsl:variable name="np.coord">
    <complexcat>
      <xsl:copy-of select="$np.res"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$np.2nd"/>
	  <slash dir="\" mode="&lt;"/>
      <xsl:copy-of select="$np.1st"/>
    </complexcat>
</xsl:variable>

  
 <!-- N Coordination -result is plural marked in "and" and unconstrained in "or" -->  
 
 <xsl:variable name="n.1st">
    <atomcat type="n">
      <fs id="21">
	     <feat attr="index"><lf><nomvar name="C1"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="n.2nd">
    <atomcat type="n">
      <fs id="22">
        <feat attr="index"><lf><nomvar name="C2"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

<xsl:variable name="n.res.pl-result">
    <atomcat type="n">
      <fs id="23">
        <feat attr="index"><lf><nomvar name="CR"/></lf></feat>
		<feat attr="num" val="pl"/>
		<feat attr="pers" val="3rd"/>
		<feat attr="nform" val="basic"/>
      </fs>
    </atomcat>
  </xsl:variable>

<xsl:variable name="n.res">
    <atomcat type="n">
      <fs id="23">
        <feat attr="index"><lf><nomvar name="CR"/></lf></feat>
        <feat attr="pers"><featvar name="PERS:pers-vals"/></feat>
		<feat attr="nform" val="basic"/>
      </fs>
    </atomcat>
  </xsl:variable>



<!-- stack over any args -->
<xsl:variable name="n.coord">
    <complexcat>
      <xsl:copy-of select="$n.res"/>
      <slash/>
	  <dollar name="1"/>
	  <slash dir="/" mode="&gt;"/>
      <complexcat>
         <xsl:copy-of select="$n.2nd"/>
	     <slash/>
	     <dollar name="1"/>  
      </complexcat>
      <slash dir="\" mode="&lt;"/>
      <complexcat>
         <xsl:copy-of select="$n.1st"/>
	     <slash/>
	     <dollar name="1"/>  
      </complexcat>
   </complexcat>
</xsl:variable>  

  <xsl:variable name="n.coord.pl-result">
    <complexcat>
      <xsl:copy-of select="$n.res.pl-result"/>
      <slash/>
	  <dollar name="1"/>
	  <slash dir="/" mode="&gt;"/>
      <complexcat>
         <xsl:copy-of select="$n.2nd"/>
	     <slash/>
	     <dollar name="1"/>  
      </complexcat>
	  
	  <slash dir="\" mode="&lt;"/>
      <complexcat>
         <xsl:copy-of select="$n.1st"/>
	     <slash/>
	     <dollar name="1"/>  
      </complexcat>

    </complexcat>
  </xsl:variable>  
  
  
  
  <!-- Sentence Coordination  -->
  
  <xsl:variable name="s.1st">
    <atomcat type="s">
      <fs id="21">
	     <feat attr="index"><lf><nomvar name="C1"/></lf></feat>
		 <feat attr="mood"><featvar name="MOOD21:mood-vals"/></feat>
		 <feat attr="vform"><featvar name="VFORM21:vform-vals"/></feat>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="s.2nd">
    <atomcat type="s">
      <fs id="22">
        <feat attr="index"><lf><nomvar name="C2"/></lf></feat>
        <feat attr="mood"><featvar name="MOOD21:mood-vals"/></feat>
        <feat attr="vform"><featvar name="VFORM21:vform-vals"/></feat>
      </fs>
    </atomcat>
  </xsl:variable>

<xsl:variable name="s.res">
    <atomcat type="s">
      <fs id="23">
        <feat attr="index"><lf><nomvar name="CR"/></lf></feat>
        <feat attr="mood"><featvar name="MOOD21:mood-vals"/></feat>
        <feat attr="vform"><featvar name="VFORM21:vform-vals"/></feat>
      </fs>
    </atomcat>
  </xsl:variable>

   <xsl:variable name="s.coord">
    <complexcat>
      <xsl:copy-of select="$s.res"/>
      <slash/>
	  <dollar name="1"/>
	  <slash dir="/" mode="&gt;"/>
      <complexcat>
         <xsl:copy-of select="$s.2nd"/>
	     <slash/>
	     <dollar name="1"/>  
      </complexcat>
	  <slash dir="\" mode="&lt;"/>
      <complexcat>
         <xsl:copy-of select="$s.1st"/>
	     <slash/>
	     <dollar name="1"/>  
      </complexcat>
    </complexcat>
  </xsl:variable>

  <!-- ================================================ -->  
  <!-- LEXICAL-MEANING DEFINITIONS                      -->
  <!-- ================================================ --> 
  
  <!-- Currently, all categories using the same semantics, and only this form -->


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
  
  
  <!-- ================================================ -->  
  <!-- LEXICAL FAMILY DEFINITIONS                       -->
  <!-- ================================================ -->  


  <xsl:template name="add-coord-families">

 <family name="coord.np" pos="COORD"  indexRel="First" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np.coord)/*"/>
        <xsl:with-param name="ext" select="$FIRST.NEXT"/>
      </xsl:call-template>
    </entry>
  </family>

 <family name="coord.np.pl-result" pos="COORD"  indexRel="First" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np.coord.pl-result)/*"/>
        <xsl:with-param name="ext" select="$FIRST.NEXT"/>
      </xsl:call-template>
    </entry>
  </family>

   
 <family name="coord.n.pl-result" pos="COORD"  indexRel="First" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($n.coord.pl-result)/*"/>
        <xsl:with-param name="ext" select="$FIRST.NEXT"/>
      </xsl:call-template>
    </entry>
  </family>
   
 <family name="coord.n" pos="COORD"  indexRel="First" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($n.coord)/*"/>
        <xsl:with-param name="ext" select="$FIRST.NEXT"/>
      </xsl:call-template>
    </entry>
  </family>
  
  <family name="coord.s" pos="COORD"  indexRel="First" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($s.coord)/*"/>
        <xsl:with-param name="ext" select="$FIRST.NEXT"/>
      </xsl:call-template>
    </entry>
  </family>
 
  </xsl:template>

</xsl:transform>