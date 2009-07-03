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

 <xsl:variable name="n">  
	<atomcat type="n">
      <fs id="30">
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
        <xsl:copy-of select="$NUM.PERS"/> 
		<feat attr="nform" val="basic"/>
		<feat attr="modifiable" val="+"/>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="n.coord">  
	<atomcat type="n">
      <fs id="30">
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
        <xsl:copy-of select="$NUM.PERS"/> 
		<feat attr="nform" val="basic"/>
		<feat attr="modifiable" val="+"/>
		<feat attr="coord" val="+"/>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="n.from">
    <atomcat type="n">
      <fs inheritsFrom="30">
        <feat attr="coord" val="-"/>
      </fs>
	</atomcat>
  </xsl:variable>

 <xsl:variable name="np">  
	<atomcat type="np">
      <fs id="30">
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
        <xsl:copy-of select="$NUM.PERS.CASE"/> 
		<feat attr="nform" val="basic"/>
		<feat attr="modifiable" val="+"/>
      </fs>
    </atomcat>
  </xsl:variable>

<!-- SEMANTICS -->

  <xsl:variable name="ENTITY">
    <lf>
      <satop nomvar="T">
		<prop name="[*DEFAULT*]"/>
      </satop>
    </lf>
  </xsl:variable>



<!-- MODIFIERS -->

	<!-- See Adj, Prep & Relative-clause (in other??) -->



<!-- COORDINATION -->

	<!-- NOTED:  COORD FEATURE used to block the "nouns with args" created by
				 adj & prep unary rules from using this coordination
				 otherwise we get weird readings for "big and red" like
						<first> big, red T
						<next> T
    -->
	

<!-- BOUND CATS -->

<xsl:variable name="n.1st">
    <atomcat type="n">
      <fs id="21">
	     <feat attr="index"><lf><nomvar name="C1"/></lf></feat>
		 		<feat attr="coord" val="+"/>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="n.2nd">
    <atomcat type="n">
      <fs id="22">
        <feat attr="index"><lf><nomvar name="C2"/></lf></feat>
				<feat attr="coord" val="+"/>
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
				<feat attr="coord" val="+"/>
      </fs>
    </atomcat>
  </xsl:variable>

<xsl:variable name="n.res">
    <atomcat type="n">
      <fs id="23">
        <feat attr="index"><lf><nomvar name="CR"/></lf></feat>
        <feat attr="pers"><featvar name="CR-PERS:pers-vals"/></feat>
		<feat attr="nform" val="basic"/>
				<feat attr="coord" val="+"/>
      </fs>
    </atomcat>
  </xsl:variable>

<!-- stack over any args -->
<xsl:variable name="coord.n">
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

  <xsl:variable name="coord.n.pl-result">
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

 <xsl:variable name="coord.np.pl-result">
    <complexcat>
      <xsl:copy-of select="$np.res.pl-result"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$np.2nd"/>
      <slash dir="\" mode="&lt;"/>
      <xsl:copy-of select="$np.1st"/>
   </complexcat>
</xsl:variable>

<xsl:variable name="coord.np">
    <complexcat>
      <xsl:copy-of select="$np.res"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$np.2nd"/>
	  <slash dir="\" mode="&lt;"/>
      <xsl:copy-of select="$np.1st"/>
    </complexcat>
</xsl:variable>


  <xsl:template name="add-noun-families">

<!-- NOUN -->

 <family name="noun" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($n.coord)/*"/>
        <xsl:with-param name="ext" select="$ENTITY"/>
      </xsl:call-template>
    </entry>
  </family>

 <family name="bare-np" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($np)/*"/>
        <xsl:with-param name="ext" select="$ENTITY"/>
      </xsl:call-template>
    </entry>
  </family>


<!-- COORDINATION -->

<family name="coord.noun.pl-result" pos="COORD"  indexRel="First" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($coord.n.pl-result)/*"/>
        <xsl:with-param name="ext" select="$FIRST.NEXT"/>
      </xsl:call-template>
    </entry>
  </family>
   
 <family name="coord.noun" pos="COORD"  indexRel="First" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($coord.n)/*"/>
        <xsl:with-param name="ext" select="$FIRST.NEXT"/>
      </xsl:call-template>
    </entry>
  </family>

<family name="coord.np.pl-result" pos="COORD"  indexRel="First" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($coord.np.pl-result)/*"/>
        <xsl:with-param name="ext" select="$FIRST.NEXT"/>
      </xsl:call-template>
    </entry>
  </family>
   
 <family name="coord.np" pos="COORD"  indexRel="First" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($coord.np)/*"/>
        <xsl:with-param name="ext" select="$FIRST.NEXT"/>
      </xsl:call-template>
    </entry>
  </family>


  </xsl:template>

</xsl:transform>




