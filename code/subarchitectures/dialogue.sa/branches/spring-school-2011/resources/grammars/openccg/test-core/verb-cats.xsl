<?xml version="1.0"?>

<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">


 <xsl:variable name="s">
	<atomcat type="s">
	  <fs id="10">
    	<feat attr="index"><lf><nomvar name="E0"/></lf></feat>
<!--        <feat attr="vform"><lf><nomvar name="VFORM11:vform-vals"/></lf></feat>
        <feat attr="num"><featvar name="NUM2:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS2:pers-vals"/></feat> -->
	  </fs>
     </atomcat>
  </xsl:variable>


<!-- MOVE ME!!!!!!! -->
 <xsl:variable name="s.from">
	<atomcat type="s">
      <fs inheritsFrom="10">
      </fs>
	</atomcat>
  </xsl:variable>


 <xsl:variable name="s.main">
	<atomcat type="s">
	  <fs id="11">
    	<feat attr="index"><lf><nomvar name="E"/></lf></feat>
<!--        <feat attr="vform"><lf><nomvar name="VFORM11:vform-vals"/></lf></feat>
        <feat attr="num"><featvar name="NUM2:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS2:pers-vals"/></feat> -->
	  </fs>
     </atomcat>
  </xsl:variable>


  <!-- iv -->

 <xsl:variable name="verb">
    <complexcat>
	  <xsl:copy-of select="$s.main"/>
      <xsl:copy-of select="$with-subj"/>
   	</complexcat>
 </xsl:variable>  

<!-- tv -->

  <xsl:variable name="verb.np">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$np.V1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

<!-- feel, + copular -->

 <xsl:variable name="verb.adj">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$adj.V1"/>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

<!-- copular -->

<xsl:variable name="verb.pp">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$pp.V1"/>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>


<!-- resultant -->

  <xsl:variable name="verb.np.adj">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$adj.V2"/> 
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$np.V1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>



<!-- caused motion -->

  <xsl:variable name="verb.np.pp.pp">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$pp.V3"/> 
        <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$pp.V2"/> 
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$np.V1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

<!-- caused motion -->

  <xsl:variable name="verb.np.pp">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$pp.V3"/> 
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$np.V1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

</xsl:transform>
