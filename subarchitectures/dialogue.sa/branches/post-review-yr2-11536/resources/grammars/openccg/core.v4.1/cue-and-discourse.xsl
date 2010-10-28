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



<!-- ================================================ -->
<!-- BASIC CATEGORY DEFINITIONS                      -->
<!-- ================================================ -->


<!-- <xsl:variable name="du.generic">
   <atomcat type="du">
     <fs id="40">
       <feat attr="index"><lf><nomvar name="D"/></lf></feat>
     </fs>
   </atomcat>
</xsl:variable> 
-->

<!-- should this be as special category, and if it remains S, should it be finite?  
     REALLY THIS SHOULD HANLDLE ANYTHING!!!   Yes, one.  Yes, I want a couple, Yes, a ball, Yes, on the table .... -->
	 
<xsl:variable name="du.generic">
   
     <xsl:copy-of select="$s.main.fin"/>
   
</xsl:variable>

<xsl:variable name="du.generic_W">
  <complexcat>
    <xsl:copy-of select="$du.generic"/>
	<slash dir="/" mode="*"/>
 	<xsl:copy-of select="$word.1"/> 
  </complexcat>
</xsl:variable>



<!-- Discourse Unit Argument slot categories -->

<xsl:variable name="s.du-arg">
   <atomcat type="s">
     <fs id="41">
       <feat attr="index"><lf><nomvar name="EDA"/></lf></feat>
     </fs>
   </atomcat>
</xsl:variable>

<xsl:variable name="s.du-arg.non-fin">
   <atomcat type="s">
     <fs id="41">
	   <feat attr="index"><lf><nomvar name="EDA"/></lf></feat>
       <feat attr="vform" val="non-fin"/>	
	 </fs>
   </atomcat>
</xsl:variable>

<xsl:variable name="np.du-arg">
   <atomcat type="np">
     <fs id="42">
       <feat attr="index"><lf><nomvar name="TDA"/></lf></feat>
     </fs>
   </atomcat>
</xsl:variable>

<!-- used for Let's -->
<xsl:variable name="vp.du-arg.subj-controlled.non-fin">
   <complexcat>
     <xsl:copy-of select="$s.du-arg.non-fin"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>

<!-- ================================================ -->
<!-- COMPLEX CATEGORY DEFINITIONS                      -->
<!-- ================================================ -->

<!-- changed result type from du to s...allows nesting, and sentences like "he said yes" -->

<xsl:variable name="du.s">
    <complexcat>
      <xsl:copy-of select="$du.generic"/> 
    <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$s.du-arg"/>  
	</complexcat>
  </xsl:variable>

<xsl:variable name="du.s_W">
   <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($du.s)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="*"/>
 	    <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
</xsl:variable>


<xsl:variable name="du.subj-conrolled-non-fin-vp">
    <complexcat>
      <xsl:copy-of select="$du.generic"/> 
    <slash dir="/" mode="*"/>
      <xsl:copy-of select="$vp.du-arg.subj-controlled.non-fin"/>  
	</complexcat>
  </xsl:variable>

<xsl:variable name="du.np">
    <complexcat>
      <xsl:copy-of select="$du.generic"/> 
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$np.du-arg"/>  
	</complexcat>
</xsl:variable>
  
<!-- ================================================ -->
<!-- SEMANTIC DEFINITIONS                             -->
<!-- ================================================ -->

<xsl:variable name="POSITIVE">
   <diamond mode="Polarity"><prop name="+"/></diamond>
</xsl:variable>
<xsl:variable name="NEGATIVE">
   <diamond mode="Polarity"><prop name="-"/></diamond>
</xsl:variable>

<!-- DUs -->

<xsl:variable name="DIS.BASE">
	<satop nomvar="E">   <!-- was dis-unit D, now generic sentence -->
	   <prop name="[*DEFAULT*]"/>
	</satop>
</xsl:variable>

<xsl:variable name="DIS.DEFAULT">
  <lf>
    <xsl:copy-of select="$DIS.BASE"/>
  </lf>
</xsl:variable>


<xsl:variable name="DIS.POSITIVE">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($DIS.BASE)/*"/>
    <xsl:with-param name="ext">
	    <xsl:copy-of select="$POSITIVE"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    
<xsl:variable name="DIS.NEGATIVE">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($DIS.BASE)/*"/>
    <xsl:with-param name="ext">
	    <xsl:copy-of select="$NEGATIVE"/>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<xsl:variable name="DIS.ADDRESSEE">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($DIS.BASE)/*"/>
    <xsl:with-param name="ext">
	    <diamond mode="Addressee"><nomvar name="TDA"/></diamond>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<!-- EVENT MODIFYING DUs -->

<xsl:variable name="DIS.BODY">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($DIS.BASE)/*"/>
    <xsl:with-param name="ext">	
         <diamond mode="Body"><nomvar name="EDA"/></diamond>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<xsl:variable name="DIS.SUGGESTION">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($DIS.BASE)/*"/>
    <xsl:with-param name="ext">
          <diamond mode="Mood"><prop name="suggestion"/></diamond>
		 <diamond mode="Actor">
		      <nomvar name="X"/>
		      <prop name="we"/>  
		 </diamond>
         <diamond mode="Body"><nomvar name="EDA"/></diamond>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>   

<xsl:variable name="DIS-EVENT-POLARITY.POSITIVE">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($DIS.BASE)/*"/>
    <xsl:with-param name="ext">
	    <xsl:copy-of select="$POSITIVE"/>
		<diamond mode="Scope"><nomvar name="EDA"/></diamond>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    

<xsl:variable name="DIS-EVENT-POLARITY.NEGATIVE">
  <lf>
    <xsl:call-template name="extend">
    <xsl:with-param name="elt" select="xalan:nodeset($DIS.BASE)/*"/>
    <xsl:with-param name="ext">
	    <xsl:copy-of select="$NEGATIVE"/>
		<diamond mode="Scope"><nomvar name="EDA"/></diamond>
    </xsl:with-param>
    </xsl:call-template>
  </lf>
</xsl:variable>    


<!-- ================================================ -->
<!-- FAMILY DEFINITIONS                               -->
<!-- ================================================ -->

<xsl:template name="add-cue-and-discourse-families">

<!-- function specified by lexical entries class, ex greeting, acknowledgement, rejection -->
<family name="dis-unit" pos="DU" closed="true">
   <entry name="basic">
      <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($du.generic)/*"/>
      <xsl:with-param name="ext" select="$DIS.DEFAULT"/>
    </xsl:call-template>
  </entry>
</family> 

<family name="dis-unit_W" pos="DU" closed="true">
   <entry name="basic">
      <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($du.generic_W)/*"/>
      <xsl:with-param name="ext" select="$DIS.DEFAULT"/>
    </xsl:call-template>
  </entry>
</family> 
  
<family name="dis-unit.addressee" pos="DU" closed="true">
   <entry name="basic">
      <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($du.np)/*"/>
      <xsl:with-param name="ext" select="$DIS.ADDRESSEE"/>
    </xsl:call-template>
  </entry>
</family> 

<family name="dis-unit.connective" pos="DU" closed="true">
   <entry name="basic">
      <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($du.s)/*"/>
      <xsl:with-param name="ext" select="$DIS.BODY"/>
    </xsl:call-template>
  </entry>
</family> 

<family name="dis-unit.connective_W" pos="DU" closed="true">
   <entry name="basic">
      <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($du.s_W)/*"/>
      <xsl:with-param name="ext" select="$DIS.BODY"/>
    </xsl:call-template>
  </entry>
</family> 



<family name="dis-unit.suggestion" pos="DU" closed="true">
   <entry name="basic">
      <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($du.subj-conrolled-non-fin-vp)/*"/>
      <xsl:with-param name="ext" select="$DIS.SUGGESTION"/>
    </xsl:call-template>
  </entry>
</family> 


<family name="sentence-polarity.positive" pos="DU" closed="true">
   <entry name="basic">
      <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($du.s)/*"/>
      <xsl:with-param name="ext" select="$DIS-EVENT-POLARITY.POSITIVE"/>
    </xsl:call-template>
  </entry>
</family> 

<family name="sentence-polarity.negative" pos="DU" closed="true">
   <entry name="basic">
      <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($du.s)/*"/>
      <xsl:with-param name="ext" select="$DIS-EVENT-POLARITY.NEGATIVE"/>
    </xsl:call-template>
  </entry>
</family> 
  
<family name="cue.positive" pos="DU" closed="true">
   <entry name="primary">
      <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($du.generic)/*"/>
      <xsl:with-param name="ext" select="$DIS.POSITIVE"/>
    </xsl:call-template>
  </entry>
</family>  
  
<family name="cue.negative" pos="DU" closed="true">
   <entry name="primary">
      <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($du.generic)/*"/>
      <xsl:with-param name="ext" select="$DIS.NEGATIVE"/>
    </xsl:call-template>
  </entry>
</family>  

 </xsl:template>

</xsl:transform>


  