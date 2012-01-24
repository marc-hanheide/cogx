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

<!-- ATOMIC CATS

		See end of this file for complex cats (converted via unary rule)
 -->

 <xsl:variable name="adv">  
	<atomcat type="adv">
      <fs id="25">
        <feat attr="index"><lf><nomvar name="M"/></lf></feat>
        <feat attr="modifiable" val="+"/>
        <feat attr="mod-type"><featvar name="MTYPE:s-modifier-types"/></feat>
		<feat attr="complexcat-type"><featvar name="CCTYPE:complex-cat-types"/></feat>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="adv_W">
     <complexcat>
         <xsl:copy-of select="$adv"/> 		 
    	 <slash dir="/" mode="^"/>
         <xsl:copy-of select="$word.1"/> 
	 </complexcat>
  </xsl:variable>

 <xsl:variable name="adv_W_W">
     <complexcat>
         <xsl:copy-of select="$adv"/> 		 
         <slash dir="/" mode="^"/>
         <xsl:copy-of select="$word.2"/> 
		 <slash dir="/" mode="^"/>
         <xsl:copy-of select="$word.1"/> 
	</complexcat>
</xsl:variable>


<!-- MODIFIER 
	
		The slash type here seems to be an issue... with 
	
-->

 <xsl:variable name="mod-right.adv">
   <complexcat>
      <xsl:copy-of select="$adv"/>
      <slash dir="/" mode="^"/> 
      <xsl:copy-of select="$adv"/>
   </complexcat>
 </xsl:variable>   


<!-- COORDINATION -->

<xsl:variable name="adv.1st">
    <atomcat type="adv">
      <fs id="21">
	     <feat attr="index"><lf><nomvar name="C1"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="adv.2nd">
    <atomcat type="adv">
      <fs id="22">
        <feat attr="index"><lf><nomvar name="C2"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

<xsl:variable name="adv.res">
    <atomcat type="adv">
      <fs id="23">
        <feat attr="index"><lf><nomvar name="CR"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

<!-- stack over any args -->
<xsl:variable name="coord.adv">
    <complexcat>
      <xsl:copy-of select="$adv.res"/>
      <xsl:copy-of select="$with-dollar"/>
	  <slash dir="/" mode="&gt;"/>
      <complexcat>
         <xsl:copy-of select="$adv.2nd"/>
	     <xsl:copy-of select="$with-dollar"/>
      </complexcat>
      <slash dir="\" mode="&lt;"/>
      <complexcat>
         <xsl:copy-of select="$adv.1st"/>
         <xsl:copy-of select="$with-dollar"/>
      </complexcat>
   </complexcat>
</xsl:variable>  

<xsl:template name="add-adv-families">

<!-- ADV FAMILIES -->

 <family name="adv" pos="ADV" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($adv)/*"/>
        <xsl:with-param name="ext" select="$MOD"/>
      </xsl:call-template>
    </entry>
 </family>

 <family name="adv_W" pos="ADV" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($adv_W)/*"/>
        <xsl:with-param name="ext" select="$MOD"/>
      </xsl:call-template>
    </entry>
 </family>

 <family name="adv_W_W" pos="ADV" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($adv_W_W)/*"/>
        <xsl:with-param name="ext" select="$MOD"/>
      </xsl:call-template>
    </entry>
 </family>


<!-- COORDINATION -->

 <family name="coord.adv" pos="COORD"  indexRel="First" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($coord.adv)/*"/>
        <xsl:with-param name="ext" select="$FIRST.NEXT"/>
      </xsl:call-template>
    </entry>
 </family>

 <!-- MODIFICATION -->

<family name="negation.adv" pos="MOD"  indexRel="Polarity" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-right.adv)/*"/>
        <xsl:with-param name="ext" select="$MOD-NEGATION"/>
      </xsl:call-template>
    </entry>
</family>

<family name="mod-right.adv" pos="MOD"  closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-right.adv)/*"/>
        <xsl:with-param name="ext" select="$MOD-MODIFIER"/>
      </xsl:call-template>
    </entry>
 </family>

</xsl:template>








<xsl:variable name="adv-common-feats.generic">  
        <feat attr="index"><lf><nomvar name="M0"/></lf></feat>
        <feat attr="modifiable" val="+"/>
 </xsl:variable>


 <!-- TYPE CHANGING RULES -->

<xsl:template name="add-adv-rules">

<!-- DYNAMIC -->
 
 <typechanging name="adv-s-left-dynamic"> 
   <arg> 
    <complexcat>
	   <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-dynamic"/>
		  <feat attr="complexcat-type" val="s-left"/>
        </fs>
       </atomcat>
       <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-loose-s.left)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XDYNAMIC"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>
 
<typechanging name="adv-s-right-dynamic"> 
   <arg> 
    <complexcat>
	   <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-dynamic"/>
		  <feat attr="complexcat-type" val="s-right"/>
        </fs>
       </atomcat>
       <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-s.right)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XDYNAMIC"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<typechanging name="adv-vp-right-dynamic"> 
   <arg> 
    <complexcat> 
	  <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-dynamic"/>
		  <feat attr="complexcat-type" val="vp-right"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-vp.right)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XDYNAMIC"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<typechanging name="adv-vp-left-dynamic"> 
   <arg> 
    <complexcat> 
	  <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-dynamic"/>
		  <feat attr="complexcat-type" val="vp-left"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-loose-vp.left)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XDYNAMIC"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<!-- MANNER -->
 
 <typechanging name="adv-s-left-manner"> 
   <arg> 
    <complexcat>
	   <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-manner"/>
		  <feat attr="complexcat-type" val="s-left"/>
        </fs>
       </atomcat>
       <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-loose-s.left)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XMANNER"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>
 
<typechanging name="adv-s-right-manner"> 
   <arg> 
    <complexcat>
	   <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-manner"/>
		  <feat attr="complexcat-type" val="s-right"/>
        </fs>
       </atomcat>
       <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-s.right)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XMANNER"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<typechanging name="adv-vp-right-manner"> 
   <arg> 
    <complexcat> 
	  <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-manner"/>
		  <feat attr="complexcat-type" val="vp-right"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-vp.right)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XMANNER"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<typechanging name="adv-vp-left-manner"> 
   <arg> 
    <complexcat> 
	  <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-manner"/>
		  <feat attr="complexcat-type" val="vp-left"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-loose-vp.left)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XMANNER"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<!-- COMMENT -->
 
 <typechanging name="adv-s-left-comment"> 
   <arg> 
    <complexcat>
	   <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-comment"/>
		  <feat attr="complexcat-type" val="s-left"/>
        </fs>
       </atomcat>
       <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-s.left)/*"/> <!-- CURRENTLY NOT LOOSE, BLOCKING I am too big as adv reading -->
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XCOMMENT"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>
 
<typechanging name="adv-s-right-comment"> 
   <arg> 
    <complexcat>
	   <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-comment"/>
		  <feat attr="complexcat-type" val="s-right"/>
        </fs>
       </atomcat>
       <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-s.right)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XCOMMENT"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<typechanging name="adv-vp-right-comment"> 
   <arg> 
    <complexcat> 
	  <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-comment"/>
		  <feat attr="complexcat-type" val="vp-right"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-vp.right)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XCOMMENT"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<typechanging name="adv-vp-left-comment"> 
   <arg> 
    <complexcat> 
	  <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-comment"/>
		  <feat attr="complexcat-type" val="vp-left"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-loose-vp.left)/*"/>  <!-- loose or not, affects I am too big -->
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XCOMMENT"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<!-- PROBABILITY -->
 
 <typechanging name="adv-s-left-probability"> 
   <arg> 
    <complexcat>
	   <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-probability"/>
		  <feat attr="complexcat-type" val="s-left"/>
        </fs>
       </atomcat>
       <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-loose-s.left)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XPROBABILITY"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>
 
<typechanging name="adv-s-right-probability"> 
   <arg> 
    <complexcat>
	   <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-probability"/>
		  <feat attr="complexcat-type" val="s-right"/>
        </fs>
       </atomcat>
       <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-s.right)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XPROBABILITY"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<typechanging name="adv-vp-right-probability"> 
   <arg> 
    <complexcat> 
	  <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-probability"/>
		  <feat attr="complexcat-type" val="vp-right"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-vp.right)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XPROBABILITY"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<typechanging name="adv-vp-left-probability"> 
   <arg> 
    <complexcat> 
	  <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-probability"/>
		  <feat attr="complexcat-type" val="vp-left"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-loose-vp.left)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XPROBABILITY"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<!-- TIME -->
 
 <typechanging name="adv-s-left-time"> 
   <arg> 
    <complexcat>
	   <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-time"/>
		  <feat attr="complexcat-type" val="s-left"/>
        </fs>
       </atomcat>
       <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-loose-s.left)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XTIME"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>
 
<typechanging name="adv-s-right-time"> 
   <arg> 
    <complexcat>
	   <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-time"/>
		  <feat attr="complexcat-type" val="s-right"/>
        </fs>
       </atomcat>
       <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-s.right)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XTIME"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<typechanging name="adv-vp-right-time"> 
   <arg> 
    <complexcat> 
	  <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-time"/>
		  <feat attr="complexcat-type" val="vp-right"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-vp.right)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XTIME"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<typechanging name="adv-vp-left-time"> 
   <arg> 
    <complexcat> 
	  <atomcat type="adv">
        <fs id="26">
	      <xsl:copy-of select="$adv-common-feats.generic"/>
		  <feat attr="mod-type" val="s-time"/>
		  <feat attr="complexcat-type" val="vp-left"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-loose-vp.left)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XTIME"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

</xsl:template>

</xsl:transform>

