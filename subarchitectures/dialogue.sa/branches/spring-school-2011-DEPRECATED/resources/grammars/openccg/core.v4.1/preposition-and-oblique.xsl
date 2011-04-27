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

 <xsl:variable name="pp">  
	<atomcat type="pp">
      <fs id="25">
        <feat attr="index"><lf><nomvar name="M"/></lf></feat>
        <feat attr="modifiable" val="+"/>
        <feat attr="mod-type"><featvar name="MTYPE:s-modifier-types"/></feat>
		<feat attr="complexcat-type"><featvar name="CCTYPE:complex-cat-types"/></feat>
      </fs>
    </atomcat>
  </xsl:variable>

<!-- these two are used for 'bare pps' like 'to the right' or 'pro-pps' like 'here, there' -->

 <xsl:variable name="pp_W">
   <complexcat>
      <xsl:copy-of select="$pp"/>
      <slash dir="/" mode="^"/>
      <xsl:copy-of select="$word.1"/>
   </complexcat>
 </xsl:variable>   

 <xsl:variable name="pp_W_W">
   <complexcat>
      <xsl:copy-of select="$pp"/>
      <slash dir="/" mode="^"/>
      <xsl:copy-of select="$word.2"/>
      <slash dir="/" mode="^"/>
      <xsl:copy-of select="$word.1"/>
   </complexcat>
 </xsl:variable>   


<!-- A Preposition is a pp waithing for its np arg -->

 <xsl:variable name="prep">
   <complexcat>
      <xsl:copy-of select="$pp"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$np.mod-arg"/>
   </complexcat>
 </xsl:variable>   
 
 <xsl:variable name="prep.of-np">
   <complexcat>
      <xsl:copy-of select="$pp"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$of.mod-arg"/>
   </complexcat>
 </xsl:variable>   
 
  <xsl:variable name="prep_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="^"/>
         <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

  <xsl:variable name="prep.of-np_W_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep.of-np)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="^"/>
         <xsl:copy-of select="$word.2"/> 
		 <slash dir="/" mode="^"/>
         <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  
 <xsl:variable name="prep.of-np_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep.of-np)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="^"/>
         <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

<!-- OBLIQUE 
	
		Oblique head inhereits all from its np arg. Really just 'marking' the np

-->

 <xsl:variable name="obl">
	<atomcat type="OBL">
      <fs id="30">
	  <feat attr="index"><lf><nomvar name="TMA"/></lf></feat> 
	<!--  <feat attr="pform" val="[*DEFAULT*]"/>  WE WANT THIS, but NEED PRED, NOT DEFAULT--> 
	  </fs>
    </atomcat>
 </xsl:variable>

 <xsl:variable name="prep.oblique">
    <complexcat>
	<xsl:copy-of select="$obl"/>
	<slash dir="/" mode="&gt;"/>   
    <xsl:copy-of select="$np.mod-arg"/>
    </complexcat>
  </xsl:variable>


<!-- Semantically Restricted PP cats -->

 <xsl:variable name="pp-common-feats.generic">  
        <feat attr="index"><lf><nomvar name="M0"/></lf></feat>
        <feat attr="modifiable" val="+"/>
 </xsl:variable>

 <xsl:variable name="pp.dynamic">  
	<atomcat type="pp">
      <fs id="26">
	    <xsl:copy-of select="$pp-common-feats.generic"/>
     	<feat attr="mod-type" val="s-dynamic"/>
      </fs>
    </atomcat>
  </xsl:variable>
  
  <xsl:variable name="pp.location">  
	<atomcat type="pp">
      <fs id="26">
	    <xsl:copy-of select="$pp-common-feats.generic"/>
		<feat attr="mod-type" val="s-location"/>
      </fs>
    </atomcat>
  </xsl:variable> 

<!-- MODIFIER -->

 <xsl:variable name="mod-right.pp">
   <complexcat>
      <xsl:copy-of select="$pp"/>
      <slash dir="/" mode="^"/>
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
      <xsl:copy-of select="$with-dollar"/>
	  <slash dir="/" mode="&gt;"/>
      <complexcat>
         <xsl:copy-of select="$pp.2nd"/>
	     <xsl:copy-of select="$with-dollar"/>
      </complexcat>
      <slash dir="\" mode="&lt;"/>
      <complexcat>
         <xsl:copy-of select="$pp.1st"/>
         <xsl:copy-of select="$with-dollar"/>
      </complexcat>
   </complexcat>
</xsl:variable>  


<xsl:template name="add-preposition-and-oblique-families">

<!-- OBLIQUES -->

 <family name="prep-oblique" pos="PREP" indexRel= "*NoSem*" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.oblique)/*"/>
      </xsl:call-template>
    </entry>
  </family>

<!-- PREPOSITION -->

 <family name="prep" pos="PREP" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($prep)/*"/>
        <xsl:with-param name="ext" select="$MOD.ARG"/>
      </xsl:call-template>
    </entry>
 </family>

 <family name="prep_W" pos="PREP" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($prep_W)/*"/>
        <xsl:with-param name="ext" select="$MOD.ARG"/>
      </xsl:call-template>
    </entry>
 </family>

 <family name="prep.of-np" pos="PREP" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($prep.of-np)/*"/>
        <xsl:with-param name="ext" select="$MOD.ARG"/>
      </xsl:call-template>
    </entry>
 </family>
 
 <family name="prep.of-np_W" pos="PREP" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($prep.of-np_W)/*"/>
        <xsl:with-param name="ext" select="$MOD.ARG"/>
      </xsl:call-template>
    </entry>
 </family>

 <family name="prep.of-np_W_W" pos="PREP" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($prep.of-np_W_W)/*"/>
        <xsl:with-param name="ext" select="$MOD.ARG"/>
      </xsl:call-template>
    </entry>
 </family>

 <family name="prep.no-arg" pos="PREP" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($pp)/*"/>
        <xsl:with-param name="ext" select="$MOD"/>
      </xsl:call-template>
    </entry>
 </family>

 <family name="prep.no-arg_W" pos="PREP" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($pp_W)/*"/>
        <xsl:with-param name="ext" select="$MOD"/>
      </xsl:call-template>
    </entry>
 </family>

 <family name="prep.no-arg_W_W" pos="PREP" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($pp_W_W)/*"/>
        <xsl:with-param name="ext" select="$MOD"/>
      </xsl:call-template>
    </entry>
 </family>


<!-- COORDINATION -->

 <family name="coord.prep" pos="COORD"  indexRel="First" closed="true">
    <entry name="primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($coord.pp)/*"/>
        <xsl:with-param name="ext" select="$FIRST.NEXT"/>
      </xsl:call-template>
    </entry>
 </family>

 <!-- MODIFICATION -->

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






 <!-- TYPE CHANGING RULES -->

<xsl:template name="add-prep-rules">
 
 
<!-- LOCATION --> 
 
<typechanging name="prep-n-location"> 
   <arg> 
    <complexcat>
	  <atomcat type="pp">
        <fs id="26">
	      <xsl:copy-of select="$pp-common-feats.generic"/>
		  <feat attr="mod-type" val="s-location"/>
		  <feat attr="complexcat-type" val="n-left"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
     <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-n.left)/*"/>
        <xsl:with-param name="ext">	
            <xsl:copy-of select="$MOD-T-XLOCATION"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>
 
<typechanging name="prep-np-location"> 
   <arg> 
    <complexcat>
	   <atomcat type="pp">
        <fs id="26">
          <xsl:copy-of select="$pp-common-feats.generic"/>
		  <feat attr="mod-type" val="s-location"/>
		  <feat attr="complexcat-type" val="n-left"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
     </complexcat>
   </arg> 
   <result>
     <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-np.left)/*"/>
        <xsl:with-param name="ext">	
            <xsl:copy-of select="$MOD-T-XLOCATION"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<typechanging name="prep-vp-location"> 
   <arg> 
    <complexcat>
	   <atomcat type="pp">
        <fs id="26">
	      <xsl:copy-of select="$pp-common-feats.generic"/>
		  <feat attr="mod-type" val="s-location"/>
		  <feat attr="complexcat-type" val="vp-left"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-vp.left)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XLOCATION"/>
	    </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>
 
<typechanging name="prep-s-left-location"> 
   <arg> 
    <complexcat>
	 <atomcat type="pp">
        <fs id="26">
	      <xsl:copy-of select="$pp-common-feats.generic"/>
		  <feat attr="mod-type" val="s-location"/>
		  <feat attr="complexcat-type" val="s-left"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-s.left)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XLOCATION"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<typechanging name="prep-s-right-location"> 
   <arg> 
    <complexcat>
	 <atomcat type="pp">
        <fs id="26">
	      <xsl:copy-of select="$pp-common-feats.generic"/>
		  <feat attr="mod-type" val="s-location"/>
		  <feat attr="complexcat-type" val="s-left"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-s.right)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XLOCATION"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>


<!-- DYNAMIC -->
 
<typechanging name="prep-s-dynamic"> 
   <arg> 
    <complexcat>
	   <atomcat type="pp">
        <fs id="26">
	      <xsl:copy-of select="$pp-common-feats.generic"/>
		  <feat attr="mod-type" val="s-dynamic"/>
		  <feat attr="complexcat-type" val="s-left"/>
        </fs>
       </atomcat>
       <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-s.left)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XDYNAMIC"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<typechanging name="prep-vp-dynamic"> 
   <arg> 
    <complexcat> 
	  <atomcat type="pp">
        <fs id="26">
	      <xsl:copy-of select="$pp-common-feats.generic"/>
		  <feat attr="mod-type" val="s-dynamic"/>
		  <feat attr="complexcat-type" val="vp-left"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-vp.left)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XDYNAMIC"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<!-- COMPARISON --> 
 
<typechanging name="prep-n-comparison"> 
   <arg> 
    <complexcat>
	  <atomcat type="pp">
        <fs id="26">
	      <xsl:copy-of select="$pp-common-feats.generic"/>
		  <feat attr="mod-type" val="s-comparison"/>
		  <feat attr="complexcat-type" val="n-left"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
     <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-n.left)/*"/>
        <xsl:with-param name="ext">	
            <xsl:copy-of select="$MOD-T-XCOMPARISON"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>
 
<typechanging name="prep-np-comparison"> 
   <arg> 
    <complexcat>
	   <atomcat type="pp">
        <fs id="26">
          <xsl:copy-of select="$pp-common-feats.generic"/>
		  <feat attr="mod-type" val="s-comparison"/>
		  <feat attr="complexcat-type" val="n-left"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
     </complexcat>
   </arg> 
   <result>
     <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-np.left)/*"/>
        <xsl:with-param name="ext">	
            <xsl:copy-of select="$MOD-T-XCOMPARISON"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>

<typechanging name="prep-vp-comparison"> 
   <arg> 
    <complexcat>
	   <atomcat type="pp">
        <fs id="26">
	      <xsl:copy-of select="$pp-common-feats.generic"/>
		  <feat attr="mod-type" val="s-comparison"/>
		  <feat attr="complexcat-type" val="vp-left"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-vp.left)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XCOMPARISON"/>
	    </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>
 
<typechanging name="prep-s-comparison"> 
   <arg> 
    <complexcat>
	 <atomcat type="pp">
        <fs id="26">
	      <xsl:copy-of select="$pp-common-feats.generic"/>
		  <feat attr="mod-type" val="s-comparison"/>
		  <feat attr="complexcat-type" val="s-left"/>
        </fs>
      </atomcat>
      <xsl:copy-of select="$with-dollar"/>
    </complexcat>
   </arg> 
   <result>
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-s.left)/*"/>
        <xsl:with-param name="ext">	
           <xsl:copy-of select="$MOD-E-XCOMPARISON"/>
        </xsl:with-param>
      </xsl:call-template>
   </result> 
</typechanging>


</xsl:template>



<!-- COMPLEX FORMING RULES 
		now just built direct into rule below

 A crappy attempt to allow modifyig of incremental questions ...
 
 <xsl:variable name="mod-vp.left">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$np.subj"/>
		  <slash dir="/" mode="&gt;"/>
		  <xsl:copy-of select="$np.generic"/>
		  <slash/>
          <dollar name="1"/>
          <slash dir="\" mode="."/>
				<complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$np.subj"/>
		  <slash dir="/" mode="&gt;"/>
		  <xsl:copy-of select="$np.generic"/>
		</complexcat>
		
       </complexcat>
 </xsl:variable>   


-->




</xsl:transform>
