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
  <!-- CATEGORY       DEFINITIONS                       -->
  <!-- ================================================ -->  



<!-- all preps for events can modify s either in-front or after, as well as trail vps to allow 
	things like extraction from coordinated questions "What did he pick up and give to GJ?" -->

 
  <xsl:variable name="prep.mod-s">
    <complexcat>
      <xsl:copy-of select="$s.generic"/>
      <setarg>
	    <slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$np.mod-arg"/>
        <slash dir="|" mode="*"/>
        <xsl:copy-of select="$s.generic"/>
	  </setarg>
   </complexcat>
  </xsl:variable>   
 
 <xsl:variable name="prep.mod-vp">
   <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($vp.generic)/*"/>
      <xsl:with-param name="ext">
	  <setarg>	
	     <slash dir="/" mode="&gt;"/>
         <xsl:copy-of select="$np.mod-arg"/>
         <slash dir="\" mode="*"/>
         <xsl:copy-of select="$vp.generic"/>
 	  </setarg>
	</xsl:with-param>     
  </xsl:call-template>
</xsl:variable>
 
<!--

  <xsl:variable name="prep.mod-s">
    <complexcat>
      <xsl:copy-of select="$s.generic"/>
	  <slash dir="|" mode="&gt;"/>
      <xsl:copy-of select="$s.generic"/>
      <slash dir="/" mode="*"/>
      <xsl:copy-of select="$np.mod-arg"/>
	</complexcat>
  </xsl:variable>   
 
  
  <xsl:variable name="prep.mod-vp">
   	  <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.forward)/*"/>
        <xsl:with-param name="ext">
	       <slash dir="/" mode="&gt;"/>
		   <xsl:copy-of select="$np.mod-arg"/>
		</xsl:with-param>     
	 </xsl:call-template>
  </xsl:variable>
--> 

 <xsl:variable name="prep.mod-s.of-np">
   <complexcat>
      <xsl:copy-of select="$s.generic"/>
	  <setarg>
	    <slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$of.mod-arg"/>
        <slash dir="|" mode="*"/>
        <xsl:copy-of select="$s.generic"/>
	  </setarg>
   </complexcat>
 </xsl:variable>   
  <xsl:variable name="prep.mod-vp.of-np">
      <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($vp.generic)/*"/>
      <xsl:with-param name="ext">
	  <setarg>	
	     <slash dir="/" mode="&gt;"/>
         <xsl:copy-of select="$of.mod-arg"/>
         <slash dir="\" mode="*"/>
         <xsl:copy-of select="$vp.generic"/>
 	  </setarg>
	</xsl:with-param>     
  </xsl:call-template>
  </xsl:variable>

 
 <xsl:variable name="prep.mod-s_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
 <xsl:variable name="prep.mod-vp_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
 
 <xsl:variable name="prep.mod-s.of-np_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s.of-np)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
 <xsl:variable name="prep.mod-vp.of-np_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp.of-np)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

<xsl:variable name="prep.mod-s.of-np_W_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s.of-np)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.2"/>
		  <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/>  
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
 <xsl:variable name="prep.mod-vp.of-np_W_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp.of-np)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.2"/>
		  <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/>  
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>



<!-- May need something like this to allow backward vp 
     Though doesn't seem to work....
  <xsl:variable name="prep.mod-vp">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.forward)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="^"/>
         <xsl:copy-of select="$np.mod-arg"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>   -->
   
   
   
   
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->
  <!-- Noun modifying Preposition                       --> 
  <!-- CAT: (n \ n) / np								--> 
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->

  
 <xsl:variable name="prep.mod-n">
    <complexcat>
   <!--     <xsl:copy-of select="$n.from-generic.non-modifiable"/> -->
	  <xsl:copy-of select="$n.from-generic"/>
	  <setarg>
        <slash dir="\" mode="*"/>
        <xsl:copy-of select="$n.generic"/>
        <slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$np.mod-arg"/>
	  </setarg>
    </complexcat>
  </xsl:variable>   

 <xsl:variable name="prep.mod-n_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>  


  <xsl:variable name="prep.mod-n.of-np">
    <complexcat>
      <xsl:copy-of select="$n.generic"/>
	  <setarg>
        <slash dir="\" mode="*"/>
        <xsl:copy-of select="$n.generic"/>
        <slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$of.mod-arg"/>
   	  </setarg>
	 </complexcat>
  </xsl:variable>  
  
   <xsl:variable name="prep.mod-n.of-np_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n.of-np)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>  

  <xsl:variable name="prep.mod-n.of-np_W_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n.of-np)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.2"/>
		  <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/>  
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>  

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->
  <!-- Noun modifying Preposition                       --> 
  <!-- CAT: (np \ np) / np								--> 
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->

<!-- needed to handle "something on the table"  which on the table ... 

	MARK NPs as type MOD-BARE so won't apply to generic np
-->

 <xsl:variable name="prep.mod-np">
    <complexcat>
      <xsl:copy-of select="$np.from-generic"/>
  	  <setarg>
        <slash dir="\" mode="*"/>
        <xsl:copy-of select="$np.generic.modifiable"/>
        <slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$np.mod-arg"/>
   	  </setarg>
	 </complexcat>
  </xsl:variable>   

 <xsl:variable name="prep.mod-np_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-np)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>  


<xsl:variable name="prep.mod-np.of-np">
    <complexcat>
      <xsl:copy-of select="$np.from-generic"/>
   	  <setarg>
        <slash dir="\" mode="*"/>
        <xsl:copy-of select="$np.generic.modifiable"/>
        <slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$of.mod-arg"/>
   	  </setarg>
	 </complexcat>
  </xsl:variable>  
  
   <xsl:variable name="prep.mod-np.of-np_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-np.of-np)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>  

  <xsl:variable name="prep.mod-np.of-np_W_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-np.of-np)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.2"/>
		  <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/>  
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>  


  
  <!-- this is used as argument for copular as well as for 'here' in 'the man here says ... ' -->
  <xsl:variable name="prep.mod-n.saturated">
    <complexcat>
      <xsl:copy-of select="$n.generic"/>
      <slash dir="\" mode="*"/>
      <xsl:copy-of select="$n.generic"/>
    </complexcat>
  </xsl:variable>   

   <xsl:variable name="prep.mod-n.saturated_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n.saturated)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>  
 
  <xsl:variable name="prep.mod-np.saturated">
    <complexcat>
      <xsl:copy-of select="$np.from-generic"/>
      <slash dir="\" mode="*"/>
      <xsl:copy-of select="$np.generic.modifiable"/>
    </complexcat>
  </xsl:variable>   

   <xsl:variable name="prep.mod-np.saturated_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-np.saturated)/*"/>
      <xsl:with-param name="ext">
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>  

 

<!-- ugly! Solve this in a better way! Don't want "bare prepositions" 
     if keep them, need to integrate the basic categories for np and OBL
	 i.e. put them together. They Shouldn't be here!
	    Maybe have a "verb slots" and "verb roles" file
		Essentially the case-marking OBLs and nps are both "thing" referring expressions
-->

 <xsl:variable name="obl">
	<atomcat type="OBL">
      <fs id="30">
	  <feat attr="index"><lf><nomvar name="TMA"/></lf></feat>
	<!--  <feat attr="pform" val="[*DEFAULT*]"/>  WE WANT THIS, but NEED PRED, NOT DEFAULT--> 
	  </fs>
    </atomcat>
 </xsl:variable>



<!--  ^ mode needed to allow what questions "into" the oblique 
	 ex.  What did she play with 
 -->
	
 <xsl:variable name="prep.oblique">
    <complexcat>
	<xsl:copy-of select="$obl"/>
	<slash dir="/" mode="&gt;"/>   
    <xsl:copy-of select="$np.mod-arg"/>
    </complexcat>
  </xsl:variable>


  <!-- ================================================ -->  
  <!-- FAMILY         DEFINITIONS                       -->
  <!-- ================================================ -->  


  <xsl:template name="add-preposition-and-oblique-families">


 <!-- Oblique -->

 <family name="prep-oblique" pos="PREP" indexRel= "*NoSem*" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.oblique)/*"/>
      </xsl:call-template>
    </entry>
  </family>


  <!-- MOD-S -->


<!-- various whereto families for different kinds of args & mwe -->
  <family name="prep-mod-s.whereto" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHERETO"/>
      </xsl:call-template>
    </entry>
   	<entry name="vp">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHERETO"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="prep-mod-s.whereto_W" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHERETO"/>
      </xsl:call-template>
    </entry>
   	<entry name="vp">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHERETO"/>
      </xsl:call-template>
    </entry>
  </family>
 
  <family name="prep-mod-s.of-np.whereto" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s.of-np)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHERETO"/>
      </xsl:call-template>
    </entry>
    <entry name="vp">
        <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp.of-np)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHERETO"/>
      </xsl:call-template>
    </entry>
  </family>

  <family name="prep-mod-s.of-np.whereto_W" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s.of-np_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHERETO"/>
      </xsl:call-template>
    </entry>
    <entry name="vp">
        <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp.of-np_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHERETO"/>
      </xsl:call-template>
    </entry>
  </family>

  <family name="prep-mod-s.of-np.whereto_W_W" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s.of-np_W_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHERETO"/>
      </xsl:call-template>
    </entry>
    <entry name="vpy">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp.of-np_W_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHERETO"/>
      </xsl:call-template>
    </entry>
  </family>
  
  <!-- various wherefrom families for different kinds of args & mwe -->
  
  <family name="prep-mod-s.wherefrom" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHEREFROM"/>
      </xsl:call-template>
    </entry>
    <entry name="vp">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHEREFROM"/>
      </xsl:call-template>
    </entry>
  </family>

  <family name="prep-mod-s.wherefrom_W" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHEREFROM"/>
      </xsl:call-template>
    </entry>
    <entry name="vp">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHEREFROM"/>
      </xsl:call-template>
    </entry>
  </family>
  
   <family name="prep-mod-s.through" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.THROUGH"/>
      </xsl:call-template>
    </entry>
    <entry name="vp">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.THROUGH"/>
      </xsl:call-template>
    </entry>
  </family>
 
   <family name="prep-mod-s.location" pos="PREP" closed="true">
    <entry name="sent">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
    <entry name="vp">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="prep-mod-s.location_W" pos="PREP" closed="true">
    <entry name="sent">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
    <entry name="vp">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
  </family>


  <family name="prep-mod-s.of-np.location" pos="PREP" closed="true">
    <entry name="sent">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s.of-np)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
    <entry name="vp">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp.of-np)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
  </family>

  <family name="prep-mod-s.of-np.location_W" pos="PREP" closed="true">
    <entry name="sent">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s.of-np_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
	<entry name="vp">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp.of-np_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
  </family>
  
  <family name="prep-mod-s.of-np.location_W_W" pos="PREP" closed="true">
    <entry name="sent">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s.of-np_W_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
    <entry name="vp">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp.of-np_W_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
  </family>

  <family name="prep-mod-s.accompaniment" pos="PREP" indexRel="Accompaniment" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.ACCOMPANIMENT"/>
      </xsl:call-template>
    </entry>
    <entry name="vp">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.ACCOMPANIMENT"/>
      </xsl:call-template>
    </entry>
  </family>

  <family name="prep-mod-s.benefactor" pos="PREP" indexRel="Benefactor" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.BENEFACTOR"/>
      </xsl:call-template>
    </entry>
    <entry name="vp">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.BENEFACTOR"/>
      </xsl:call-template>
    </entry>
  </family>

 <family name="prep-mod-s.purpose" pos="PREP" indexRel="Purpose" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.PURPOSE-ENTITY"/>
      </xsl:call-template>
    </entry>
    <entry name="vp">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.PURPOSE-ENTITY"/>
      </xsl:call-template>
    </entry>
 </family>

  <family name="prep-mod-s.comparison" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-s)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.COMPARISION-ENTITY"/>
      </xsl:call-template>
    </entry>
   <entry name="vp">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-vp)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.COMPARISION-ENTITY"/>
      </xsl:call-template>
    </entry>
  </family>


<!-- MOD-N -->

<!-- used in copular as in "I am here"
	better than making it them nps because then get loads weird readings -->
  
  <family name="here-and-there" pos="PREP" closed="true">
     <entry name="n">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n.saturated)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.LOCATION"/>
      </xsl:call-template>
    </entry>
    <entry name="np">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-np.saturated)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.LOCATION"/>
      </xsl:call-template>
    </entry>
  </family> 
  <family name="here-and-there_W" pos="PREP" closed="true">
     <entry name="n">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n.saturated_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.LOCATION"/>
      </xsl:call-template>
    </entry>
     <entry name="np">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-np.saturated_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.LOCATION"/>
      </xsl:call-template>
    </entry>
  </family> 



  <family name="prep-mod-n.comparison" pos="PREP" closed="true">
    <entry name="n">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.COMPARISON-ENTITY"/>
      </xsl:call-template>
    </entry>
	<entry name="np">  <!--needed to handle "something like a ball" -->
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-np)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.COMPARISON-ENTITY"/>
      </xsl:call-template>
    </entry>
  </family>

  <family name="prep-mod-n.owner" pos="PREP" closed="true">
    <entry name="n">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.OWNER"/>
      </xsl:call-template>
    </entry>
	<entry name="np">  <!--needed to handle "GJ's ball" -->
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-np)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.OWNER"/>
      </xsl:call-template>
    </entry>
  </family>

  
  <family name="prep-mod-n.location" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
    <entry name="np">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-np)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="prep-mod-n.location_W" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
    <entry name="np">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-np_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
  </family>


  <family name="prep-mod-n.of-np.location" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n.of-np)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
	<entry name="np">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-np.of-np)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
  </family>
  
  <family name="prep-mod-n.of-np.location_W" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n.of-np_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
    <entry name="np">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-np.of-np_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="prep-mod-n.of-np.location_W_W" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n.of-np_W_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
      <entry name="np">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-np.of-np_W_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
  </family>
  
  
   <family name="prep-mod-n.benefactor" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.BENEFACTOR"/>
      </xsl:call-template>
    </entry>
    <entry name="np">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-np)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.BENEFACTOR"/>
      </xsl:call-template>
    </entry>
  </family>

 <family name="prep-mod-n.accompaniment" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.ACCOMPANIMENT"/>
      </xsl:call-template>
    </entry>
    <entry name="np">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-np)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.ACCOMPANIMENT"/>
      </xsl:call-template>
    </entry>
  </family>
 
 
  
  <!--  
  <family na me="prep-mod-physical-n.accompaniment" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n)/*"/>
        <xsl:with-param name="ext" select="$MOD-T-PHYSICAL.ACCOMPANIMENT"/>
      </xsl:call-template>
    </entry>
  </family>
  
  
  <family na me="prep-mod-physical-n.location" pos="PREP" closed="true">
    <entry name="Primary">
       <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prep.mod-n)/*"/>
        <xsl:with-param name="ext" select="$MOD-T-PHYSICAL.LOCATION-WITH-ANCHOR"/>
      </xsl:call-template>
    </entry>
  </family>
-->




  </xsl:template>

</xsl:transform>