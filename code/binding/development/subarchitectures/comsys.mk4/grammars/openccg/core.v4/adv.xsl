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


<!-- These 3 categories allow "she quickly ran" "quickly she ran" "she ran quickly"	-->
<!-- NOTE:!!!! I just changed this to . (from >) to handle copular negatiion. If this is too
				 promiscous for others, then separate them -->
  
   <xsl:variable name="adv.mod-vp.forward">
   	  <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($vp.generic)/*"/>
        <xsl:with-param name="ext">
	       <slash dir="\" mode="."/>
		   <xsl:copy-of select="$vp.generic"/>
		</xsl:with-param>     
	 </xsl:call-template>
  </xsl:variable>

   <xsl:variable name="adv.mod-vp.backward">
   	  <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($vp.generic)/*"/>
        <xsl:with-param name="ext">
	       <slash dir="/" mode="&gt;"/>
		   <xsl:copy-of select="$vp.generic"/>
		</xsl:with-param>     
	 </xsl:call-template>
  </xsl:variable>


<!-- changed from purley initial, to initial and final for s. Needed to 
     handle imperative modification like "move quickly" because "move" is of type s
	 
	  NOTE: should change name to adv.mod-s and drop .initial...
	 -->
  <xsl:variable name="adv.mod-vp.initial">
    <complexcat>
        <xsl:copy-of select="$s.generic"/>	
		<slash dir="|" mode="^"/>
	    <xsl:copy-of select="$s.generic"/>
    </complexcat> 
  </xsl:variable>

<!-- These handle ADV with 1 MWE argument, ex. I_THINK, you_KNOW -->

 <xsl:variable name="adv.mod-vp.forward_W">
   	  <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.forward)/*"/>
        <xsl:with-param name="ext">
		   <slash dir="/" mode="*"/>
		   <xsl:copy-of select="$word.1"/> 
	    </xsl:with-param>     
	 </xsl:call-template>
  </xsl:variable>

   <xsl:variable name="adv.mod-vp.backward_W">
   	  <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.backward)/*"/>
        <xsl:with-param name="ext">
		   <slash dir="/" mode="*"/>
		   <xsl:copy-of select="$word.1"/> 
	    </xsl:with-param>     
	 </xsl:call-template>
  </xsl:variable>

  <xsl:variable name="adv.mod-vp.initial_W">
   <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.initial)/*"/>
        <xsl:with-param name="ext">
		   <slash dir="/" mode="*"/>
		   <xsl:copy-of select="$word.1"/> 
	    </xsl:with-param>     
	 </xsl:call-template>
  </xsl:variable>

<!-- These handle ADV with 2 MWE argument, ex. He ran to_the_left = he ran left -->

 <xsl:variable name="adv.mod-vp.forward_W_W">
   	  <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.forward)/*"/>
        <xsl:with-param name="ext">
		   <slash dir="/" mode="*"/>
		   <xsl:copy-of select="$word.2"/> 
		   <slash dir="/" mode="*"/>
		   <xsl:copy-of select="$word.1"/> 
	    </xsl:with-param>     
	 </xsl:call-template>
  </xsl:variable>

   <xsl:variable name="adv.mod-vp.backward_W_W">
   	  <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.backward)/*"/>
        <xsl:with-param name="ext">
           <slash dir="/" mode="*"/>
		   <xsl:copy-of select="$word.2"/> 
		   <slash dir="/" mode="*"/>
		   <xsl:copy-of select="$word.1"/> 
	    </xsl:with-param>     
	 </xsl:call-template>
  </xsl:variable>

  <xsl:variable name="adv.mod-vp.initial_W_W">
   <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.initial)/*"/>
        <xsl:with-param name="ext">
           <slash dir="/" mode="*"/>
		   <xsl:copy-of select="$word.2"/> 
		   <slash dir="/" mode="*"/>
		   <xsl:copy-of select="$word.1"/> 
	    </xsl:with-param>     
	 </xsl:call-template>
  </xsl:variable>
  
  <!-- ================================================ -->  
  <!-- LEXICAL FAMILY DEFINITIONS                       -->
  <!-- ================================================ -->  

  <xsl:template name="add-adv-families">
	
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->
  <!-- MUST EVENTUALLY BREAK UP INTO TO ALLOW GREATER CONTROL
       BUT CAN STILL KEEP THIS CAT AS DEFAULT          --> 
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->


<!-- Cop negation handled like normal forward adv, but has semantically restricted event argument 
     Aux negation handled differently because of NO-SUBJ strategy. See other.xsl -->
  <family name="copular-negation" pos="ADV" closed="true">
     <entry name="cop">
         <xsl:call-template name="extend">
         <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.forward)/*"/>
         <xsl:with-param name="ext" select="$MOD-E.NEGATION"/>		
         </xsl:call-template>
	   </entry>
  </family>

  <family name="adv.manner" pos="ADV" closed="true">
     <entry name="initial">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.initial)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.MANNER"/>
      </xsl:call-template>
     </entry>
	 <entry name="forward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.forward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.MANNER"/>
      </xsl:call-template>
     </entry>
	 <entry name="backward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.backward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.MANNER"/>
      </xsl:call-template>
     </entry>
  </family>
 
 <family name="adv.time" pos="ADV" closed="true">
     <entry name="initial">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.initial)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.TIME"/>
      </xsl:call-template>
     </entry>
	 <entry name="forward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.forward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.TIME"/>
      </xsl:call-template>
     </entry>
	 <entry name="backward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.backward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.TIME"/>
      </xsl:call-template>
     </entry>
  </family>

  
  <family name="adv.probability" pos="ADV" closed="true">
     <entry name="initial">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.initial)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.PROBABILITY"/>
      </xsl:call-template>
     </entry>
	 <entry name="forward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.forward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.PROBABILITY"/>
      </xsl:call-template>
     </entry>
	 <entry name="backward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.backward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.PROBABILITY"/>
      </xsl:call-template>
     </entry>
  </family>
  
  <family name="adv.frequency" pos="ADV" closed="true">
     <entry name="initial">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.initial)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.FREQUENCY"/>
      </xsl:call-template>
     </entry>
	 <entry name="forward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.forward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.FREQUENCY"/>
      </xsl:call-template>
     </entry>
	 <entry name="backward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.backward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.FREQUENCY"/>
      </xsl:call-template>
     </entry>
  </family>

  <family name="adv.comment" pos="ADV" closed="true">
     <entry name="initial">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.initial)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.COMMENT"/>
      </xsl:call-template>
     </entry>
	 <entry name="forward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.forward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.COMMENT"/>
      </xsl:call-template>
     </entry>
	 <entry name="backward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.backward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.COMMENT"/>
      </xsl:call-template>
     </entry>
  </family>
  
  <family name="adv.comment_W" pos="ADV" closed="true">
     <entry name="initial">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.initial_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.COMMENT"/>
      </xsl:call-template>
     </entry>
	 <entry name="forward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.forward_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.COMMENT"/>
      </xsl:call-template>
     </entry>
	 <entry name="backward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.backward_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.COMMENT"/>
      </xsl:call-template>
     </entry>
  </family>

  
  <!-- directional adverbs:  left, forward.... and to_the_right for MWE -->
  <family name="adv.direction" pos="ADV" closed="true">
    <entry name="initial">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.initial)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHICHWAY"/>
      </xsl:call-template>
     </entry>
	 <entry name="forward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.forward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHICHWAY"/>
      </xsl:call-template>
     </entry>
	 <entry name="backward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.backward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHICHWAY"/>
      </xsl:call-template>
     </entry>
  </family>
  <family name="adv.direction_W_W" pos="ADV" closed="true">
    <entry name="initial">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.initial_W_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHICHWAY"/>
      </xsl:call-template>
     </entry>
	 <entry name="forward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.forward_W_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHICHWAY"/>
      </xsl:call-template>
     </entry>
	 <entry name="backward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.backward_W_W)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHICHWAY"/>
      </xsl:call-template>
     </entry>
  </family>

  
  
  <family name="adv.whereto" pos="ADV" closed="true">
    <entry name="initial">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.initial)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHERETO"/>
      </xsl:call-template>
     </entry>
	 <entry name="forward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.forward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHERETO"/>
      </xsl:call-template>
     </entry>
	 <entry name="backward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.backward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.WHERETO"/>
      </xsl:call-template>
     </entry>
  </family>

   <family name="adv.location" pos="ADV" closed="true">
     <entry name="initial">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.initial)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.LOCATION"/>
      </xsl:call-template>
     </entry>
	 <entry name="forward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.forward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.LOCATION"/>
      </xsl:call-template>
     </entry>
	 <entry name="backward">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($adv.mod-vp.backward)/*"/>
        <xsl:with-param name="ext" select="$MOD-E.LOCATION"/>
      </xsl:call-template>
     </entry>
  </family>

    
  </xsl:template>
</xsl:transform>

