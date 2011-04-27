<?xml version="1.0"?>
<!--
Copyright (C) 2005 DFKI GmbH (Geert-Jan M. Kruijff gj@dfki.de)

This file contains the cats, semantics and families of those words (and constructions)
which don't have their own files (yet?)

-->

<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">



<!-- ================================================ -->  
<!-- NEGATION OF AUXILIARY/MODAL and COPULAR          -->
<!-- ================================================ -->  


<!-- NOTE: COPULAR handled via normal adv with semantic sort restriction (m-negation)
		   Aux cannot because it has no-subj restriction for verb.. -->

<!-- Spent an hour or so Oct 3 trying to sort out negation. Dont want I am happy not to parse, but can't seem to block it.
	Also, note that when(if) aux changed to subject raising verbs, then will need to adopt the standard copular which is now
	just an adverb, really. NOTE:  tried 3 versions of copular, 1 for each np, adj, pp but didnt seem to work -->

<xsl:variable name="neg.aux-verb">
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb.aux)/*"/>
      <xsl:with-param name="ext">
	     <slash dir="\" mode="*"/>
         <xsl:copy-of select="$verb.aux"/> 
	  </xsl:with-param>
    </xsl:call-template>
 </xsl:variable>

<xsl:variable name="neg.aux-verb.progr">
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb.aux.progr)/*"/>
      <xsl:with-param name="ext">
	     <slash dir="\" mode="*"/>
         <xsl:copy-of select="$verb.aux.progr"/> 
	  </xsl:with-param>
    </xsl:call-template>
 </xsl:variable>

 <xsl:variable name="neg.copular">
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($vp.generic)/*"/>
      <xsl:with-param name="ext">
	     <slash dir="\" mode="&lt;"/>
         <xsl:copy-of select="$vp.generic"/> 
	  </xsl:with-param>
    </xsl:call-template>
 </xsl:variable>
 
<!-- I think the prop is necessary to allow generation -->
 <xsl:variable name="MOD-AUX.NEGATION">
    <lf>
	  <satop nomvar="E:m-negation">
	      <diamond mode="Polarity"><prop name="-"/></diamond>
      </satop>
	</lf>
  </xsl:variable>

<!-- ================================================ -->  
<!--  PARTICLE CATS -->	
<!-- ================================================ -->  

<!-- When you include the part-form here, when it has already been included in the family definition, it wont parse pick up -->

  <xsl:variable name="prt.generic">
    <atomcat type="prt">
	  <fs id="26">
        <feat attr="prt-form" val="[*DEFAULT*]" />
        <feat attr="index"><lf><nomvar name="PR"/></lf></feat>
      </fs>
	</atomcat>
  </xsl:variable>

  <xsl:variable name="PRT.DEFAULT">
    <lf>
	  <satop nomvar="PR">
        <prop name="[*DEFAULT*]"/>
      </satop>
	</lf>
  </xsl:variable>


<!-- ================================================ -->  
<!--  PRO-SENTENCES  (ex. She said that )  -->	
<!-- ================================================ -->  


  <xsl:variable name="s.generic.deictic">
 	  <atomcat type="s">
	    <fs id="10">
	      <feat attr="index"><lf><nomvar name="E0"/></lf></feat>
	      <feat attr="vform" val="vform-deictic"/>	  
		</fs>
	  </atomcat>
  </xsl:variable> 	


  <xsl:variable name="E.DEICTIC">
    <lf>
	  <satop nomvar="E0:event">
        <prop name="[*DEFAULT*]"/>
      </satop>
	</lf>
  </xsl:variable>


<!-- ================================================ -->  
<!--  RELATIVE PRONOUNS (for reduced, see unary-rules.xsl)  -->	
<!-- ================================================ -->  

 <xsl:variable name="relative">
    <complexcat>
      <xsl:copy-of select="$n.from-generic"/>
	  <slash dir="\" mode="*"/>
      <xsl:copy-of select="$n.from-generic"/>
      <slash dir="/" mode="*"/>
      <complexcat>
          <xsl:copy-of select="$s.mod-arg"/>
          <slash dir="|" mode="^"/>
          <xsl:copy-of select="$np.generic"/>
      </complexcat>
    </complexcat>
  </xsl:variable>
  
  <xsl:variable name="relative.np">
    <complexcat>
      <xsl:copy-of select="$np.from-generic"/>
	  <slash dir="\" mode="*"/>
      <xsl:copy-of select="$np.generic.modifiable"/>
      <slash dir="/" mode="*"/>
      <complexcat>
          <xsl:copy-of select="$s.mod-arg"/>
          <slash dir="|" mode="^"/>
          <xsl:copy-of select="$np.from-generic"/>
      </complexcat>
    </complexcat>
  </xsl:variable>
  
<!-- ================================================ -->  
<!-- NOUN MODIFIER: PURPOSE ASCRIBING "TO" & "FOR"	-->
<!-- ================================================ -->  

<!-- "there are balls to play with"  "I have lots of food to cook" 
		"there are some carrots for cooking" -->

 <xsl:variable name="n-to-n.non-fin-verb">
    <complexcat>
      <xsl:copy-of select="$n.from-generic"/>
	  <slash dir="\" mode="&lt;"/>
      <xsl:copy-of select="$n.from-generic"/>
      <slash dir="/" mode="*"/>
      <complexcat>
 		  <xsl:copy-of select="$s.mod-arg.non-fin"/>
          <slash dir="\" mode="&lt;"/>
          <xsl:copy-of select="$np.subj"/>
          <slash dir="/" mode="^"/>
          <xsl:copy-of select="$np.generic"/>
      </complexcat>
    </complexcat>
  </xsl:variable>
  
  <xsl:variable name="np-to-np.non-fin-verb">
    <complexcat>
      <xsl:copy-of select="$np.from-generic"/>
	  <slash dir="\" mode="*"/>
      <xsl:copy-of select="$np.generic.modifiable"/>
      <slash dir="/" mode="*"/>
      <complexcat>
          <xsl:copy-of select="$s.mod-arg.non-fin"/>
          <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$np.subj"/>
          <slash dir="/" mode="^"/>
          <xsl:copy-of select="$np.from-generic"/>
      </complexcat>
    </complexcat>
  </xsl:variable>
 <xsl:variable name="n-to-n.progr-verb">
    <complexcat>
      <xsl:copy-of select="$n.from-generic"/>
	  <slash dir="\" mode="^"/>
      <xsl:copy-of select="$n.from-generic"/>
      <slash dir="/" mode="*"/>
      <complexcat>
 		  <xsl:copy-of select="$s.mod-arg.progr"/>
          <slash dir="\" mode="&lt;"/>
          <xsl:copy-of select="$np.subj"/>
          <slash dir="/" mode="^"/>
          <xsl:copy-of select="$np.generic"/>
      </complexcat>
    </complexcat>
  </xsl:variable>
  
  <xsl:variable name="np-to-np.progr-verb">
    <complexcat>
      <xsl:copy-of select="$np.from-generic"/>
	  <slash dir="\" mode="*"/>
      <xsl:copy-of select="$np.generic.modifiable"/>
      <slash dir="/" mode="*"/>
      <complexcat>
          <xsl:copy-of select="$s.mod-arg.progr"/>
          <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$np.subj"/>
          <slash dir="/" mode="^"/>
          <xsl:copy-of select="$np.from-generic"/>
      </complexcat>
    </complexcat>
  </xsl:variable>



  <!-- ================================================ -->  
  <!-- POSSESSIVE 's (modifies object much like adj)    -->
  <!-- ================================================ -->  

 <xsl:variable name="possessive">
    <complexcat>
      <xsl:copy-of select="$np.generic"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$n.generic"/>
      <slash dir="\" mode="&lt;"/>
      <xsl:copy-of select="$np.mod-arg"/>
   </complexcat>
 </xsl:variable>  

 <xsl:variable name="possessive2">
    <complexcat>
      <xsl:copy-of select="$n.generic"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$np.mod-arg"/>
      <slash dir="\" mode="&lt;"/>
      <xsl:copy-of select="$n.generic"/>
   </complexcat>
 </xsl:variable>  

 
<xsl:variable name="possessed">
    <complexcat>
      <xsl:copy-of select="$np.generic"/>
      <slash dir="\" mode="&lt;"/>
      <xsl:copy-of select="$np.mod-arg"/>
   </complexcat>
 </xsl:variable>  



  <!-- ================================================ -->  
  <!-- Comparative "than" as in, "bigger than you"      -->
  <!-- ================================================ -->  
 
  <xsl:variable name="mod-comparative-adj.np-arg">
    <complexcat>
       <xsl:copy-of select="$adj.comparative"/>
       <slash dir="/" mode="&gt;"/>
       <xsl:copy-of select="$np.mod-arg"/>
       <slash dir="\" mode="&lt;"/>
	   <xsl:copy-of select="$adj.comparative"/>
    </complexcat>
  </xsl:variable>
  
  <xsl:variable name="MOD-M.COMPARATIVE">
    <lf>
      <satop nomvar="M">
        <xsl:copy-of select="$COMPARISON-ENTITY"/> 
      </satop>
    </lf>
   </xsl:variable>


  <xsl:variable name="number">
    <atomcat type="num">
      <fs id="1">
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>



  <!-- ================================================ -->  
  <!-- FAMILY         DEFINITIONS                       -->
  <!-- ================================================ -->  


  <xsl:template name="add-other-families">

<!-- Cop negation handled like normal forward adv, but has semantically restricted event argument 
     Aux negation handled differently because of NO-SUBJ strategy. See other.xsl -->


	<family name="negation.sentential" pos="MOD" indexRel="Polarity" closed="true">
	  <entry name="vp">
         <xsl:call-template name="extend">
		 <!-- found in mod-cats -->
         <xsl:with-param name="elt" select="xalan:nodeset($mod-loose-vp.left.no-dollar)/*"/>
         <xsl:with-param name="ext" select="$MOD-E.NEGATION"/>		
         </xsl:call-template>
	  </entry>
	 
	 </family>



	<family name="copular-negation" pos="ADV" indexRel="Polarity" closed="true">
	 
      <entry name="pp">
         <xsl:call-template name="extend">
         <xsl:with-param name="elt" select="xalan:nodeset($neg.copular)/*"/>
         <xsl:with-param name="ext" select="$MOD-E.NEGATION"/>		
         </xsl:call-template>
	  </entry>
	 
	 </family>

     <family name="aux-negation" pos="ADV" indexRel="Polarity" closed="true">
	   <entry name="aux">
         <xsl:call-template name="extend">
         <xsl:with-param name="elt" select="xalan:nodeset($neg.aux-verb)/*"/>
         <xsl:with-param name="ext" select="$MOD-AUX.NEGATION"/>
         </xsl:call-template>
       </entry>
     <!--  <entry name="aux.progr">
         <xsl:call-template name="extend">
         <xsl:with-param name="elt" select="xalan:nodeset($neg.aux-verb.progr)/*"/>
         <xsl:with-param name="ext" select="$MOD-AUX.NEGATION"/>
         </xsl:call-template>
       </entry> -->
	  </family>

  <family name="particle" pos="PRT" closed="true">
    <entry name="Primary">
	  <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($prt.generic)/*"/>
        <xsl:with-param name="ext" select="$PRT.DEFAULT"/>
      </xsl:call-template>	
	</entry>
 </family>


  <family name="pro-sentence" pos="S" closed="true">
    <entry name="Primary">
	  <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($s.generic.deictic)/*"/>
        <xsl:with-param name="ext" select="$E.DEICTIC"/>
      </xsl:call-template>	
    </entry>
  </family>
  
  
 <family name="rel-pronoun" pos="MARKER" closed="true" indexRel="Restr" >
    <entry name="n">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($relative)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.RESTR"/>
      </xsl:call-template>	  
     </entry>
	 <entry name="np">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($relative.np)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.RESTR"/>
      </xsl:call-template>
	 </entry> 
  </family>

 <family name="n-mod.purpose.non-fin-verb" pos="MARKER" closed="true" indexRel="Purpose" >
    <entry name="n">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($n-to-n.non-fin-verb)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.PURPOSE"/>
      </xsl:call-template>	  
     </entry>
	 <entry name="np">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np-to-np.non-fin-verb)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.PURPOSE"/>
      </xsl:call-template>
	 </entry> 
  </family>
  
   <family name="n-mod.purpose.progr-verb" pos="MARKER" closed="true" indexRel="Purpose" >
    <entry name="n">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($n-to-n.progr-verb)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.PURPOSE"/>
      </xsl:call-template>	  
     </entry>
	 <entry name="np">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np-to-np.progr-verb)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.PURPOSE"/>
      </xsl:call-template>
	 </entry> 
  </family>


  
 <family name="possessive-'s" pos="MARKER" indexRel="GenOwner" closed="true" >
    <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($possessive)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.OWNER"/>
      </xsl:call-template>
     </entry>
  </family>

 <family name="possessive-of" pos="MARKER" indexRel="GenOwner" closed="true" >
    <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($possessive2)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.OWNER"/>
      </xsl:call-template>
     </entry>
  </family>


 <family name="possessed-'s" pos="MARKER" indexRel="GenOwner" closed="true" >
    <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($possessed)/*"/>
        <xsl:with-param name="ext" select="$MOD-T.OWNER"/>
      </xsl:call-template>
     </entry>
  </family>
  
 <family name="comparative-adj-than" pos="MARKER" closed="true" >
    <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($mod-comparative-adj.np-arg)/*"/>
        <xsl:with-param name="ext" select="$MOD-M.COMPARATIVE"/>
      </xsl:call-template>
     </entry>
  </family>

 <family name="number" pos="NUM" closed="true" >
    <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($number)/*"/>
        <xsl:with-param name="ext" select="$ENTITY"/>
      </xsl:call-template>
     </entry>
  </family>


  </xsl:template>
</xsl:transform>


<!--
  <family na me="vp-negation" pos="ADV" closed="true">
     <entry name="aux">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($neg.aux-verb)/*"/>
        <xsl:with-param name="ext" select="$MOD-AUX.NEGATION"/>
      </xsl:call-template>
     </entry>
     <entry na me="aux-progr">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($neg.aux-verb.progr)/*"/>
        <xsl:with-param name="ext" select="$MOD-AUX.NEGATION"/>
      </xsl:call-template>
     </entry>
     <entry nam e="cop-np">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($neg.copular-np)/*"/>
        <xsl:with-param name="ext" select="$MOD-AUX.NEGATION"/>
      </xsl:call-template>
     </entry>
     <entry na me="cop-pp">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($neg.copular-pp)/*"/>
        <xsl:with-param name="ext" select="$MOD-AUX.NEGATION"/>
      </xsl:call-template>
     </entry>
     <entry nam e="cop-adj">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($neg.copular-adj)/*"/>
        <xsl:with-param name="ext" select="$MOD-AUX.NEGATION"/>
      </xsl:call-template>
     </entry>
  </family>
-->


 <!-- NOW HANDLED VIA NORMAL ADV with SEMANTIC RESTRICTION
 <xsl:variable n ame="ne g.copular-np">
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($copular-np)/*"/>
      <xsl:with-param name="ext">
	     <slash dir="\" mode="^"/>
         <xsl:copy-of select="$copular-np"/> 
	  </xsl:with-param>
    </xsl:call-template>
 </xsl:variable>
 
 <xsl:variable n ame="neg.copular-adj">
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($copular-adj)/*"/>
      <xsl:with-param name="ext">
	     <slash dir="\" mode="^"/>
         <xsl:copy-of select="$copular-adj"/> 
	  </xsl:with-param>
    </xsl:call-template>
 </xsl:variable>

 <xsl:variable n ame="neg.copular-pp">
  <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($copular-pp)/*"/>
      <xsl:with-param name="ext">
	     <slash dir="\" mode="^"/>
         <xsl:copy-of select="$copular-pp"/> 
	  </xsl:with-param>
    </xsl:call-template>
 </xsl:variable>
-->

