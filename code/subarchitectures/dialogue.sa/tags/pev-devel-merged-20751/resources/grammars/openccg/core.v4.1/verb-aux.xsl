<?xml version="1.0"?>
<!--
Copyright (C) 2005 DFKI GmbH (Geert-Jan M. Kruijff gj@dfki.de)
	RESTR-E is a hack. Should make a basic category for s which uses T as variable,
		then can have uniform... better, created local "missing-np" "missing-s" which use same variable!!

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



<!-- Think of a better name and put in Verb-roles -->

 <xsl:variable name="fronted-obj-whnp">
    <atomcat type="np">
      <fs id="7">
	    <feat attr="nform"><featvar name="WHNP:WHNP-obj"/></feat> 
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

<xsl:variable name="fronted-scomp-whnp">
    <atomcat type="np">
      <fs id="7">
	    <feat attr="nform"><featvar name="WHNP:WHNP-scomp"/></feat> 
        <feat attr="index"><lf><nomvar name="E0"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>
  
 <xsl:variable name="fronted-pp-mod-whnp">
    <atomcat type="np">
      <fs id="7">
        <feat attr="nform"><featvar name="WHNP:WHNP-pp-mod"/></feat>
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>



<!-- standard auxiliary:  " I do like bananas" -->

   <xsl:variable name="verb.aux">
       <xsl:copy-of select="$verb.non-fin-verb"/> 
   </xsl:variable>


<!-- THESE ARE THE CATS PRE-INTRODUCTION OF AGENT (to make incremental)
	
	TO FIX SEMANTICS, just take out AGENT


 <xsl:variable name="verb.aux">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($vp.main.fin)/*"/>
      <xsl:with-param name="ext">
	     <slash dir="/" mode="^"/>
         <xsl:copy-of select="$vp.vcomp.subj-controlled.non-fin"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  
    <xsl:variable name="inverted">
    <complexcat> 
	  <xsl:copy-of select="$s.main.int"/>
	  <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$s.vcomp.non-fin"/>
	</complexcat>  
  </xsl:variable>

  
  -->

 <xsl:variable name="verb.aux.progr">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($vp.main.fin)/*"/>
      <xsl:with-param name="ext">
	     <slash dir="/" mode="^"/>
         <xsl:copy-of select="$vp.vcomp.subj-controlled.progr"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

<!-- Inverted Form "Do you like bananas" -->

 <xsl:variable name="inverted">
    <complexcat> 
	  <xsl:copy-of select="$s.main.int"/>
	  <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$vp.vcomp.subj-controlled.non-fin"/>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$np.subj"/>
	</complexcat>  
 </xsl:variable>

 
 <xsl:variable name="inverted.progr">
     <complexcat> 
	  <xsl:copy-of select="$s.main.int"/>
	  <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$s.vcomp.progr"/>
    </complexcat>  
  </xsl:variable>



<!-- Inverted Form "Where do you want to go" 
  PROBLEM!  Only goes one layer down, not to last....
  Ex. Where do you want him to run,  where attaches to want, not run...
    (unlike objects, which can run arbitrarily far down....)
-->


  <xsl:variable name="fronted-whnp">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($inverted)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="\" mode="&lt;"/>
 	    <xsl:copy-of select="$fronted-pp-mod-whnp"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

  <xsl:variable name="fronted-whnp.progr">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($inverted.progr)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="\" mode="&lt;"/>
 	    <xsl:copy-of select="$fronted-pp-mod-whnp"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
 
   
 <xsl:variable name="fronted-whnp.obj-controlled">
    <complexcat> 
	  <xsl:copy-of select="$s.main.int"/> 
      <slash dir="/" mode="&gt;"/>
      <complexcat>
         <xsl:copy-of select="$s.vcomp.non-fin"/>
	     <slash dir="\" mode="&lt;"/>
		 <xsl:copy-of select="$np.subj"/>
         <slash dir="/" mode="&gt;"/>
		 <xsl:copy-of select="$np.generic"/>
	  </complexcat>
      <slash dir="/" mode="&gt;"/>
      <xsl:copy-of select="$np.subj"/> 
	  <slash dir="\" mode="&lt;"/>
	  <xsl:copy-of select="$fronted-obj-whnp"/>    	 	  
   </complexcat>  
  </xsl:variable>
  

<xsl:variable name="fronted-whnp.scomp">
    <complexcat> 
	  <xsl:copy-of select="$s.main.int"/> 
      <slash dir="/" mode="&gt;"/>
      <complexcat>
         <xsl:copy-of select="$s.vcomp.non-fin"/>
		 <slash dir="\" mode="&lt;"/>
		 <xsl:copy-of select="$np.subj"/> 
         <slash dir="/" mode="&gt;"/>
		 <xsl:copy-of select="$s.generic"/> 
	 </complexcat>
     <slash dir="/" mode="&lt;"/>
     <xsl:copy-of select="$np.subj"/> 
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$fronted-scomp-whnp"/>    	 	  
   </complexcat>  
  </xsl:variable>


  <xsl:variable name="fronted-whnp.obj-controlled.progr">
    <complexcat> 
	  <xsl:copy-of select="$s.main.int"/> 
      <slash dir="/" mode="^"/>
      <complexcat>
         <xsl:copy-of select="$s.vcomp.progr"/>
  	     <slash dir="/" mode="^"/>
		 <xsl:copy-of select="$np.generic"/> 
	 </complexcat>
	 <slash dir="\" mode="^"/>
	 <xsl:copy-of select="$fronted-obj-whnp"/>    	 	  
   </complexcat>  
  </xsl:variable>


  <xsl:variable name="fronted-whnp.scomp.progr">
    <complexcat> 
	  <xsl:copy-of select="$s.main.int"/> 
      <slash dir="/" mode="^"/>
      <complexcat>
         <xsl:copy-of select="$s.vcomp.progr"/>
	     <slash dir="/" mode="&gt;"/>
		 <xsl:copy-of select="$s.generic"/> 
	 </complexcat>
	 <slash dir="\" mode="^"/>
	 <xsl:copy-of select="$fronted-scomp-whnp"/>    	 	  
   </complexcat>  
  </xsl:variable>

  
<!-- Inverted Form "Who do you like" 
	NOTE: May 2nd, changed from np.obj, whnp index:Y
	      to general (T) idea being any! missing comp 
		  gets questioned. (hence, dt, etc)


  <xsl:variable name="fronted-whnp.obj-controlled">
    <complexcat> 
	  <xsl:copy-of select="$s.main.int"/> 
      <slash dir="/" mode="&gt;"/>
      <complexcat>
         <xsl:copy-of select="$s.vcomp.non-fin"/>
     	 <slash dir="\" mode="&lt;"/>
         <xsl:copy-of select="$np.subj"/>
	     <slash dir="/" mode="&gt;"/>
		 <xsl:copy-of select="$np.generic"/> 
	  </complexcat>
	  <slash dir="\" mode="&lt;"/>
	 <xsl:copy-of select="$fronted-obj-whnp"/>    	 	  
   </complexcat>  
  </xsl:variable>

  <xsl:variable name="fronted-whnp.obj-controlled.progr">
    <complexcat> 
	  <xsl:copy-of select="$s.main.int"/> 
      <slash dir="/" mode="&gt;"/>
      <complexcat>
         <xsl:copy-of select="$s.vcomp.progr"/>
    	 <slash dir="\" mode="&lt;"/>
         <xsl:copy-of select="$np.subj"/>
 	     <slash dir="/" mode="&gt;"/>
		 <xsl:copy-of select="$np.generic"/> 
	 </complexcat>
	 <slash dir="\" mode="&lt;"/>
	 <xsl:copy-of select="$fronted-obj-whnp"/>    	 	  
   </complexcat>  
  </xsl:variable>

<xsl:variable name="fronted-whnp.scomp">
    <complexcat> 
	  <xsl:copy-of select="$s.main.int"/> 
      <slash dir="/" mode="&gt;"/>
      <complexcat>
         <xsl:copy-of select="$s.vcomp.non-fin"/>
     	 <slash dir="\" mode="&lt;"/>
         <xsl:copy-of select="$np.subj"/>
         <slash dir="/" mode="&gt;"/>
		 <xsl:copy-of select="$s.generic"/> 
	 </complexcat>
     <slash dir="/" mode="&gt;"/>
     <xsl:copy-of select="$np.subj"/>
	 <slash dir="\" mode="&lt;"/>
	 <xsl:copy-of select="$fronted-scomp-whnp"/>    	 	  
   </complexcat>  
  </xsl:variable>

  <xsl:variable name="fronted-whnp.scomp.progr">
    <complexcat> 
	  <xsl:copy-of select="$s.main.int"/> 
      <slash dir="/" mode="&gt;"/>
      <complexcat>
         <xsl:copy-of select="$s.vcomp.progr"/>
     	 <slash dir="\" mode="&lt;"/>
         <xsl:copy-of select="$np.subj"/>
	     <slash dir="/" mode="&gt;"/>
		 <xsl:copy-of select="$s.generic"/> 
	 </complexcat>
     <slash dir="/" mode="&gt;"/>
     <xsl:copy-of select="$np.subj"/>
	 <slash dir="\" mode="&lt;"/>
	 <xsl:copy-of select="$fronted-scomp-whnp"/>    	 	  
   </complexcat>  
  </xsl:variable>
-->


  <!-- ================================================ -->  
  <!-- LEXICAL-MEANING DEFINITIONS                      -->
  <!-- ================================================ -->  
 
 
 <!-- these need to be unified with copular and handled in same way as
      all other semantic roles -->
	  
 <xsl:variable name="SSCOPE">
    <diamond mode="Scope"><nomvar name="EC"/></diamond> 
</xsl:variable>
 
 <xsl:variable name="AUX.SCOPE.INDICATIVE">
  <lf>
    <satop nomvar="E">
	    <prop name="[*DEFAULT*]"/>
		<diamond mode="Mood"><prop name="ind"/></diamond>
		<xsl:copy-of select="$ACTOR"/>
	    <xsl:copy-of select="$SSCOPE"/>
    </satop>
  </lf> 
 </xsl:variable>  

<xsl:variable name="AUX.SCOPE.INTERROGATIVE">
  <lf>
    <satop nomvar="E">
	    <prop name="[*DEFAULT*]"/>
	    <diamond mode="Mood"><prop name="int"/></diamond>
		<xsl:copy-of select="$ACTOR"/>
		<xsl:copy-of select="$SSCOPE"/>
    </satop>
  </lf> 
 </xsl:variable>  
 
 <xsl:variable name="AUX.RESTR.SCOPE.INTERROGATIVE">
  <lf>
    <satop nomvar="E">
	    <prop name="[*DEFAULT*]"/>
	    <diamond mode="Mood"><prop name="int"/></diamond>
		<diamond mode="Restr"><nomvar name="T"/></diamond>
		<xsl:copy-of select="$ACTOR"/>
		<xsl:copy-of select="$SSCOPE"/>
    </satop>
  </lf> 
 </xsl:variable>  
 <xsl:variable name="AUX.RESTR-E.SCOPE.INTERROGATIVE">
  <lf>
    <satop nomvar="E">
	    <prop name="[*DEFAULT*]"/>
	    <diamond mode="Mood"><prop name="int"/></diamond>
		<diamond mode="Restr"><nomvar name="E0"/></diamond>
		<xsl:copy-of select="$ACTOR"/>
		<xsl:copy-of select="$SSCOPE"/>
    </satop>
  </lf> 
 </xsl:variable>  

<!-- these two are cheats, they only handle single level sentence embedding,
     i.e. Where does he run?, but not Where does he want to run
	 -->
 
 <xsl:variable name="AUX.SCOPE-LOCATION.INTERROGATIVE">
  <lf>
    <satop nomvar="E">
	    <prop name="[*DEFAULT*]"/>
	    <diamond mode="Restr"><nomvar name="T"/></diamond>
		<diamond mode="Mood"><prop name="int"/></diamond>
		<xsl:copy-of select="$ACTOR"/>
		<diamond mode="Scope">
		   <nomvar name="EC:m-location"/>
		   <diamond mode="Location"><nomvar name="T"/></diamond>
		</diamond>   
    </satop>
  </lf> 
 </xsl:variable>  

<xsl:variable name="AUX.SCOPE-WHERETO.INTERROGATIVE">
  <lf>
    <satop nomvar="E">
	    <prop name="[*DEFAULT*]"/>
		<diamond mode="Restr"><nomvar name="T"/></diamond>
	    <diamond mode="Mood"><prop name="int"/></diamond>
		<xsl:copy-of select="$ACTOR"/>
		<diamond mode="Scope">
		   <nomvar name="EC:m-dynamic"/>
		   <diamond mode="Dir:WhereTo"><nomvar name="T"/></diamond>
		</diamond>   
    </satop>
  </lf> 
 </xsl:variable>  



  <!-- ==================================================================================== -->
  <!-- MODAL VERBS                                                                          -->
  <!-- ==================================================================================== -->

 <xsl:template name="add-aux-families">


 <family name="aux"  pos="V" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.aux)/*"/>
		<xsl:with-param name="ext" select="$AUX.SCOPE.INDICATIVE"/>
      </xsl:call-template>
     </entry>
  </family>

 <family name="aux.polar-question" indexRel="Polar-Question" pos="V" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($inverted)/*"/>
		<xsl:with-param name="ext" select="$AUX.SCOPE.INTERROGATIVE"/>
      </xsl:call-template>
     </entry>
  </family>
 
 <family name="aux.fronted-obj-question" indexRel="Obj-Question" pos="V" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($fronted-whnp.obj-controlled)/*"/>
		<xsl:with-param name="ext" select="$AUX.RESTR.SCOPE.INTERROGATIVE"/>
      </xsl:call-template>
     </entry>
  </family>
 
  <family name="aux.fronted-scomp-question" indexRel="Obj-Question" pos="V" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($fronted-whnp.scomp)/*"/>
		<xsl:with-param name="ext" select="$AUX.RESTR-E.SCOPE.INTERROGATIVE"/>
      </xsl:call-template>
     </entry>
  </family>
 
 <family name="aux.fronted-location-question" indexRel="Location-Question" pos="V" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($fronted-whnp)/*"/>
		<xsl:with-param name="ext" select="$AUX.SCOPE-LOCATION.INTERROGATIVE"/>
      </xsl:call-template>
     </entry>
  </family>
 
 <family name="aux.fronted-whereto-question" indexRel="Whereto-Question" pos="V" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($fronted-whnp)/*"/>
		<xsl:with-param name="ext" select="$AUX.SCOPE-WHERETO.INTERROGATIVE"/>
      </xsl:call-template>
     </entry>
  </family>
 
 
<!-- Progressive Aux (ie. be) -->

 <family name="aux.progr" pos="V" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.aux.progr)/*"/>
		<xsl:with-param name="ext" select="$AUX.SCOPE.INDICATIVE"/>
      </xsl:call-template>
     </entry>
  </family>

<family name="aux.progr.polar-question" indexRel="Polar-Question" pos="V" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($inverted.progr)/*"/>
		<xsl:with-param name="ext" select="$AUX.SCOPE.INTERROGATIVE"/>
      </xsl:call-template>
     </entry>
  </family>

<family name="aux.progr.fronted-obj-question" indexRel="Obj-Question" pos="V" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($fronted-whnp.obj-controlled.progr)/*"/>
		<xsl:with-param name="ext" select="$AUX.RESTR.SCOPE.INTERROGATIVE"/>
      </xsl:call-template>
     </entry>
  </family>
<family name="aux.progr.fronted-scomp-question" indexRel="Obj-Question" pos="V" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($fronted-whnp.scomp.progr)/*"/>
		<xsl:with-param name="ext" select="$AUX.RESTR-E.SCOPE.INTERROGATIVE"/>
      </xsl:call-template>
     </entry>
  </family>
  
 <family name="aux.progr.fronted-location-question" indexRel="Location-Question" pos="V" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($fronted-whnp.progr)/*"/>
		<xsl:with-param name="ext" select="$AUX.SCOPE-LOCATION.INTERROGATIVE"/>
      </xsl:call-template>
     </entry>
  </family>
 
 <family name="aux.progr.fronted-whereto-question" indexRel="Whereto-Question" pos="V" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($fronted-whnp.progr)/*"/>
		<xsl:with-param name="ext" select="$AUX.SCOPE-WHERETO.INTERROGATIVE"/>
      </xsl:call-template>
     </entry>
  </family>


</xsl:template>

</xsl:transform>


<!-- IGNORE ME NOW 
 <xsl:variable name="FRONTED-WHAT">
    <complexcat> 
	<xsl:copy-of select="$s.main.int"/>
    <slash dir="/" mode="&gt;"/>
    <complexcat> 
	  <xsl:copy-of select="$s.vcomp.int"/>
      <slash/>
      <dollar name="1"/>
      <slash dir="/" mode="&gt;"/>
	  <xsl:copy-of select="$np.generic"/>  
	</complexcat>
	</complexcat>  
 </xsl:variable>

 <xsl:variable name="FRONTED-WHAT-SEM">
  <lf>
    <satop nomvar="E">
     	<prop name="fronted"/>
	    <diamond mode="Mood"><prop name="int"/></diamond>
		<diamond mode="Restr"><nomvar name="T"/><prop name="[*DEFAULT*]"/></diamond>
		<xsl:copy-of select="$SSCOPE"/>
    </satop>
  </lf> 
 </xsl:variable>  
 
  <family name="WHAT"  pos="V" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($FRONTED-WHAT)/*"/>
		<xsl:with-param name="ext" select="$FRONTED-WHAT-SEM"/>
      </xsl:call-template>
     </entry>
  </family>

-->
 


<!-- OLD, pre-NO-SUBJ changes
  <xsl:variable nam e="fronted-whnp">
    <complexcat> 
	  <xsl:copy-of select="$s.main"/> 
      <slash dir="/" mode="^"/>
      <xsl:copy-of select="$s.vcomp.non-fin"/> 
	  <slash dir="\" mode="^"/>
	  <xsl:copy-of select="$fronted-pp-mod-whnp"/>    	 	  
    </complexcat>  
  </xsl:variable>

  <xsl:variable nam e="fronted-whnp.progr">
    <complexcat> 
	  <xsl:copy-of select="$s.main"/> 
      <slash dir="/" mode="^"/>
      <xsl:copy-of select="$s.vcomp.progr"/> 
	  <slash dir="\" mode="*"/>
	  <xsl:copy-of select="$fronted-pp-mod-whnp"/>    	 	  
    </complexcat>  
  </xsl:variable>
-->



  <!-- move me! I AM BEING USED BY NOUN-FAMILY Question-word
		Figure out macro vs family dilemma, and move
		
		Person with ...s is selected by aux, version witout is used in Noun-family
  -->
	

<!-- having mod questions as np works for depth one!
  <xsl:variable name="fronted-pp-mod-whnp">
    <atomcat type="np">
      <fs id="7">
		 <feat attr="nform"><featvar name="WHNP:WHNP-pp-mod"/></feat>
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

<xsl:variable name="whnp.pp-mod">
    <atomcat type="np">
      <fs id="7">
		 <feat attr="nform"><featvar name="WHNP-pp-mod"/></feat>
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>
-->




   <!-- REASON WHY FUTURE CAN'T BE MACRO'd 
            Will uses E0 for indicative, but inter uses E as outside,
			so would need two different macros for two types...

			might be worth doing!!!  ie.  @future-main @future-second
			or should we just bring will into line with all other aux???



  this is for will (should I just standardize! ??) 

  <xsl:variable name="non-fin-vp.becomes.fin-vp">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" s elect="xalan:nodes et($vp.from-second.fin)/*"/>
      <xsl:with-param name="ext">
	     <slash dir="/" mode="^"/>
         <xsl:copy-of se lect="$vp.second..non-fin"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

 <family name="aux.polar-question.future" indexRel="Polar-Question-Future" pos="V" closed="true">
     <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" sel ect="xalan:nodeset($inverted)/*"/>
		<xsl:with-param name="ext" select="$AUX.SCOPE.INTERROGATIVE.FUTURE"/>
      </xsl:call-template>
     </entry>
  </family>


  <family name="aux.future-ind" indexRel="Future-Indicative" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" se lect=" xalan:nodeset($non-fin-vp.becomes.fin-vp)/*"/>
        <xsl:with-param name="ext" select="$AUX.MAKE-FUTURE-INDICATIVE"/>
      </xsl:call-template>
    </entry>
  </family>

				

<xsl:variable name="AUX.MAKE-FUTURE-INDICATIVE">
  <lf>
    <satop nomvar="E2">
		<diamond mode="tense"><prop name="future"/></diamond>
	    <diamond mode="Mood"><prop name="ind"/></diamond>
   </satop>
  </lf> 
 </xsl:variable>  

<xsl:variable name="AUX.SCOPE.INTERROGATIVE.FUTURE">
  <lf>
    <satop nomvar="E">
	    <prop name="[*DEFAULT*]"/>
	    <diamond mode="Mood"><prop name="int"/></diamond>
		<diamond mode="tense"><prop name="future"/></diamond>
		<diamond mode="Scope"><nomvar name="E2:configuration"/></diamond>
    </satop>
  </lf> 
 </xsl:variable>  

-->

