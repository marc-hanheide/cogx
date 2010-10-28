<?xml version="1.0"?>
<!--
Copyright (C) 2005 DFKI (Geert-Jan M. Kruijff, gj@dfki.de) 

-->
<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">

  <!-- ================================================ -->  
  <!-- Lexical macros                                   -->
  <!-- ================================================ -->  

  <xsl:template name="add-macros">

  

<macro name="@s-location">
   <fs id="25" attr="mod-type" val="s-location"/>
</macro>
<macro name="@s-whereto">
   <fs id="25" attr="mod-type" val="s-whereto"/>
</macro>
<macro name="@s-wherefrom">
   <fs id="25" attr="mod-type" val="s-wherefrom"/>
</macro>
<macro name="@s-direction">
   <fs id="25" attr="mod-type" val="s-direction"/>
</macro>
<macro name="@s-through">
   <fs id="25" attr="mod-type" val="s-through"/>
</macro>
<macro name="@s-manner">
   <fs id="25" attr="mod-type" val="s-manner"/>
</macro>
<macro name="@s-comparison">
   <fs id="25" attr="mod-type" val="s-comparison"/>
</macro>
<macro name="@s-probability">
   <fs id="25" attr="mod-type" val="s-probability"/>
</macro>
<macro name="@s-comment">
   <fs id="25" attr="mod-type" val="s-comment"/>
</macro>
<macro name="@s-time">
   <fs id="25" attr="mod-type" val="s-time"/>
</macro>



<macro name="@adv-all">
   <fs id="25" attr="complexcat-type" val="adv-all"/>
</macro>
<macro name="@adv-no-s-right">
   <fs id="25" attr="complexcat-type" val="adv-no-s-right"/>
</macro>

<macro name="@all-left">
   <fs id="25" attr="complexcat-type" val="all-left"/>
</macro>
<macro name="@sent-left">
   <fs id="25" attr="complexcat-type" val="sent-left"/>
</macro>
<macro name="@noun-left">
   <fs id="25" attr="complexcat-type" val="noun-left"/>
</macro>


  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- Macros: GENERIC N and NP (T, 1)                  -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
 
   <!--  Number, syntactic and semantic           -->
    
  <macro name="@num.sg">
    <fs id="1" attr="num" val="sg"/>
    <lf>
      <satop nomvar="T">
        <diamond mode="num"><prop name="sg"/></diamond>
      </satop>
    </lf>
  </macro>
  <macro name="@num.pl">
    <fs id="1" attr="num" val="pl"/>
    <lf>
      <satop nomvar="T">
        <diamond mode="num"><prop name="pl"/></diamond>
      </satop>
    </lf>
  </macro>

 <macro name="@num.sg-second">
    <fs id="8" attr="num" val="sg"/>
    <lf>
      <satop nomvar="T">
        <diamond mode="num"><prop name="sg"/></diamond>
      </satop>
    </lf>
  </macro>
  <macro name="@num.pl-second">
    <fs id="8" attr="num" val="pl"/>
    <lf>
      <satop nomvar="T">
        <diamond mode="num"><prop name="pl"/></diamond>
      </satop>
    </lf>
  </macro>



  <macro name="@syn-num.sg">
	 <fs id="1" attr="num" val="sg"/>
  </macro>
  <macro name="@syn-num.pl">
	<fs id="1" attr="num" val="pl"/>
  </macro>
  
  <macro name="@sem-num.sg">
     <lf>
	<satop nomvar="T">
	    <diamond mode="Number"><prop name="sg"/></diamond>
	</satop>
     </lf>
  </macro>
  <macro name="@sem-num.pl">
     <lf>
	<satop nomvar="T">
	    <diamond mode="Number"><prop name="pl"/></diamond>
	</satop>
     </lf>
  </macro>
    
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- Macros: np/n-type.1:person                       -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <macro name="@pers.1st">
	 <fs id="1" attr="pers" val="1st"/>
  </macro>

  <macro name="@pers.2nd">
	<fs id="1" attr="pers" val="2nd"/>
  </macro>

  <macro name="@pers.3rd">
	<fs id="1" attr="pers" val="3rd"/>
  </macro>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- Macros: case.1									-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <macro name="@nom">
	 <fs id="1" attr="case" val="nom"/>
  </macro>
  <macro name="@acc">
	 <fs id="1" attr="case" val="acc"/>
  </macro>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->
  <!-- Macros: det:demonstrative:spatial:proximity	--> 
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->


  <macro name="@proximity.proximal">
    <lf>
      <satop nomvar="T">
        <diamond mode="Proximity"><prop name="proximal"/></diamond>
      </satop>
    </lf>
  </macro>
  <macro name="@proximity.distal">
    <lf>
      <satop nomvar="T">
        <diamond mode="Proximity"><prop name="distal"/></diamond>
      </satop>
    </lf>
  </macro>

<!-- used for deictic events  I can do this, I can't do that -->
  <macro name="@proximity.proximal-event">
    <lf>
      <satop nomvar="E0">
        <diamond mode="Proximity"><prop name="proximal"/></diamond>
      </satop>
    </lf>
  </macro>
  <macro name="@proximity.distal-event">
    <lf>
      <satop nomvar="E0">
        <diamond mode="Proximity"><prop name="distal"/></diamond>
      </satop>
    </lf>
  </macro>

  
  <!-- Delimitation       -->


  <macro name="@del.unique">
    <lf>
      <satop nomvar="T">
        <diamond mode="Delimitation"><prop name="unique"/></diamond>
      </satop>
    </lf>	
  </macro>

  <macro name="@del.exist">
    <lf>
      <satop nomvar="T">
        <diamond mode="Delimitation"><prop name="existential"/></diamond>
      </satop>
    </lf>	
  </macro>
  
  <macro name="@del.var">
    <lf>
      <satop nomvar="T">
        <diamond mode="Delimitation"><prop name="variable2"/></diamond>
      </satop>
    </lf>	
  </macro>

  <!-- Quantification       -->

  <macro name="@quant.nonsg">
    <lf>
      <satop nomvar="T">
        <diamond mode="Quantification"><prop name="specific_non-singular"/></diamond>
      </satop>
    </lf>	
  </macro>

  <macro name="@quant.sg">
    <lf>
      <satop nomvar="T">
        <diamond mode="Quantification"><prop name="specific_singular"/></diamond>
      </satop>
    </lf>	
  </macro>

  <macro name="@quant.unspec.nonsg">
    <lf>
      <satop nomvar="T">
        <diamond mode="Quantification"><prop name="unspecific_non-singular2"/></diamond>
      </satop>
    </lf>	
  </macro>

  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- Macros: prep-type.P:degree of comparison         -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  


 <macro name="@degree.base">
	<fs id="25" attr="aform" val="aform-base"/>
 </macro>

  <macro name="@degree.comparative">
	<fs id="25" attr="aform" val="comparative"/>
    <lf>
      <satop nomvar="M">
        <diamond mode="Degree"><prop name="comparative"/></diamond>
      </satop>
    </lf>
 </macro>

  <macro name="@degree.superlative">
    <fs id="25" attr="aform" val="superlative"/>
    <lf>
      <satop nomvar="M">
        <diamond mode="Degree"><prop name="superlative"/></diamond>
      </satop>
    </lf>	
  </macro>

  

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- Macros: v-type.vmood.conditional               -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++  

  <macro name="@+3rd-agr">
    <fs id="11" attr="3rd" val="+"/>
    <fs id="2" attr="3rd" val="+"/>
  </macro>

  <macro name="@-3rd">
    <fs id="11" attr="3rd" val="-"/>
    <fs id="2" attr="3rd" val="-"/>
  </macro>

  <macro name="@-3rd-agr">
    <fs id="11" attr="3rd" val="-"/>
    <fs id="2" attr="3rd" val="-"/>
  </macro>

  
  <macro name="@sg-agr">
    <fs id="11" attr="num" val="sg"/>
    <fs id="2" attr="num" val="sg"/>
  </macro>


  <macro name="@pl-agr">
    <fs id="11" attr="num" val="pl"/>
    <fs id="2" attr="num" val="pl"/>
  </macro> --> 
  
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- Macros: Verb:num-agreement                       -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <macro name="@num.sg-agr">
	 <fs id="11" attr="num" val="sg"/>
	 <fs id="2" attr="num" val="sg"/>
  </macro>

  <macro name="@num.pl-agr">
	<fs id="11" attr="num" val="pl"/>
	<fs id="2" attr="num" val="pl"/>
  </macro>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- Macros: Verb:person-agreement                    -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  


<!-- not working... was for non-fin -->
  <macro name="@no-subj">
	 <fs id="2" attr="pers" val="pers-na"/>
 	 <fs id="2" attr="num" val="num-na"/>
	<!-- <fs id="2" attr="pers" val="pers-na"/>
	 <fs id="2" attr="num" val="num-na"/> -->
  </macro>

  <macro name="@pers.1st-agr">
	 <fs id="11" attr="pers" val="1st"/>
	 <fs id="2" attr="pers" val="1st"/>
  </macro>

  <macro name="@pers.2nd-agr">
	<fs id="11" attr="pers" val="2nd"/>
	<fs id="2" attr="pers" val="2nd"/>
  </macro>

  <macro name="@pers.3rd-agr">
	<fs id="11" attr="pers" val="3rd"/>
	<fs id="2" attr="pers" val="3rd"/>
  </macro>
  
  <macro name="@pers.non-3rd-agr">
    <fs id="11">
      <feat attr="pers"><featvar name="PERS:non-3rd"/></feat>
    </fs>
    <fs id="2">
      <feat attr="pers"><featvar name="PERS:non-3rd"/></feat>
    </fs>
  </macro>


  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- Macros: VERB-Vform                               -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <macro name="@finite">
    <fs id="11" attr="vform" val="fin"/>
  </macro>

  <!-- These two vforms have closed subj positions! -->
 
  <macro name="@nonfin">
    <fs id="11" attr="vform" val="non-fin"/>
 <!--   <fs id="2" attr="pers" val="pers-na"/>
 	<fs id="2" attr="num" val="num-na"/> -->
  </macro>
  
  <macro name="@vform.progr">
	<fs id="11" attr="vform" val="progr"/>
 <!--   <fs id="2" attr="pers" val="pers-na"/>
 	<fs id="2" attr="num" val="num-na"/> -->
  </macro>
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- Macros: VERB-Mood                                -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <macro name="@imp-addressee">
    <fs id="11" attr="mood" val="imp"/>
  	<lf>
       <satop nomvar="X:animate"><prop name="addressee"/></satop>         
	   <satop nomvar="E"><diamond mode="Mood"><prop name="imp"/></diamond></satop>
	</lf>
  </macro>
 
  <macro name="@imp">
    <fs id="11" attr="mood" val="imp"/>
  	<lf>
 	   <satop nomvar="E"><diamond mode="Mood"><prop name="imp"/></diamond></satop>
	</lf>
  </macro>
  
  <macro name="@int">
	<fs id="11" attr="mood" val="int"/>
    <lf>
	<satop nomvar="E">
        <diamond mode="Mood"><prop name="int"/></diamond>
    </satop>
	</lf>
  </macro>
  
  <macro name="@ind">
	<fs id="11" attr="mood" val="ind"/>
	<lf>
	<satop nomvar="E">
        <diamond mode="Mood"><prop name="ind"/></diamond>
    </satop>
    </lf>
  </macro>


  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- Macros:  VERB-Tense								-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ 

  <macro name="@vform.non-pres">  
  	<fs id="11" attr="vform" val="non-pres"/>
  </macro>
  
  <macro name="@vform.non-past">
	<fs id="11" attr="vform" val="non-past"/>
  </macro>
  
  <macro name="@vform.pres">
	<fs id="11" attr="vform" val="pres"/>
  </macro>

  <macro name="@vform.progr">
	<fs id="11" attr="vform" val="progr"/>  
  </macro> --> 
 
  
  
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- Macros:  VERB- Semantics Tense					-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ --> 

  
  <macro name="@progr">
    <lf>
      <satop nomvar="E">
        <diamond mode="Aspect"><prop name="progr"/></diamond>
      </satop>
    </lf>
  </macro>

  <macro name="@past">
    <lf>
      <satop nomvar="E">
        <diamond mode="Tense"><prop name="past"/></diamond>
      </satop>
    </lf>
  </macro>

  <macro name="@pres">
    <lf>
      <satop nomvar="E">
        <diamond mode="Tense"><prop name="pres"/></diamond>
      </satop>
    </lf>
  </macro>

  <macro name="@future">
    <lf>
      <satop nomvar="E">
        <diamond mode="Tense"><prop name="future"/></diamond>
      </satop>
    </lf>
  </macro>  
  
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- Macros:  VERB- Polarity       					-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ --> 

  
  <macro name="@negative">
    <lf>
      <satop nomvar="E">
        <diamond mode="Polarity"><prop name="-"/></diamond>
      </satop>
    </lf>
  </macro>

  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- Macros: N-form (used for existentials, qw, etc)  -->
  <!--    first set mark nouns, next mark verbs         -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  
  <macro name="@dummy-deic">
    <fs id="1">
      <feat attr="nform" val="dummy-deic"/>
    </fs>
  </macro>

  <macro name="@dummy-pers">
    <fs id="1">
      <feat attr="nform" val="dummy-pers"/>
    </fs>
  </macro>
  
  <macro name="@subj-dummy-deic">
    <fs id="2">
      <feat attr="nform" val="dummy-deic"/>
    </fs>
  </macro>
  
  <macro name="@subj-dummy-pers">
    <fs id="2">
      <feat attr="nform" val="dummy-pers"/>
    </fs>
  </macro>
      
  <macro name="@nform-who-obj">
    <fs id="1">
      <feat attr="nform" val="who-obj"/>
    </fs>
  </macro>
    
  <macro name="@nform-who-subj">
    <fs id="2">
      <feat attr="nform" val="who-subj"/>
    </fs>
  </macro>

  <!-- question word n-forms -->
  
  <macro name="@nform-what-obj">
    <fs id="7">
      <feat attr="nform" val="what-obj"/>
    </fs>
  </macro>
 
  <macro name="@nform-where-location">
    <fs id="7">
      <feat attr="nform" val="where-location"/>
    </fs>
  </macro>
  
  
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- Macros: P-form (needed for casemarking)          -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <!-- these mark the preps themselves -->

  <macro name="@pform-of">
    <fs id="30">
      <feat attr="pform" val="of"/>
    </fs>
  </macro>

  <macro name="@pform-for">
    <fs id="30">
      <feat attr="pform" val="for"/>
    </fs>
  </macro>
  

  <macro name="@pform-to">
    <fs id="30">
      <feat attr="pform" val="to"/>
    </fs>
  </macro>
  
  <macro name="@pform-at">
    <fs id="30">
      <feat attr="pform" val="at"/>
    </fs>
  </macro>

  <macro name="@pform-with">
    <fs id="30">
      <feat attr="pform" val="with"/>
    </fs>
  </macro>

 <macro name="@pform-about">
    <fs id="30">
      <feat attr="pform" val="about"/>
    </fs>
  </macro>


<!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- Macros: W-form (needed for MWE)                  -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ 

GENERATED AUTOMATICALLY and PLACED IN DICT NEXT TO LEX ENTRY -->  


  </xsl:template>

</xsl:transform>