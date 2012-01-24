<?xml version="1.0"?>


<!--

This file contains all of the s & vp categories used by the rest of the  files
It contains generic and main sentence categories

-->


<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">

<!-- ==================================================================================== -->
<!--  SENTENCE  CATS                                                                      -->
<!-- ==================================================================================== -->


<!-- GENERIC SENTENCE    
	Used in sentential modifiers (pp, adv, to-infinitive) 
	Certainly Don't need all of these!!! -->
	 
<!-- Sentence, ID 10, Variable E0  --> 

 <xsl:variable name="s.generic">
 	  <atomcat type="s">
	    <fs id="10">
	      <feat attr="index"><lf><nomvar name="E0"/></lf></feat>
		</fs>
	  </atomcat>
  </xsl:variable>
 
  <xsl:variable name="s.generic.inf">
 	  <atomcat type="s">
	    <fs id="10">
	      <feat attr="index"><lf><nomvar name="E0"/></lf></feat>
	      <feat attr="vform" val="inf"/>	  
		</fs>
	  </atomcat>
  </xsl:variable>
  
  <xsl:variable name="s.generic.fin">
 	  <atomcat type="s">
	    <fs id="10">
	      <feat attr="index"><lf><nomvar name="E0"/></lf></feat>
	      <feat attr="vform" val="fin"/>	  
		</fs>
	  </atomcat>
  </xsl:variable> 	

  <xsl:variable name="s.generic.fin.non-imp">
 	  <atomcat type="s">
	    <fs id="10">
	      <feat attr="index"><lf><nomvar name="E0"/></lf></feat>
	      <feat attr="vform" val="fin"/>
		  <feat attr="mood" val="non-imp"/>
		</fs>
	  </atomcat>
  </xsl:variable> 	

  <xsl:variable name="s.generic.fin.ind">
 	  <atomcat type="s">
	    <fs id="10">
	      <feat attr="index"><lf><nomvar name="E0"/></lf></feat>
	      <feat attr="vform" val="fin"/>
		  <feat attr="mood" val="ind"/>
		</fs>
	  </atomcat>
  </xsl:variable> 	

  <xsl:variable name="s.generic.fin.imp">
 	  <atomcat type="s">
	    <fs id="10">
	      <feat attr="index"><lf><nomvar name="E0"/></lf></feat>
	      <feat attr="vform" val="fin"/>
		  <feat attr="mood" val="imp"/>
		</fs>
	  </atomcat>
  </xsl:variable> 	

	    
  <xsl:variable name="s.generic.non-fin">
 	  <atomcat type="s">
	    <fs id="10">
	      <feat attr="index"><lf><nomvar name="E0"/></lf></feat>
	      <feat attr="vform" val="non-fin"/>	  
		</fs>
	  </atomcat>
  </xsl:variable>

 <xsl:variable name="s.generic.progr">
 	  <atomcat type="s">
	    <fs id="10">
	      <feat attr="index"><lf><nomvar name="E0"/></lf></feat>
	      <feat attr="vform" val="progr"/>	  
		</fs>
	  </atomcat>
  </xsl:variable>

  <xsl:variable name="s.from-generic.fin">
    <atomcat type="s">
      <fs inheritsFrom="10">
        <feat attr="vform" val="fin"/>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="s.from-generic">
    <atomcat type="s">
      <fs inheritsFrom="10">
      </fs>
    </atomcat>
  </xsl:variable>


  <xsl:variable name="s.from-generic.fin.imp">
    <atomcat type="s">
      <fs inheritsFrom="10">
        <feat attr="vform" val="fin"/>
		<feat attr="mood" val="imp"/>
      </fs>
    </atomcat>
  </xsl:variable>



  <xsl:variable name="s.from-generic.inf">
    <atomcat type="s">
      <fs inheritsFrom="10">
        <feat attr="vform" val="inf"/>
      </fs>
    </atomcat>
  </xsl:variable>

<!-- generic subject controlled VP 
	... this should get sorted out ...-->

 <xsl:variable name="vp.generic">
   <complexcat>
     <xsl:copy-of select="$s.generic"/>
	 <slash dir="\" mode="&lt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>

 <xsl:variable name="vp.generic.subj-controlled">
   <complexcat>
     <xsl:copy-of select="$s.generic"/>
	 <slash dir="\" mode="&lt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>

<xsl:variable name="vp.from-generic.fin">
   <complexcat>
     <xsl:copy-of select="$s.from-generic.fin"/>
	 <slash dir="\" mode="&lt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>

 <xsl:variable name="vp.from-generic.inf">
   <complexcat>
     <xsl:copy-of select="$s.from-generic.inf"/>
	 <slash dir="\" mode="&lt;" ability="inert"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>
  
 <xsl:variable name="vp.generic.subj-controlled.inf">
   <complexcat>
     <xsl:copy-of select="$s.generic.inf"/>
	 <slash dir="\" mode="&lt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>
 
 <!--
 <xsl:variable name="vp.generic.subj-controlled.non-fin">
   <complexcat>
     <xsl:copy-of select="$s.generic.non-fin"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>
 
 <xsl:variable name="vp.generic.subj-controlled.progr">
   <complexcat>
     <xsl:copy-of select="$s.generic.progr"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.from-subj"/>
   </complexcat>
 </xsl:variable>

 <xsl:variable name="vp.generic.obj-controlled.non-fin">
   <complexcat>
     <xsl:copy-of select="$s.generic.non-fin"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.from-obj"/>
   </complexcat>
 </xsl:variable>

 <xsl:variable name="vp.generic.obj-controlled.inf">
   <complexcat>
     <xsl:copy-of select="$s.generic.inf"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.from-obj"/>
   </complexcat>
 </xsl:variable>
-->

<!-- PRIMARY (MAIN) SENTENCE     --> 
<!-- Sentence, ID 11, Variable E  --> 


  <xsl:variable name="s.main">
	<atomcat type="s">
	  <fs id="11">
    	<feat attr="index"><lf><nomvar name="E"/></lf></feat>
        <feat attr="vform"><lf><nomvar name="VFORM11:vform-vals"/></lf></feat>
        <feat attr="num"><featvar name="NUM2:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS2:pers-vals"/></feat>
	  </fs>
     </atomcat>
  </xsl:variable>
 
  <xsl:variable name="s.main.fin">
    <atomcat type="s">
      <fs id="11">
        <feat attr="vform" val="fin"/>
        <feat attr="index"><lf><nomvar name="E"/></lf></feat>
        <feat attr="num"><featvar name="NUM2:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS2:pers-vals"/></feat>		
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="s.main.progr">
    <atomcat type="s">
      <fs id="11">
        <feat attr="vform" val="progr"/>
        <feat attr="index"><lf><nomvar name="E"/></lf></feat>
        <feat attr="num"><featvar name="NUM2:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS2:pers-vals"/></feat>		
      </fs>
    </atomcat>
  </xsl:variable>


 <xsl:variable name="s.main.fronted">
    <atomcat type="s">
      <fs id="11">
        <feat attr="vform" val="fronted"/>
        <feat attr="index"><lf><nomvar name="E"/></lf></feat>
        <feat attr="num"><featvar name="NUM2:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS2:pers-vals"/></feat>		
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="s.main.int">
	<atomcat type="s">
	  <fs id="11">
    	<feat attr="index"><lf><nomvar name="E"/></lf></feat>
        <feat attr="vform"><lf><nomvar name="VFORM11:vform-vals"/></lf></feat>
        <feat attr="num"><featvar name="NUM2:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS2:pers-vals"/></feat>
		<feat attr="mood" val="int"/>
	  </fs>
     </atomcat>
  </xsl:variable>



  <xsl:variable name="s.from-main.fronted">
    <atomcat type="s">
      <fs inheritsFrom="11">
        <feat attr="vform" val="fronted"/>
     </fs>
    </atomcat>
  </xsl:variable>
  
  <xsl:variable name="vp.main">
   <complexcat>
     <xsl:copy-of select="$s.main"/>
	 <slash dir="\" mode="&lt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>

<xsl:variable name="vp.main.fin">  <!-- used in aux verb" I do like bananas" -->
   <complexcat>
     <xsl:copy-of select="$s.main.fin"/>
	 <slash dir="\" mode="&lt;"/>
	 <xsl:copy-of select="$np.subj"/> 
   </complexcat>
 </xsl:variable>

</xsl:transform>




<!-- ******************* OLD HOPEFULLY OBSOLETE CATEGORIES *********************************** -->
  
  
<!--
  <xsl:variable name="s.main.non-fin">
    <atomcat type="s">
      <fs id="11">
        <feat attr="vform" val="non-fin"/>
        <feat attr="index"><lf><nomvar name="E"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="s.from-main.inf">
    <atomcat type="s">
      <fs inheritsFrom="11">
        <feat attr="vform" val="inf"/>
      </fs>
    </atomcat>
  </xsl:variable>
  
 <xsl:variable name="s.from-main.fin">
    <atomcat type="s">
      <fs inheritsFrom="11">
        <feat attr="vform" val="fin"/>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="s.from-main.non-fin">
    <atomcat type="s">
      <fs inheritsFrom="11">
        <feat attr="vform" val="non-fin"/>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="s.from-main">
    <atomcat type="s">
      <fs inheritsFrom="11"/>
    </atomcat>
  </xsl:variable>
  -->


<!-- Generic vp (same as moodless intransitive...where should this be??) -->



<!--
 <xsl:variable name="vp.main.subj-controlled.non-fin">
   <complexcat>
     <xsl:copy-of select="$s.main.non-fin"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>

<xsl:variable name="vp.main.non-fin">
   <complexcat>
     <xsl:copy-of select="$s.main.non-fin"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>

 <xsl:variable name="vp.from-main.non-fin">
   <complexcat>
     <xsl:copy-of select="$s.from-main.non-fin"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>

<xsl:variable name="vp.from-main">
   <complexcat>
     <xsl:copy-of select="$s.from-main"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>

 <xsl:variable name="vp.from-main.fin">
   <complexcat>
     <xsl:copy-of select="$s.from-main.fin"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>
-->
<!-- generic sentences  When this, when second?? -->


 


 <!-- OLD.  NOW HANDLED VIA MACROS IN VERB DICT ENTRIES
 
 <xsl:variable name="np.subj.dummy-deic">
	<atomcat type="np">
      <fs id="2">
	    <feat attr="nform"><featvar name="dummy-deic"/></feat>
        <feat attr="num"><featvar name="NUM"/></feat>
        <feat attr="pers"><featvar name="PERS"/></feat>
        <feat attr="index"><lf><nomvar name="X"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>


 <xsl:variable name="np.subj.dummy-pers">
	<atomcat type="np">
      <fs id="2">
	    <feat attr="nform"><featvar name="dummy-pers"/></feat>
        <feat attr="num"><featvar name="NUM"/></feat>
        <feat attr="pers"><featvar name="PERS"/></feat>
        <feat attr="index"><lf><nomvar name="X"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>
  
  -->

<!-- THIRD SENTENCE     --> 
<!-- Used for some conjunctions when needed-->   
<!-- Sentence, ID 13, Variable E3,
 
 <xsl:variable name="s.third">
 	  <atomcat type="s">
	    <fs id="13">
	      <feat attr="index"><lf><nomvar name="E3"/></lf></feat>
		</fs>
	  </atomcat>
  </xsl:variable>

 <xsl:variable name="s.third.fin">
 	  <atomcat type="s">
	    <fs id="13">
	      <feat attr="index"><lf><nomvar name="E3"/></lf></feat>
	      <feat attr="vform" val="fin"/>	  
		</fs>
	  </atomcat>
  </xsl:variable>

-->

<!--
 <xsl:variable name="vp.second.subj-controlled">
   <complexcat>
     <xsl:copy-of select="$s.second"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>

<xsl:variable name="vp.from-second.fin">
   <complexcat>
     <xsl:copy-of select="$s.from-second.fin"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>

 <xsl:variable name="vp.from-second.inf">
   <complexcat>
     <xsl:copy-of select="$s.from-second.inf"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>
  
 <xsl:variable name="vp.second.subj-controlled.inf">
   <complexcat>
     <xsl:copy-of select="$s.second.inf"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>
 
 <xsl:variable name="vp.second.subj-controlled.non-fin">
   <complexcat>
     <xsl:copy-of select="$s.second.non-fin"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>
 
 
 <xsl:variable name="vp.second.obj-controlled.non-fin">
   <complexcat>
     <xsl:copy-of select="$s.second.non-fin"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.obj"/>
   </complexcat>
 </xsl:variable>

 <xsl:variable name="vp.second.obj-controlled.inf">
   <complexcat>
     <xsl:copy-of select="$s.second.inf"/>
	 <slash dir="\" mode="&gt;"/>
	 <xsl:copy-of select="$np.obj"/>
   </complexcat>
 </xsl:variable>

-->


<!-- SECONDARY SENTENCE     --> 
<!-- Used for VP complements, also certain "neben" sentences-->   
<!-- Sentence, ID 12, Variable E2,

 <xsl:variable name="s.second">
 	  <atomcat type="s">
	    <fs id="12">
	      <feat attr="index"><lf><nomvar name="E2"/></lf></feat>
		</fs>
	  </atomcat>
  </xsl:variable>
 
  <xsl:variable name="s.second.inf">
 	  <atomcat type="s">
	    <fs id="12">
	      <feat attr="index"><lf><nomvar name="E2"/></lf></feat>
	      <feat attr="vform" val="inf"/>	  
		</fs>
	  </atomcat>
  </xsl:variable>
  
  <xsl:variable name="s.second.fin">
 	  <atomcat type="s">
	    <fs id="12">
	      <feat attr="index"><lf><nomvar name="E2"/></lf></feat>
	      <feat attr="vform" val="fin"/>	  
		</fs>
	  </atomcat>
  </xsl:variable>
	
	    
  <xsl:variable name="s.second.non-fin">
 	  <atomcat type="s">
	    <fs id="12">
	      <feat attr="index"><lf><nomvar name="E2"/></lf></feat>
	      <feat attr="vform" val="non-fin"/>	  
		</fs>
	  </atomcat>
  </xsl:variable>

  <xsl:variable name="s.from-second.fin">
    <atomcat type="s">
      <fs inheritsFrom="12">
        <feat attr="vform" val="fin"/>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="s.from-second.inf">
    <atomcat type="s">
      <fs inheritsFrom="12">
        <feat attr="vform" val="inf"/>
      </fs>
    </atomcat>
  </xsl:variable>
-->


 
 
