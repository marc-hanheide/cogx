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


  <!-- ################################################ -->
  <!--             SYNTACTIC SLOTS                      -->
  <!-- for np, obl, adj, vp, sent used as verb comps    -->
  <!-- ################################################ -->

<!-- CONTROLLED!!  These are all associated with a specific position relative the 
				   controlling verb (ex. subj, obj, indobj, etc)   
				   
				   Each slot is either a np or an oblique (ex. with a book, to the girl)
				   OBLIQUES are simply "case-marked" np's. They are arguments of the verb.
				   Thus, they are different from from standard prep-phrases which are MODIFIERS
				   and have internal category structure
	
				   NOTE: Right now, the actual "prep" (pform) which attaches to obliques are set
						 using macros in the dict. An alternative would be to have obl.obj.with, ...to, etc.
					     This would of course having many many more verb types ex.verb.with-obj, verb.to-obj, ..
				   
				   
         NOTE: Each NUM and PERS feature are also marked with the number of the slot (2 for subj, etc)
			   so that these features are shared by the args across the verb.f
			   Without this, "the girl kicked the balls" wouldn't parse because when "kicked the balls" combined
               the NUM of balls would also set the NUM of the yet un-combined subject slot.
				   
-->


<!-- used as primary subject -->  
<!-- ID 2, variable X -->
  
  <xsl:variable name="np.subj">
	<atomcat type="np">
      <fs id="2">
	    <feat attr="nform" val="basic"/> 
        <feat attr="num"><featvar name="NUM2:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS2:pers-vals"/></feat>
		<feat attr="case" val="nom"/> 
        <feat attr="index"><lf><nomvar name="X:entity"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

	<!-- used in "command" unary rule
  <xsl:variable name="np.subj.addressable">
	<atomcat type="np">
      <fs id="2">
	    <feat attr="nform" val="basic"/>
        <feat attr="num"><featvar name="NUM2:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS2:pers-vals"/></feat>
	    <feat attr="index"><lf><nomvar name="X:entity"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>  -->



<!-- this category used to select verbs with "closed" subject slots, like
	 non-fin, inf, progr, etc 
	 NOTE: must use inheritsFrom here because otherwise, the subject of the sentence
	       would force its num & pers onto this dummy slot, and thus couldn't select for it!
	  
	    -->

 <xsl:variable name="np.dummy-deic-subj">
	<atomcat type="np">
      <fs id="2">
	    <feat attr="nform" val="dummy-deic"/> 
        <feat attr="num"><featvar name="NUM2:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS2:pers-vals"/></feat>
        <feat attr="index"><lf><nomvar name="X:entity"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>
 <xsl:variable name="np.dummy-pers-subj">
	<atomcat type="np">
      <fs id="2">
	    <feat attr="nform" val="dummy-pers"/> 
        <feat attr="num"><featvar name="NUM2:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS2:pers-vals"/></feat>
        <feat attr="index"><lf><nomvar name="X:entity"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>


<!-- used as primary object -->
<!-- ID 3, variable Y -->
  
 <xsl:variable name="np.obj">
	<atomcat type="np">
      <fs id="3">
	    <feat attr="nform" val="basic"/>  
        <feat attr="num"><featvar name="NUM3:num-vals"/></feat>
		<feat attr="case" val="acc"/>
        <feat attr="pers"><featvar name="PERS3:pers-vals"/></feat>
        <feat attr="index"><lf><nomvar name="Y:entity"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="from-np.obj.nom">
	<atomcat type="np">
      <fs inheritsFrom="3">
	  	<feat attr="case" val="nom"/>
      </fs>
    </atomcat>
  </xsl:variable>

   <xsl:variable name="obl.obj">
	<atomcat type="OBL">
      <fs id = "3">	    
        <feat attr="index"><lf><nomvar name="Y:entity"/></lf></feat>
		<feat attr="case" val="acc"/>
	  </fs>
    </atomcat>
 </xsl:variable>

 <xsl:variable name="with.obj">
	<atomcat type="OBL">
      <fs id = "3">
        <feat attr="pform" val="with"/> 	 	    
		<feat attr="case" val="acc"/>
        <feat attr="index"><lf><nomvar name="Y:entity"/></lf></feat>
	  </fs>
    </atomcat>
 </xsl:variable>
 <xsl:variable name="at.obj">
	<atomcat type="OBL">
      <fs id = "3">
        <feat attr="pform" val="at"/> 	 	    
		<feat attr="case" val="acc"/>
        <feat attr="index"><lf><nomvar name="Y:entity"/></lf></feat>
	  </fs>
    </atomcat>
 </xsl:variable>
<xsl:variable name="to.obj">
	<atomcat type="OBL">
      <fs id = "3">
        <feat attr="pform" val="to"/>
		<feat attr="case" val="acc"/> 	 	    
        <feat attr="index"><lf><nomvar name="Y:entity"/></lf></feat>
	  </fs>
    </atomcat>
 </xsl:variable>

 <xsl:variable name="adj.obj">
	<atomcat type="adj">
      <fs id = "3">	    
        <feat attr="index"><lf><nomvar name="Y:quality"/></lf></feat>
	  </fs>
    </atomcat>
 </xsl:variable>

<!-- these two used in complex n/n arg of copular -->
<xsl:variable name="n.obj">
	<atomcat type="n">
      <fs id="3">
        <feat attr="num"><featvar name="NUMY:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERSY:pers-vals"/></feat>
        <feat attr="index"><lf><nomvar name="Y"/></lf></feat>
		<feat attr="nform" val="basic"/>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="n.from-obj">
    <atomcat type="n">
      <fs inheritsFrom="3"/>
    </atomcat>
  </xsl:variable>



<!-- used for indirect objects of DTV -->
<!-- ID 4, variable Z -->

 <xsl:variable name="np.indobj">
	<atomcat type="np">
      <fs id="4">
        <feat attr="nform" val="basic"/> 
        <feat attr="num"><featvar name="NUM4:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS4:pers-vals"/></feat>
		<feat attr="case" val="acc"/>
        <feat attr="index"><lf><nomvar name="Z:entity"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="at.indobj">
	<atomcat type="OBL">
      <fs id = "4">
        <feat attr="pform" val="at"/> 	  
		<feat attr="case" val="acc"/>
        <feat attr="index"><lf><nomvar name="Z:entity"/></lf></feat>
	  </fs>
    </atomcat>
 </xsl:variable>

<xsl:variable name="to.indobj">
	<atomcat type="OBL">
      <fs id = "4">
       <feat attr="pform" val="to"/> 	  
		<feat attr="case" val="acc"/>
       <feat attr="index"><lf><nomvar name="Z:entity"/></lf></feat>
	  </fs>
    </atomcat>
 </xsl:variable>

<xsl:variable name="for.indobj">
	<atomcat type="OBL">
      <fs id = "4">
       <feat attr="pform" val="for"/> 	  
		<feat attr="case" val="acc"/>
       <feat attr="index"><lf><nomvar name="Z:entity"/></lf></feat>
	  </fs>
    </atomcat>
 </xsl:variable>

<xsl:variable name="about.indobj">
	<atomcat type="OBL">
      <fs id = "4">
       <feat attr="pform" val="about"/> 	  
		<feat attr="case" val="acc"/>
       <feat attr="index"><lf><nomvar name="Z:entity"/></lf></feat>
	  </fs>
    </atomcat>
 </xsl:variable>

 <xsl:variable name="adj.indobj">
	<atomcat type="adj">
      <fs id = "4">
        <feat attr="index"><lf><nomvar name="Z"/></lf></feat>
	  </fs>
    </atomcat>
 </xsl:variable>

 <xsl:variable name="pp.indobj">
	<atomcat type="pp">
      <fs id="4">
       <feat attr="index"><lf><nomvar name="Z"/></lf></feat>
      </fs>
    </atomcat>
 </xsl:variable>

 <xsl:variable name="WHERETO">
    <diamond mode="Dynamic"><nomvar name="Z:m-whereto"/></diamond>
 </xsl:variable>


<!--PComp : not marked for specific slot position -->
<!-- ID 6, INDEX PC (Particle Compliment) -->

 <xsl:variable name="prt.vcomp">
	<atomcat type="prt">
      <fs id = "6">
        <feat attr="index"><lf><nomvar name="PC"/></lf></feat>
	  </fs>
    </atomcat>
 </xsl:variable>


<!-- VComp :  Could nail down exact slot-position, but doesn't seem necessary
              but as it seems we only need one, can allow this sloOBLiness -->
<!-- ID 5, INDEX EC (Event, Compliment) -->

<!-- Sentences -->

 <xsl:variable name="s.vcomp">
 	  <atomcat type="s">
	    <fs id="5">
	      <feat attr="index"><lf><nomvar name="EC:event"/></lf></feat>
	    </fs>
	  </atomcat>
 </xsl:variable>


 <xsl:variable name="s.vcomp.non-imp">
 	  <atomcat type="s">
	    <fs id="5">
	      <feat attr="index"><lf><nomvar name="EC:event"/></lf></feat>
  		  <feat attr="mood" val="non-imp"/>
	    </fs>
	  </atomcat>
 </xsl:variable>

 <xsl:variable name="s.vcomp.inf">
 	  <atomcat type="s">
	    <fs id="5">
	      <feat attr="index"><lf><nomvar name="EC:event"/></lf></feat>
		  <feat attr="vform" val="inf"/>	
	    </fs>
	  </atomcat>
 </xsl:variable>

 <xsl:variable name="s.vcomp.int">
 	  <atomcat type="s">
	    <fs id="5">
	      <feat attr="index"><lf><nomvar name="EC:event"/></lf></feat>
		  <feat attr="mood" val="int"/>	
	    </fs>
	  </atomcat>
 </xsl:variable>


<xsl:variable name="s.vcomp.fin">
 	  <atomcat type="s">
	    <fs id="5">
	      <feat attr="index"><lf><nomvar name="EC:event"/></lf></feat>
		  <feat attr="vform" val="fin"/>	
	    </fs>
	  </atomcat>
 </xsl:variable>


<xsl:variable name="s.vcomp.progr">
 	  <atomcat type="s">
	    <fs id="5">
	      <feat attr="index"><lf><nomvar name="EC:event"/></lf></feat>
		  <feat attr="vform" val="progr"/>	
	    </fs>
	  </atomcat>
 </xsl:variable>

<xsl:variable name="s.vcomp.non-fin">
 	  <atomcat type="s">
	    <fs id="5">
	      <feat attr="index"><lf><nomvar name="EC:event"/></lf></feat>
		  <feat attr="vform" val="non-fin"/>	
	    </fs>
	  </atomcat>
 </xsl:variable>

<xsl:variable name="s.vcomp.deictic">
 	  <atomcat type="s">
	    <fs id="5">
	      <feat attr="index"><lf><nomvar name="EC:event"/></lf></feat>
		  <feat attr="vform" val="vform-deictic"/>	
	    </fs>
	  </atomcat>
 </xsl:variable>


<!-- vcomp -->

<!-- NOTE: inf, progr & non-fin vps have "closed" subjects, i.e.
           to disallow sentences like GJ to go, or the girl walking,
		   verbs of these forms are "blocked" from taking a subject -->
		   
<xsl:variable name="vp.vcomp.subj-controlled.inf">
   <complexcat>
     <xsl:copy-of select="$s.vcomp.inf"/>
	 <slash dir="\" mode="&lt;" ability="inert"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>

<xsl:variable name="vp.vcomp.subj-controlled.non-fin">
   <complexcat>
     <xsl:copy-of select="$s.vcomp.non-fin"/>
	 <slash dir="\" mode="&lt;"/>
	 <xsl:copy-of select="$np.subj"/> 
   </complexcat>
 </xsl:variable>

<xsl:variable name="vp.vcomp.subj-controlled.progr">
   <complexcat>
     <xsl:copy-of select="$s.vcomp.progr"/>
	 <slash dir="\" mode="&lt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>

<xsl:variable name="vp.vcomp.obj-controlled.non-fin">
   <complexcat>
     <xsl:copy-of select="$s.vcomp.non-fin"/>
	 <slash dir="\" mode="&lt;"/>
	 <xsl:copy-of select="$from-np.obj.nom"/>
   </complexcat>
 </xsl:variable>

<xsl:variable name="vp.vcomp.obj-controlled.inf">
   <complexcat>
     <xsl:copy-of select="$s.vcomp.inf"/>
	 <slash dir="\" mode="&lt;" ability="inert" />
	 <xsl:copy-of select="$from-np.obj.nom"/>
   </complexcat>
 </xsl:variable>

<xsl:variable name="vp.vcomp.generic-controlled.inf">
   <complexcat>
     <xsl:copy-of select="$s.vcomp.inf"/>
	 <slash dir="\" mode="&lt;" ability="inert" />
	 <xsl:copy-of select="$np.generic"/>
   </complexcat>
 </xsl:variable>


<!-- ==================================================================================== -->
<!-- VERB SEMANTIC ROLES															      -->
<!-- ==================================================================================== -->



 <!-- Actor-Like Subject Roles , variable X -->


<xsl:variable name="ACTOR">
   <diamond mode="Actor"><nomvar name="X:entity"/></diamond>
</xsl:variable>

<xsl:variable name="RESTR">
<diamond mode="Restr"><nomvar name="X"/></diamond>
</xsl:variable>


<!--Patient like Object Roles, variable Y -->

<xsl:variable name="PATIENT">
   <diamond mode="Patient"><nomvar name="Y:entity"/></diamond>
</xsl:variable>

<xsl:variable name="PATIENT-ANIMATE">
   <diamond mode="Patient"><nomvar name="Y:animate"/></diamond>
</xsl:variable>

<xsl:variable name="PATIENT-PHYSICAL">
   <diamond mode="Patient"><nomvar name="Y:physical"/></diamond>
</xsl:variable>

<xsl:variable name="EXISTS"> <!-- match these two -->
   <diamond mode="CtxtRef"><nomvar name="Y:entity"/></diamond> 
</xsl:variable>

<xsl:variable name="SCOPE">
   <diamond mode="Scope"><nomvar name="Y"/></diamond> 
</xsl:variable>



 <!-- <Recipients> , variable Z -->


<xsl:variable name="RECIPIENT">
   <diamond mode="Recipient"><nomvar name="Z:entity"/></diamond>
</xsl:variable>

<xsl:variable name="BENEFACTOR">
   <diamond mode="Benefactor"><nomvar name="Z:entity"/></diamond>
</xsl:variable>


<xsl:variable name="RECIPIENT-ANIMATE">
   <diamond mode="Recipient"><nomvar name="Z:animate"/></diamond>
</xsl:variable>

<xsl:variable name="RECIPIENT-PHYSICAL">
   <diamond mode="Recipient"><nomvar name="Z:physical"/></diamond>
</xsl:variable>

<xsl:variable name="RESULTANT">
   <diamond mode="Resultant"><nomvar name="Z:quality"/></diamond>
</xsl:variable>


<!-- Need Real Names for these... -->

<xsl:variable name="PARTICLE">
<diamond mode="Particle"><nomvar name="PC"/></diamond>
</xsl:variable>

<xsl:variable name="VCOMP">
   <diamond mode="EComp">
      <nomvar name="EC:event"/>
   </diamond> 
</xsl:variable>


</xsl:transform>