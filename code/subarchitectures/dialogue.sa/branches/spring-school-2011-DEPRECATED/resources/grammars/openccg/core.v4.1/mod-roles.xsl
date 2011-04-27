<?xml version="1.0"?>
<!--
Copyright (C) 2005 DFKI GmbH (Geert-Jan M. Kruijff gj@dfki.de)
-->

<!-- These roles are the dependency relations used by all kinds of modifiers,
     i.e. event modifiers like advs and prepositions, and entity modifiers like
	 adjectives and prepositions 

	 For the actual full sEMA:eventntics of these modifiers see mod-event-sEMA:eventntics.xsl
													  and mod-entity-sEMA:eventntics.xsl
-->


<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">
  


<!-- SINCE CHANGING, I'VE ACTUALLY SEEN THESE 
<xsl:copy-of select="$vp.mod-arg.subj-controlled.non-fin"/> in adv
<xsl:copy-of select="$s.mod-arg.fin"/> in adv
<xsl:copy-of select="$np.mod-arg"/> in preposition

-->

  <!-- ################################################ -->
  <!--             SYNTACTIC SLOTS                      -->
  <!-- for np and sentence args used in modifiers       -->
  <!-- ################################################ -->

  
  <!-- used in prepositions -->
  <!-- ID 30, INDEX TMA:entity (Entity, Modifier Argument) -->
  
  <xsl:variable name="np.mod-arg">  
	<atomcat type="np">
      <fs id="30">
        <feat attr="num"><featvar name="NUMMA:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERSMA:pers-vals"/></feat>
		<feat attr="case" val="acc"/>
        <feat attr="index"><lf><nomvar name="TMA:entity"/></lf></feat>
		<feat attr="nform" val="basic"/>
      </fs>
    </atomcat>
  </xsl:variable>
 
   <xsl:variable name="n.mod-arg">  
	<atomcat type="n">
      <fs id="30">
        <feat attr="num"><featvar name="NUMMA:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERSMA:pers-vals"/></feat>
		<feat attr="case" val="acc"/>
        <feat attr="index"><lf><nomvar name="TMA:entity"/></lf></feat>
		<feat attr="nform" val="basic"/>
      </fs>
    </atomcat>
  </xsl:variable>
 
   <xsl:variable name="MOD">
    <lf>
      <satop nomvar="M">
		<prop name="[*DEFAULT*]"/>
      </satop>
    </lf>
 </xsl:variable>

  <xsl:variable name="MOD.ARG">
    <lf>
      <satop nomvar="M">
		<prop name="[*DEFAULT*]"/>
        <xsl:copy-of select="$ARG"/>
      </satop>
    </lf>
  </xsl:variable>

 <xsl:variable name="MOD.ANCHOR">
    <lf>
      <satop nomvar="M">
		<prop name="[*DEFAULT*]"/>
        <xsl:copy-of select="$ANCHOR"/>
      </satop>
    </lf>
  </xsl:variable>

  <xsl:variable name="MOD-MODIFIER">
	<lf>
	  <satop nomvar="M">
	     <xsl:copy-of select="$MODIFIER"/>
      </satop>
	</lf>
  </xsl:variable>

 <xsl:variable name="MOD-NEGATION">
	<lf>
	  <satop nomvar="M">
	     <xsl:copy-of select="$NEGATIVE"/>
      </satop>
	</lf>
  </xsl:variable>

 <xsl:variable name="MOD.COMPARATIVE">
    <lf>
      <satop nomvar="M">
           <diamond mode="Degree"><prop name="comparative"/></diamond>
      </satop>
    </lf>
 </xsl:variable>

 <xsl:variable name="MOD.SUPERLATIVE">
    <lf>
      <satop nomvar="M">
           <diamond mode="Degree"><prop name="superlative"/></diamond>
      </satop>
    </lf>
 </xsl:variable>

  <!-- ################################################ -->
  

  
  

<!-- used as argument of "proud of" type adjectives -->

 <xsl:variable name="of.mod-arg">
	<atomcat type="OBL">
      <fs id="30">
	  <feat attr="pform" val="of"/>
      <feat attr="case" val="acc"/>
	  <feat attr="index"><lf><nomvar name="TMA:entity"/></lf></feat>
	  </fs>
    </atomcat>
 </xsl:variable>

<!-- careful with -->
 <xsl:variable name="with.mod-arg">
	<atomcat type="OBL">
      <fs id="30">
	  <feat attr="pform" val="with"/>
      <feat attr="case" val="acc"/>
	  <feat attr="index"><lf><nomvar name="TMA:entity"/></lf></feat>
	  </fs>
    </atomcat>
 </xsl:variable>


<!-- used by neben-satz adverbials -->
<!-- ID 31, INDEX EMA:event (Event, Modifier Argument) -->

 <xsl:variable name="s.mod-arg">
 	  <atomcat type="s">
	    <fs id="31">
	      <feat attr="index"><lf><nomvar name="EMA:event"/></lf></feat>
		</fs>
	  </atomcat>
  </xsl:variable>
 
 <xsl:variable name="s.mod-arg.non-fin">
 	  <atomcat type="s">
	    <fs id="31">
	      <feat attr="index"><lf><nomvar name="EMA:event"/></lf></feat>
	      <feat attr="vform" val="non-fin"/>	  
		</fs>
	  </atomcat>
  </xsl:variable>

 <xsl:variable name="s.mod-arg.progr">
 	  <atomcat type="s">
	    <fs id="31">
	      <feat attr="index"><lf><nomvar name="EMA:event"/></lf></feat>
	      <feat attr="vform" val="progr"/>	  
		</fs>
	  </atomcat>
  </xsl:variable>
 <xsl:variable name="s.mod-arg.inf">
 	  <atomcat type="s">
	    <fs id="31">
	      <feat attr="index"><lf><nomvar name="EMA:event"/></lf></feat>
	      <feat attr="vform" val="inf"/>	  
		</fs>
	  </atomcat>
  </xsl:variable>
 
  <xsl:variable name="s.mod-arg.fin">
 	  <atomcat type="s">
	    <fs id="31">
	      <feat attr="index"><lf><nomvar name="EMA:event"/></lf></feat>
	      <feat attr="vform" val="fin"/>	  
		</fs>
	  </atomcat>
  </xsl:variable>

 <xsl:variable name="vp.mod-arg.subj-controlled.non-fin">
   <complexcat>
     <xsl:copy-of select="$s.mod-arg.non-fin"/>
	 <slash dir="\" mode="&lt;"/>
	 <xsl:copy-of select="$np.subj"/>
   </complexcat>
 </xsl:variable>
 
 <xsl:variable name="vp.mod-arg.inf">
   <complexcat>
     <xsl:copy-of select="$s.mod-arg.inf"/>
	 <slash dir="\" mode="&lt;"/>
	 <xsl:copy-of select="$np.generic"/>
   </complexcat>
 </xsl:variable>
 
 
 
  <!-- ################################################ -->
  <!--             SEMA:eventNTIC ROLES                       -->
  <!-- ################################################ -->

 
 
 
  <!-- ################################################ -->
  <!-- Simple Roles with T argument of modifier         -->
  <!-- ################################################ -->
    
	
	
  <xsl:variable name="ARG">
    <diamond mode="Arg"><nomvar name="TMA:entity"/></diamond>
  </xsl:variable>
 
  <xsl:variable name="ANCHOR">
    <diamond mode="Anchor"><nomvar name="TMA:entity"/></diamond>
  </xsl:variable>

  <xsl:variable name="M-ECOMP">
     <diamond mode="EComp"><nomvar name="EMA:event"/></diamond>
  </xsl:variable>

  <xsl:variable name="M-BODY">
	   <diamond mode="Body"><nomvar name="EMA:event"/></diamond>
  </xsl:variable>

<!--
		THESE DIDNT GENERATE BECAUSE NO EXPLICIT MENTION OF PROPISITION (ie. DEFAULT) in SEMANTICS
			SEE BELOW FOR NEW GEN'ING VERSIONS
 -->
 
   <xsl:variable name="OWNER">
    <diamond mode="GenOwner"><nomvar name="TMA:entity"/></diamond>
  </xsl:variable>


  <xsl:variable name="ACCOMPANIMENT">
    <diamond mode="Accompaniment"><nomvar name="TMA:entity"/></diamond>
  </xsl:variable>
  
  <xsl:variable name="M-BENEFACTOR">
    <diamond mode="Benefactor"><nomvar name="TMA:entity"/></diamond>
  </xsl:variable>
  
  <xsl:variable name="PURPOSE-ENTITY">
     <diamond mode="Purpose"><nomvar name="TMA:entity"/></diamond>
  </xsl:variable>

  <xsl:variable name="M-RESTR">
     <diamond mode="Restr"><nomvar name="EMA:event"/></diamond>
  </xsl:variable>



  
  
  <!-- ################################################ --> 
  <!-- Main Roles (include the name (default) of the mod) -->
  <!-- ################################################ -->
  
  <!-- Generic -->
  
  <xsl:variable name="PROPERTY">
     <diamond mode="Property">
        <nomvar name="P"/>
        <prop name="[*DEFAULT*]"/>
     </diamond>
  </xsl:variable>
  
  <xsl:variable name="PROPERTY-WITH-ARG">
     <diamond mode="Property">
	    <nomvar name="PRPA"/>
	    <prop name="[*DEFAULT*]"/> 
       	  <xsl:copy-of select="$ARG"/>
     </diamond>
  </xsl:variable>

  
 <xsl:variable name="MODIFIER">
     <diamond mode="Modifier">
		<nomvar name="M"/>
	    <prop name="[*DEFAULT*]"/> 
     </diamond>
  </xsl:variable>

  <xsl:variable name="MODIFIER-WITH-ARG">
     <diamond mode="Modifier">
	    <nomvar name="MODA"/>
	    <prop name="[*DEFAULT*]"/> 
       	  <xsl:copy-of select="$ARG"/>
     </diamond>
  </xsl:variable>

<!-- Locational and Directional Roles -->

 <xsl:variable name="LOCATION">
 	 <diamond mode="Location">
		<nomvar name="LOC"/>
	    <prop name="[*DEFAULT*]"/> 
     </diamond>
 </xsl:variable>

 <xsl:variable name="LOCATION-WITH-ANCHOR">
     <diamond mode="Location">
	    <nomvar name="LOCA"/>
	    <prop name="[*DEFAULT*]"/> 
       	<xsl:copy-of select="$ANCHOR"/>
     </diamond>
  </xsl:variable>
	      
 <xsl:variable name="WHICHWAY">
    <diamond mode="Dir:WhichWay">
       <nomvar name="WAY"/>
	   <prop name="[*DEFAULT*]"/> 
    </diamond>
 </xsl:variable>

  <xsl:variable name="WHERETO">
     <diamond mode="Dir:WhereTo">
	    <nomvar name="WTO"/>
	    <prop name="[*DEFAULT*]"/> 
       	<xsl:copy-of select="$ANCHOR"/>
     </diamond>
  </xsl:variable>

  <xsl:variable name="WHEREFROM">
     <diamond mode="Dir:WhereFrom">
	    <nomvar name="FRM"/>
	    <prop name="[*DEFAULT*]"/> 
       	<xsl:copy-of select="$ANCHOR"/>
     </diamond>
  </xsl:variable>


  <xsl:variable name="THROUGH">
     <diamond mode="Dir:Through">
	    <nomvar name="THR"/>
	    <prop name="[*DEFAULT*]"/> 
       	<xsl:copy-of select="$ANCHOR"/>
     </diamond>
  </xsl:variable>


<!-- Others 
 <xsl:variable name="OWNER">
    <diamond mode="GenOwner">
	    <nomvar name="GEN"/>
	    <prop name="[*DEFAULT*]"/> 
       	<xsl:copy-of select="$ARG"/>
    </diamond>
  </xsl:variable>

   <xsl:variable name="ACCOMPANIMENT">
    <diamond mode="Accompaniment">
	    <nomvar name="ACC"/>
	    <prop name="[*DEFAULT*]"/> 
       	<xsl:copy-of select="$ARG"/>
    </diamond>
  </xsl:variable>
  
  <xsl:variable name="M-BENEFACTOR">
    <diamond mode="Benefactor">
	    <nomvar name="BEN"/>
	    <prop name="[*DEFAULT*]"/> 
       	<xsl:copy-of select="$ARG"/>
    </diamond>
  </xsl:variable>

   <xsl:variable name="PURPOSE-ENTITY">
     <diamond mode="Purpose">
	    <nomvar name="PURT"/>
	    <prop name="[*DEFAULT*]"/> 
       	<xsl:copy-of select="$ARG"/>
    </diamond>
  </xsl:variable> -->


<!-- for Relative Pronoun 

<xsl:variable name="M-RESTR">
     <diamond mode="Restr">
 	    <nomvar name="RSTR"/>
	    <prop name="[*DEFAULT*]"/> 
       	<xsl:copy-of select="$M-BODY"/>
    </diamond>
  </xsl:variable>
-->

 <!-- Comparative "something like a ball" -->

<xsl:variable name="COMPARISON-ENTITY">
  <diamond mode="Comparative">
      <nomvar name="COMP"/>
      <prop name="[*DEFAULT*]"/>
       	<xsl:copy-of select="$ARG"/>
   </diamond>
</xsl:variable> 

<xsl:variable name="COMPARISON-EVENT">
  <diamond mode="Comparative">
      <nomvar name="COMP"/> 
      <prop name="[*DEFAULT*]"/>
       	<xsl:copy-of select="$M-BODY"/>
   </diamond>
</xsl:variable> 

  <!-- ################################################ -->
  <!-- Simple Roles with E argument of modifier         -->
  <!-- ################################################ -->

 
  <xsl:variable name="RESULT">
     <diamond mode="Result">
	     <nomvar name="EMA:event"/>
 	  </diamond>
  </xsl:variable>

 <xsl:variable name="CAUSE">
     <diamond mode="Cause"> 
    	 <nomvar name="EMA:event"/>
    </diamond>
  </xsl:variable>

 <xsl:variable name="CONDITION">
   <diamond mode="Condition">
      <nomvar name="EMA:event"/>
   </diamond>
  </xsl:variable>

 <xsl:variable name="PURPOSE">
     <diamond mode="Purpose">
         <nomvar name="EMA:event"/>
    </diamond>
 </xsl:variable>

<xsl:variable name="TIME">
  <diamond mode="Time">
      <nomvar name="TIME"/> 
      <prop name="[*DEFAULT*]"/>
       	<xsl:copy-of select="$M-BODY"/>
   </diamond>
</xsl:variable> 



  <!-- Neben-satz-ish roles. Taken from Petkevic/GJ 

  
  <xsl:variable name="RESULT">
     <diamond mode="Result">
 	    <nomvar name="RES"/>
	    <prop name="[*DEFAULT*]"/> 
       	<xsl:copy-of select="$M-BODY"/>
    </diamond>
  </xsl:variable>

 <xsl:variable name="CAUSE">
     <diamond mode="Cause"> 
	    <nomvar name="CAUS"/>
	    <prop name="[*DEFAULT*]"/> 
       	<xsl:copy-of select="$M-BODY"/>
    </diamond>
  </xsl:variable>

 <xsl:variable name="CONDITION">
   <diamond mode="Condition">
   	    <nomvar name="COND"/>
	    <prop name="[*DEFAULT*]"/> 
       	<xsl:copy-of select="$M-BODY"/>
    </diamond>
  </xsl:variable>

 <xsl:variable name="PURPOSE">
     <diamond mode="Purpose">
   	    <nomvar name="PURPE"/>
	    <prop name="[*DEFAULT*]"/> 
       	<xsl:copy-of select="$M-BODY"/>
    </diamond>
  </xsl:variable>
 -->
 
</xsl:transform>