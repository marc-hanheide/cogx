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

<!-- ==================================================================================== -->
<!-- GENERIC NOUNS AND NPS                                                                -->
<!-- ==================================================================================== -->

<!-- GENERIC!!!  Used in "any old" np, i.e., no assigned function (like verb role, pp-obj, etc)
			     Are later "bound" to a specific functioned specified by the controller 
                 used within noun phrases, also in adj, dets, -->

<!-- ID 1, variable T -->

 <xsl:variable name="np.generic">  
	<atomcat type="np">
      <fs id="1">
        <feat attr="num"><featvar name="NUM:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS:pers-vals"/></feat>
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
		<feat attr="nform" val="basic"/>
		<feat attr="modifiable" val="-"/>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="np.generic.modifiable">  
	<atomcat type="np">
      <fs id="1">
        <feat attr="num"><featvar name="NUM:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS:pers-vals"/></feat>
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
        <feat attr="nform"><featvar name="NFORM:nform"/></feat>
		<feat attr="modifiable" val="+"/>
      </fs>
    </atomcat>
  </xsl:variable>


 <xsl:variable name="np.generic.pl">  
	<atomcat type="np">
      <fs id="1">
        <feat attr="num" val="pl"/>
        <feat attr="pers"><featvar name="PERS:pers-vals"/></feat>
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
		<feat attr="nform" val="basic"/>
     	<feat attr="modifiable" val="-"/>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="np.generic.sg">  
	<atomcat type="np">
      <fs id="1">
        <feat attr="num" val="sg"/>
        <feat attr="pers"><featvar name="PERS:pers-vals"/></feat>
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
		<feat attr="nform" val="basic"/>
 		<feat attr="modifiable" val="-"/>
      </fs>
    </atomcat>
  </xsl:variable>

<xsl:variable name="np.generic.3rd.sg">  
	<atomcat type="np">
      <fs id="1">
        <feat attr="num" val="sg"/>
        <feat attr="pers" val="3rd"/>
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
		<feat attr="nform" val="basic"/>
 		<feat attr="modifiable" val="-"/>
      </fs>
    </atomcat>
  </xsl:variable>

<xsl:variable name="np.generic.3rd.pl">  
	<atomcat type="np">
      <fs id="1">
        <feat attr="num" val="pl"/>
        <feat attr="pers" val="3rd"/>
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
		<feat attr="nform" val="basic"/>
 		<feat attr="modifiable" val="-"/>
      </fs>
    </atomcat>
  </xsl:variable>


 <xsl:variable name="np.generic.sg.modifiable">  
	<atomcat type="np">
      <fs id="1">
        <feat attr="num" val="sg"/>
        <feat attr="pers"><featvar name="PERS:pers-vals"/></feat>
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
		<feat attr="nform" val="basic"/>
 		<feat attr="modifiable" val="+"/>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="np.generic.closed">
	<atomcat type="np">
       <fs inheritsFrom="1">
	    <feat attr="num"  val="num-na"/>
        <feat attr="pers" val="pers-na"/>
       </fs>
    </atomcat>
  </xsl:variable>


 <xsl:variable name="np.generic.definite">  
 	<atomcat type="np">
      <fs id="1">
        <feat attr="num"><featvar name="NUM:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS:pers-vals"/></feat>
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
		<feat attr="form" val="definite"/>
		<feat attr="nform" val="basic"/>
		<feat attr="modifiable" val="-"/>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="np.from-generic.definite">
    <atomcat type="np">
      <fs inheritsFrom="1">
        <feat attr="form" val="definite"/>
 		<feat attr="modifiable" val="-"/>
      </fs>	
    </atomcat>
  </xsl:variable>

  <xsl:variable name="np.from-generic.demonstrative">
    <atomcat type="np">
      <fs inheritsFrom="1">
        <feat attr="form" val="dem"/>
 		<feat attr="modifiable" val="-"/>
      </fs>	
    </atomcat>
  </xsl:variable>

  <xsl:variable name="np.from-generic">
    <atomcat type="np">
      <fs inheritsFrom="1">
	  </fs>	
	</atomcat>
  </xsl:variable>
 
  <xsl:variable name="np.from-generic.non-modifiable">
    <atomcat type="np">
      <fs inheritsFrom="1">
        <feat attr="modifiable" val="-"/>
	  </fs>	
	</atomcat>
  </xsl:variable>
 
  <xsl:variable name="np.from-generic.modifiable">
    <atomcat type="np">
      <fs inheritsFrom="1">
        <feat attr="nform"><featvar name="NFORM:nform"/></feat>
        <feat attr="modifiable" val="+"/>
	  </fs>	
	</atomcat>
  </xsl:variable>

<!-- used for question word nps (what, who, where) -->
  
  <xsl:variable name="whnp.obj">
    <atomcat type="np">
      <fs id="1">
	     <feat attr="nform" val="WHNP-obj"/>  
         <feat attr="modifiable" val="+"/>
         <feat attr="index"><lf><nomvar name="T"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="whnp.scomp">
    <atomcat type="np">
      <fs id="1">
	     <feat attr="nform" val="WHNP-scomp"/>  
         <feat attr="modifiable" val="+"/>
         <feat attr="index"><lf><nomvar name="E0"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="whnp.pp-mod">
    <atomcat type="np">
      <fs id="1">
		 <feat attr="nform" val="WHNP-pp-mod"/>  
         <feat attr="modifiable" val="+"/>
		 <feat attr="index"><lf><nomvar name="T"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>


 <xsl:variable name="whnp.obj.with-mod">
    <complexcat>
      <xsl:copy-of select="$whnp.obj"/>
	  <slash dir="/" mode="."/>
      <xsl:copy-of select="$n.second"/>
    </complexcat>
  </xsl:variable>   
  


<!-- NOUNS -->
  
<xsl:variable name="n.generic">
	<atomcat type="n">
      <fs id="1">
        <feat attr="num"><featvar name="NUM:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS:pers-vals"/></feat>
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
		<feat attr="nform" val="basic"/>
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="n.generic.sg">
	<atomcat type="n">
      <fs id="1">
        <feat attr="num" val="sg"/>
        <feat attr="pers"><featvar name="PERS:pers-vals"/></feat>
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
		<feat attr="nform" val="basic"/>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="n.generic.pl">
	<atomcat type="n">
      <fs id="1">
        <feat attr="num" val="pl"/>
        <feat attr="pers"><featvar name="PERS:pers-vals"/></feat>
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
		<feat attr="nform" val="basic"/>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="n.generic.3rd">
	<atomcat type="n">
      <fs id="1">
        <feat attr="num"><featvar name="NUM:num-vals"/></feat>
        <feat attr="pers" val="3rd"/>
        <feat attr="index"><lf><nomvar name="T"/></lf></feat>
		<feat attr="nform" val="basic"/>
      </fs>
    </atomcat>
  </xsl:variable>
  
  <xsl:variable name="n.from-generic">
    <atomcat type="n">
      <fs inheritsFrom="1"/>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="n.from-generic.non-modifiable">
    <atomcat type="n">
       <fs inheritsFrom="1">
        <feat attr="modifiable" val="-"/>
	  </fs>	
    </atomcat>
  </xsl:variable>

<!-- used in deitics for second outside np -->
 
 <xsl:variable name="np.second">  
	<atomcat type="np">
      <fs id="8">
        <feat attr="num"><featvar name="NUM8:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS8:pers-vals"/></feat>
        <feat attr="index"><lf><nomvar name="T2"/></lf></feat>
		<feat attr="nform" val="basic"/>
		<feat attr="modifiable" val="-"/>
      </fs>
    </atomcat>
  </xsl:variable>
 
  <xsl:variable name="np.second.modifiable">  
	<atomcat type="np">
      <fs id="8">
        <feat attr="num"><featvar name="NUM8:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS8:pers-vals"/></feat>
        <feat attr="index"><lf><nomvar name="T2"/></lf></feat>
        <feat attr="nform"><featvar name="NFORM8:nform"/></feat>
   		<feat attr="modifiable" val="+"/>
	  </fs>
    </atomcat>
  </xsl:variable>

    

<!-- used in whnp with mods like "what colour" 
	uses same index-id as np arg of preps  -->

<xsl:variable name="n.second">
	<atomcat type="n">
      <fs id="8">
        <feat attr="num"><featvar name="NUM8:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS8:pers-vals"/></feat>
        <feat attr="index"><lf><nomvar name="T2"/></lf></feat>
		<feat attr="nform" val="basic"/>
      </fs>
    </atomcat>
  </xsl:variable>
  
  <xsl:variable name="n.from-second">
    <atomcat type="n">
      <fs inheritsFrom="8"/>
    </atomcat>
  </xsl:variable>


<!-- Complex n categories for use in measure words -->

  <xsl:variable name="n.of-np-arg">
     <complexcat>
       <xsl:copy-of select="$n.second"/>
	    <slash dir="/" mode="^"/>
        <xsl:copy-of select="$of-np.generic"/> 
     </complexcat>  
  </xsl:variable>

 <xsl:variable name="of-np.generic">
	<atomcat type="OBL">
      <fs id="1">
	  <feat attr="pform" val="of"/>
	  <feat attr="index"><lf><nomvar name="T:entity"/></lf></feat>
	  </fs>
    </atomcat>
  </xsl:variable>
 
 <!-- Multi-word n categories for use in compounds and the like -->

  <xsl:variable name="n.generic.sg_W">
     <complexcat>
       <xsl:copy-of select="$n.generic.sg"/>
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/> 
     </complexcat>
   </xsl:variable>  

 <xsl:variable name="n.generic.pl_W">
      <complexcat>
       <xsl:copy-of select="$n.generic.pl"/>
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/> 
     </complexcat>
 </xsl:variable>  
 
  <xsl:variable name="np.generic.pl_W">
     <complexcat>
       <xsl:copy-of select="$np.generic.pl"/>
		 <slash dir="/" mode="*"/>
         <xsl:copy-of select="$word.1"/> 
     </complexcat>
  </xsl:variable>  
 
 
 <!-- attempts to handle   room 101  & leibniz room -->
  <xsl:variable name="head-np.number">
     <complexcat>
       <xsl:copy-of select="$np.generic"/>
	    <slash dir="/" mode="*"/>
        <xsl:copy-of select="$number.second"/> 
     </complexcat>
  </xsl:variable>  


  <xsl:variable name="number.second">
    <atomcat type="num">
      <fs id="8">
        <feat attr="index"><lf><nomvar name="T2"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>
 
  <xsl:variable name="dep-np.head-np">
     <complexcat>
        <xsl:copy-of select="$np.generic"/>
		<slash dir="\" mode="*"/>
        <xsl:copy-of select="$np.second"/> 
     </complexcat>
  </xsl:variable>  
 
 
  <!-- ================================================ -->  
  <!-- LEXICAL-MEANING DEFINITIONS                      -->
  <!-- ================================================ -->  

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    
  <!-- Defines the basic meaning of a thing             -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    

  <xsl:variable name="ENTITY">
    <lf>
      <satop nomvar="T:entity">
		<prop name="[*DEFAULT*]"/>
      </satop>
    </lf>
  </xsl:variable>

 <xsl:variable name="EVENT">
    <lf>
      <satop nomvar="E0">
		<prop name="[*DEFAULT*]"/>
      </satop>
    </lf>
  </xsl:variable>

  
  <xsl:variable name="ENTITY.SINGULAR">
    <lf>
      <satop nomvar="T:entity">
		<prop name="[*DEFAULT*]"/>
		<diamond mode="Number"><prop name="sg"/></diamond>
      </satop>
    </lf>
  </xsl:variable>
  
  <xsl:variable name="ENTITY.PLURAL">
    <lf>
      <satop nomvar="T:entity">
		<prop name="[*DEFAULT*]"/>
		<diamond mode="Number"><prop name="pl"/></diamond>
      </satop>
    </lf>
  </xsl:variable>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    
  <!-- Defines the basic meaning of a plural thing      -->
  <!-- without a determiner "balls are round"           -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <xsl:variable name="ENTITY.PLURAL.DELIMITED">
    <lf>
      <satop nomvar="T:entity">
		<prop name="[*DEFAULT*]"/>
		<diamond mode="Number"><prop name="pl"/></diamond>
		<diamond mode="Delimitation"><prop name="variable"/></diamond>
        <diamond mode="Quantification"><prop name="unspecific_non-singular"/></diamond>
      </satop>
    </lf>
  </xsl:variable>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    
  <!-- Defines the basic meaning of a mass noun      -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    

  <xsl:variable name="ENTITY.MASS">
    <lf>
      <satop nomvar="T:entity">
		<prop name="[*DEFAULT*]"/>
		<diamond mode="Delimitation"><prop name="variable"/></diamond>
        <diamond mode="Quantification"><prop name="uncountable"/></diamond>
      </satop>
    </lf>
  </xsl:variable>
  
  <xsl:variable name="ENTITY.VIS-CTXT">
    <lf>
      <satop nomvar="T2:entity">
	    <prop name="[*DEFAULT*]"/>
	    <diamond mode="VisCtxt"> 
		    <nomvar name="T:entity"/>
		    <prop name="entity"/>
		</diamond>
      </satop>
    </lf>
  </xsl:variable>


<!-- used for "one" pronoun as n, so the ones -->
<xsl:variable name="ENTITY.SIT-CTXT">
    <lf>
      <satop nomvar="T2:entity">
	    <prop name="[*DEFAULT*]"/>
		<diamond mode="SitCtxt"> 
		    <nomvar name="T:entity"/>
		    <prop name="entity"/>
		</diamond>
      </satop>
    </lf>
  </xsl:variable>

<!-- used for "one" pronoun as np, so I want one -->
  <xsl:variable name="ENTITY.SIT-CTXT.EXISTENTIAL.SPEC-NON-SING">
    <lf>
      <satop nomvar="T2:entity">
	    <prop name="[*DEFAULT*]"/>
		<diamond mode="Delimitation"><prop name="existential"/></diamond>
     	<diamond mode="Quantification"><prop name="specific_non-singular"/></diamond>
	    <diamond mode="SitCtxt"> 
		    <nomvar name="T:entity"/>
		    <prop name="entity"/>
		</diamond>
      </satop>
    </lf>
  </xsl:variable>
  
  <!-- used for QW-NP with delimiter, e.g. "Which book" do you want -->
  
 <xsl:variable name="ENTITY.MODIFIER">
    <lf>
      <satop nomvar="T:entity">
	    <prop name="[*DEFAULT*]"/>
	    <diamond mode="Modifier"> <nomvar name="T2:entity"/></diamond>
      </satop>
    </lf>
  </xsl:variable>

 <!-- used for measure words such as "a pile of books" -->
 
 <xsl:variable name="ENTITY.MEASURE">
    <lf>
      <satop nomvar="T2:entity">
        <prop name="[*DEFAULT*]"/>
         <diamond mode="Of">
		   <nomvar name="T:entity"/>
         </diamond>
      </satop>
    </lf>
 </xsl:variable>
 
 <!-- for use in "possessed" pronouns like mine yours -->
 <xsl:variable name="ENTITY.SG-OWNER.SINGULAR">
    <lf>
      <satop nomvar="T:entity">
        <prop name="owned-entity"/>
		<diamond mode="GenOwner">
          <nomvar name="OWNER:entity"/>
          <prop name="[*DEFAULT*]"/>
		  <diamond mode="Num"><prop name="sg"/></diamond>
        </diamond>
        <diamond mode="Num"><prop name="sg"/></diamond>
	  </satop>
    </lf>
  </xsl:variable>

 <xsl:variable name="ENTITY.SG-OWNER.PLURAL">
    <lf>
      <satop nomvar="T:entity">
        <prop name="owned-entity"/>
		<diamond mode="GenOwner">
          <nomvar name="OWNER:entity"/>
          <prop name="[*DEFAULT*]"/>
		  <diamond mode="Num"><prop name="sg"/></diamond>
        </diamond>
        <diamond mode="Num"><prop name="pl"/></diamond>
	  </satop>
    </lf>
  </xsl:variable>
 	 
  <xsl:variable name="ENTITY.PL-OWNER.SINGULAR">
    <lf>
      <satop nomvar="T:entity">
        <prop name="owned-entity"/>
		<diamond mode="GenOwner">
          <nomvar name="OWNER:entity"/>
          <prop name="[*DEFAULT*]"/>
		  <diamond mode="Num"><prop name="pl"/></diamond>
        </diamond>
        <diamond mode="Num"><prop name="sg"/></diamond>
	  </satop>
    </lf>
  </xsl:variable>

 <xsl:variable name="ENTITY.PL-OWNER.PLURAL">
    <lf>
      <satop nomvar="T:entity">
        <prop name="owned-entity"/>
		<diamond mode="GenOwner">
          <nomvar name="OWNER:entity"/>
          <prop name="[*DEFAULT*]"/>
		  <diamond mode="Num"><prop name="pl"/></diamond>
        </diamond>
        <diamond mode="Num"><prop name="pl"/></diamond>
	  </satop>
    </lf>
  </xsl:variable>


<!-- used in room 101 & leibniz room -->

  <xsl:variable name="ENTITY.IDENTIFIER">
    <lf>
      <satop nomvar="T:entity">
        <prop name="[*DEFAULT*]"/>
    	<diamond mode="Identifier">
            <nomvar name="T2:e-identifier"/> 
	    </diamond>
      </satop>
    </lf>
  </xsl:variable>


  <!-- ================================================ -->  
  <!-- LEXICAL FAMILY DEFINITIONS                       -->
  <!-- ================================================ -->  

  <xsl:template name="add-n-families">

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    
  <!-- FAMILY: nouns                                    -->
  <!-- Sign:   n[NUM:num-vals,PERS:pers-vals]:T							-->
  <!--         @T(*DEFAULT*)                            -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    

  <family name="noun" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($n.generic.sg)/*"/>
        <xsl:with-param name="ext" select="$ENTITY.SINGULAR"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="noun_W" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($n.generic.sg_W)/*"/>
        <xsl:with-param name="ext" select="$ENTITY.SINGULAR"/>
      </xsl:call-template>
    </entry>
  </family>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    
  <!-- Handles both "boys are cool" and "the boys ran"  -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    

  <family name="noun.plural" pos="N" closed="true">
    <entry name="np.generic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np.generic.pl)/*"/>  
        <xsl:with-param name="ext" select="$ENTITY.PLURAL.DELIMITED"/>
      </xsl:call-template>
    </entry>
    <entry name="plural-noun">
	  <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($n.generic.pl)/*"/>
        <xsl:with-param name="ext" select="$ENTITY.PLURAL"/>
      </xsl:call-template>
   </entry>
  </family>

 <family name="noun.plural_W" pos="N" closed="true">
    <entry name="np.generic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np.generic.pl_W)/*"/>  
        <xsl:with-param name="ext" select="$ENTITY.PLURAL.DELIMITED"/>
      </xsl:call-template>
    </entry>
    <entry name="plural-noun">
	  <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($n.generic.pl_W)/*"/>
        <xsl:with-param name="ext" select="$ENTITY.PLURAL"/>
      </xsl:call-template>
   </entry>
  </family>

  <family name="noun.mass" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($np.generic)/*"/>
        <xsl:with-param name="ext" select="$ENTITY.MASS"/>
      </xsl:call-template>
    </entry>
  </family>
  
   <family name="bare-np" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np.generic.sg)/*"/>  
        <xsl:with-param name="ext" select="$ENTITY"/>  <!-- must get real semantics -->
      </xsl:call-template>
    </entry>
  </family>

 <family name="bare-np.modifiable" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np.generic.sg.modifiable)/*"/>  
        <xsl:with-param name="ext" select="$ENTITY"/>  <!-- must get real semantics -->
      </xsl:call-template>
    </entry>
  </family>

   
 <!-- pronouns -->
 
  <family name="personal-pronoun.sg" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np.generic.sg)/*"/>  
        <xsl:with-param name="ext" select="$ENTITY.SINGULAR"/>  <!-- must get real semantics -->
      </xsl:call-template>
    </entry>
  </family>
  <family name="personal-pronoun.pl" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np.generic.pl)/*"/>  
        <xsl:with-param name="ext" select="$ENTITY.PLURAL"/>  <!-- must get real semantics -->
      </xsl:call-template>
    </entry>
  </family>


  <family name="deictic-pronoun" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np.second)/*"/>  
        <xsl:with-param name="ext" select="$ENTITY.VIS-CTXT"/> 
      </xsl:call-template>
    </entry>
  </family>


 <!-- mine, yours, etc -->
 <family name="possessed-pronoun.sg" pos="N" closed="true">
    <entry name="sg">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np.generic.3rd.sg)/*"/>  
        <xsl:with-param name="ext" select="$ENTITY.SG-OWNER.SINGULAR"/> 
	    </xsl:call-template>
    </entry>
	<entry name="pl">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np.generic.3rd.pl)/*"/>  
        <xsl:with-param name="ext" select="$ENTITY.SG-OWNER.PLURAL"/> 
	    </xsl:call-template>
    </entry>
  </family>
  
  <family name="possessed-pronoun.pl" pos="N" closed="true">
   <entry name="sg">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np.generic.3rd.sg)/*"/>  
        <xsl:with-param name="ext" select="$ENTITY.PL-OWNER.SINGULAR"/> 
	    </xsl:call-template>
    </entry>
	<entry name="pl">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np.generic.3rd.pl)/*"/>  
        <xsl:with-param name="ext" select="$ENTITY.PL-OWNER.PLURAL"/> 
	    </xsl:call-template>
    </entry>
  </family>

  <!-- the one on the table,  I want one with a .. -->
  <family name="one-pronoun.n" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($n.second)/*"/>  
        <xsl:with-param name="ext" select="$ENTITY.SIT-CTXT"/> 
      </xsl:call-template>
    </entry>
  </family>
    
  <family name="one-pronoun.np" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np.second.modifiable)/*"/>  
        <xsl:with-param name="ext" select="$ENTITY.SIT-CTXT.EXISTENTIAL.SPEC-NON-SING"/> 
      </xsl:call-template>
    </entry>
  </family>
  
 <!-- Question word NP what, which, where -->
  
  <family name="question-word.obj" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($whnp.obj)/*"/>  
        <xsl:with-param name="ext" select="$ENTITY"/>  <!-- must get real semantics -->
      </xsl:call-template>
    </entry>
    <entry name="with-mod">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($whnp.obj.with-mod)/*"/>  
        <xsl:with-param name="ext" select="$ENTITY.MODIFIER"/>  <!-- must get real semantics -->
      </xsl:call-template>
    </entry>
  </family>

 <family name="question-word.scomp" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($whnp.scomp)/*"/>  
        <xsl:with-param name="ext" select="$EVENT"/>  <!-- must get real semantics -->
      </xsl:call-template>
    </entry>
  </family>


  <family name="question-word.mod" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($whnp.pp-mod)/*"/>  
        <xsl:with-param name="ext" select="$ENTITY"/>  <!-- must get real semantics -->
      </xsl:call-template>
    </entry>
  </family>


  <family name="dummy-np" pos="N" indexRel="*nosem*" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($np.generic)/*"/>
      </xsl:call-template>
    </entry>
  </family>


	<!-- Measure Words such as pile, group, bunch, etc. -->
  <family name="measure-word" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($n.of-np-arg)/*"/>
        <xsl:with-param name="ext" select="$ENTITY.MEASURE"/>
      </xsl:call-template>
    </entry>
  </family>


	<!-- N-N compound type deals....anyway, being used for Room 101 & Leibniz Room-->
  
  <family name="compound-head-number" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($head-np.number)/*"/>
        <xsl:with-param name="ext" select="$ENTITY.IDENTIFIER"/>
      </xsl:call-template>
    </entry>
  </family>

 <family name="compound-id-head" pos="N" closed="true">
    <entry name="default">
      <xsl:call-template name="extend">
		<xsl:with-param name="elt" select="xalan:nodeset($dep-np.head-np)/*"/>
        <xsl:with-param name="ext" select="$ENTITY.IDENTIFIER"/>
      </xsl:call-template>
    </entry>
  </family>




 
  </xsl:template>

</xsl:transform>