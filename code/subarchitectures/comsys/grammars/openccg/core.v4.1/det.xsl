<?xml version="1.0"?>

<!--
Copyright (C) 2005 DFKI GmbH (Geert-Jan M. Kruijff gj@dfki.de)
Modified: March, 2007 Trevor Benjamin

Added in indefinite and definite
Still no Possessive, must look at "CASE" issue for Gen
Negative articles??
		as is cant parse not a girl, but allows not girl
Not using   det.dem.pl or .sg in families
Took out plural form for indefinite: dont know of a use, and allowed "a balls"


-->

<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">


<!-- ================================================ -->
<!--    CATEGORY DEFINITIONS						  -->
<!-- ================================================ -->

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->
  <!-- det.sg                                    -->
  <!-- Defines the basic category of a determiner,      -->
  <!-- modifying a singular noun                        -->
  <!--                                                  --> 
  <!-- CAT: np[sg] /* n[sg]                             -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->

  <xsl:variable name="det.sg">
    <complexcat>
      <xsl:copy-of select="$np.from-generic"/>
      <slash dir="/" mode="^"/>
      <xsl:copy-of select="$n.generic.sg"/>
    </complexcat>
  </xsl:variable>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->
  <!-- det.pl                                    -->
  <!-- Defines the basic category of a determiner,      -->
  <!-- modifying a plural noun                          -->
  <!--                                                  --> 
  <!-- CAT: np[pl] /* n[pl]                             -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->

  <xsl:variable name="det.pl">
    <complexcat>
      <xsl:copy-of select="$np.from-generic"/>
      <slash dir="/" mode="^"/>
      <xsl:copy-of select="$n.generic.pl"/>	
    </complexcat>
  </xsl:variable>

<!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->
  <!-- det.def.sg                                -->
  <!-- Defines the basic category of a determiner,      -->
  <!-- modifying a singular noun. The npform feature is -->
  <!-- set to "definite".                               -->
  <!--                                                  --> 
  <!-- CAT: np[sg,definite] /* n[sg]                    -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->

  <xsl:variable name="det.def.sg">
    <complexcat>
      <xsl:copy-of select="$np.from-generic.definite"/>
      <slash dir="/" mode="^"/>
      <xsl:copy-of select="$n.generic.sg"/>
    </complexcat>
  </xsl:variable>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->
  <!-- det.dem.sg                                -->
  <!-- Defines the basic category of a determiner,      -->
  <!-- modifying a singular noun. The npform feature is -->
  <!-- set to "demonstrative".                          -->
  <!--                                                  --> 
  <!-- CAT: np[sg,dem] /* n[sg]                    	-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->

  <xsl:variable name="det.dem.sg">
    <complexcat>
      <xsl:copy-of select="$np.from-generic.demonstrative"/>
      <slash dir="/" mode="^"/>
      <xsl:copy-of select="$n.generic.sg"/>
    </complexcat>
  </xsl:variable>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->
  <!-- det.dem.pl                                -->
  <!-- Defines the basic category of a determiner,      -->
  <!-- modifying a plural noun. The npform feature is   -->
  <!-- set to "demonstrative".                          -->
  <!--                                                  --> 
  <!-- CAT: np[pl,dem] /* n[pl]                      	-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->

  <xsl:variable name="det.dem.pl">
    <complexcat>
      <xsl:copy-of select="$np.from-generic.demonstrative"/>
      <slash dir="/" mode="^"/>
      <xsl:copy-of select="$n.generic.pl"/>
    </complexcat>
  </xsl:variable>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->
  <!-- det.def.pl                                -->
  <!-- Defines the basic category of a determiner,      -->
  <!-- modifying a plural noun. The npform feature is   -->
  <!-- set to "definite".                               --> 
  <!--                                                  --> 
  <!-- CAT: np[pl,definite] /* n[pl]                    -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->

  <xsl:variable name="det.def.pl">
    <complexcat>
      <xsl:copy-of select="$np.from-generic.definite"/>
      <slash dir="/" mode="^"/>
      <xsl:copy-of select="$n.generic.pl"/>	
    </complexcat>
  </xsl:variable>
  
  <!-- these are used in quantifiers 
	   I want all of the balls
	   I want all the balls 
    -->

  <xsl:variable name="det.np.pl">
    <complexcat>
      <xsl:copy-of select="$np.from-generic"/>
      <slash dir="/" mode="^"/>
      <xsl:copy-of select="$np.generic.pl"/>
    </complexcat>
  </xsl:variable>
 
  <xsl:variable name="det.of-np">
    <complexcat>
      <xsl:copy-of select="$np.from-generic"/>
      <slash dir="/" mode="^"/>
      <xsl:copy-of select="$of-np.generic"/>
    </complexcat>
  </xsl:variable>

  <!-- MWE for dets -->
  
  <!-- word head word -->
  
  <xsl:variable name="det.pl_W">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($det.pl)/*"/>
      <xsl:with-param name="ext">
	   <slash dir="/" mode="*"/>
 	    <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  
   <xsl:variable name="det.of-np_W">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($det.of-np)/*"/>
      <xsl:with-param name="ext">
	   <slash dir="/" mode="*"/>
 	    <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  
  <xsl:variable name="det.np.pl_W">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($det.np.pl)/*"/>
      <xsl:with-param name="ext">
	   <slash dir="/" mode="*"/>
 	    <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

  <!-- this was for " a lot of" but has no been subsumed by "a lot".of np -->
  <xsl:variable name="det.pl_W_W">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($det.pl)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="*"/>
 	    <xsl:copy-of select="$word.2"/> 
		<slash dir="/" mode="*"/>
 	    <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>


<!-- ================================================ -->
<!--    LEXICAL-MEANING DEFINITIONS					  -->
<!-- ================================================ -->


  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->
  <!-- D.DELIM.UNQ.QUANT.SPECSING.T                     -->
  <!-- Defines the basic meaning of a "definite"        -->
  <!-- modifying a singular object                      -->
  <!--                                                  -->
  <!-- LF: @T &lt;Delimitation&gt;unique          -->
  <!--        ^ &lt;Quantification&gt;specific_singular --> 
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->

  <xsl:variable name="D.DELIM.UNQ.QUANT.SPECSING.T">
    <lf>
       <satop nomvar="T:entity">
          <diamond mode="Delimitation"><prop name="unique"/></diamond>
          <diamond mode="Quantification"><prop name="specific_singular"/></diamond>
       </satop>
    </lf>
  </xsl:variable>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->
  <!-- D.DELIM.UNQ.QUANT.SPECNONSING.T                  -->
  <!-- Defines the basic meaning of a "definite"        -->
  <!-- modifying a plural object                        -->
  <!--                                                  -->
  <!-- LF: @T &lt;Delimitation&gt;unique                -->
  <!--     ^ &lt;Quantification&gt;specific_non-singular--> 
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->

  <xsl:variable name="D.DELIM.UNQ.QUANT.SPECNONSING.T">
    <lf>
       <satop nomvar="T:entity">
          <diamond mode="Delimitation"><prop name="unique"/></diamond>
          <diamond mode="Quantification"><prop name="specific_non-singular"/></diamond>
       </satop>
    </lf>
  </xsl:variable>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->
  <!-- D.DELIM.EX.QUANT.SPECSING.T                      -->
  <!-- Defines the basic meaning of an "indefinite"     -->
  <!-- modifying a singular object                      -->
  <!--                                                  -->
  <!-- LF: @T:thing &lt;Delimitation&gt;existential     -->
  <!--        ^ &lt;Quantification&gt;specific_singular --> 
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->

  <xsl:variable name="D.DELIM.EX.QUANT.SPECSING.T">
    <lf>
       <satop nomvar="T:entity">
          <diamond mode="Delimitation"><prop name="existential"/></diamond>
          <diamond mode="Quantification"><prop name="specific_singular"/></diamond>
       </satop>
    </lf>
  </xsl:variable>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->
  <!-- D.DELIM.EX.QUANT.SPECNONSING.T                   -->
  <!-- Defines the basic meaning of an "indefinite"     -->
  <!-- modifying a plural object                        -->
  <!--                                                  -->
  <!-- LF: @T &lt;Delimitation&gt;existential           -->
  <!--     ^ &lt;Quantification&gt;specific_non-singular--> 
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->

  <xsl:variable name="D.DELIM.EX.QUANT.SPECNONSING.T">
    <lf>
       <satop nomvar="T:entity">
          <diamond mode="Delimitation"><prop name="existential"/></diamond>
          <diamond mode="Quantification"><prop name="specific_non-singular"/></diamond>
       </satop>
    </lf>
  </xsl:variable>


  <xsl:variable name="D.GENOWNER.SINGULAR">
    <lf>
      <satop nomvar="T:entity">
		<diamond mode="GenOwner">
          <nomvar name="P:entity"/>
          <prop name="[*DEFAULT*]"/>
		  <diamond mode="Num"><prop name="sg"/></diamond>
        </diamond>
      </satop>
    </lf>
  </xsl:variable>
  
  <xsl:variable name="D.GENOWNER.PLURAL">
    <lf>
      <satop nomvar="T:entity">
        <diamond mode="GenOwner">
          <nomvar name="P:entity"/>
          <prop name="[*DEFAULT*]"/>
		  <diamond mode="Num"><prop name="pl"/></diamond>
        </diamond>
      </satop>
    </lf>
  </xsl:variable>


<!-- used for measure words attaches <Quantity> type to noun -->  
  <xsl:variable name="D.QUANTITY">
    <lf>
      <satop nomvar="T:entity">
        <diamond mode="Quantity">
          <nomvar name="P:entity"/>
          <prop name="[*DEFAULT*]"/>
        </diamond>
      </satop>
    </lf>
  </xsl:variable>

  <!--  ARE WE USING THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->
  <!-- D.DELIM.EX.QUANT.NEGATIVE.T                      -->
  <!-- Defines the basic meaning of a  "negative"       -->
  <!-- modifying an object                              -->
  <!--                                                  -->
  <!-- LF: @T &lt;Delimitation&gt;existential			-->
  <!--     ^ &lt;Quantification&gt;negative             --> 
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->

  <xsl:variable name="D.DELIM.EX.QUANT.NEGATIVE.T">
    <lf>
       <satop nomvar="T:entity">
          <diamond mode="Delimitation"><prop name="existential"/></diamond>
          <diamond mode="Quantification"><prop name="negative"/></diamond>
       </satop>
    </lf>
  </xsl:variable>


  <!-- ================================================ -->  
  <!-- LEXICAL FAMILY DEFINITIONS                       -->
  <!-- ================================================ -->  

  <xsl:template name="add-det-families">



  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    
  <!-- FAMILY: determiner.definite                      -->
  <!-- Sign:   np[sg,PERS]:T							-->
  <!--         @T(*DEFAULT* ^ <Delimitation>unique      -->
  <!--           ^ <Quantification>specific_singular)   -->
  <!-- Sign:   np[pl,PERS]:T                            -->
  <!--         @T(*DEFAULT* ^ <Delimitation>unique      -->
  <!--           ^ <Quantification>specific_nonsingular)-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    

  <family name="determiner.definite" pos="DET" indexRel="Delimitation" closed="true">
    <entry name="n.sg">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.def.sg)/*"/>
        <xsl:with-param name="ext" select="$D.DELIM.UNQ.QUANT.SPECSING.T"/>
      </xsl:call-template>
    </entry>
    <entry name="n.pl">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.def.pl)/*"/>
        <xsl:with-param name="ext" select="$D.DELIM.UNQ.QUANT.SPECNONSING.T"/>
      </xsl:call-template>
    </entry>
  </family>
  
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    
  <!-- FAMILY: determiner.indefinite                    -->
  <!-- Sign:   np[sg,PERS]:T                            -->
  <!--         @T(*DEFAULT* ^ <Delimitation>existential -->
  <!--           ^ <Quantification>specific_singular)   -->
  <!-- Sign:   np[pl,PERS]:T                            -->
  <!--         @T(*DEFAULT* ^ <Delimitation>existential -->
  <!--           ^ <Quantification>specific_nonsingular)-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    

  <family name="determiner.indefinite" pos="DET" indexRel="Delimitation" closed="true">
    <entry name="n.sg">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.sg)/*"/>
        <xsl:with-param name="ext" select="$D.DELIM.EX.QUANT.SPECSING.T"/>
      </xsl:call-template>
    </entry>
	<!-- This entry allows " a balls" to parse
    <entry name="n.pl">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.pl)/*"/>
        <xsl:with-param name="ext" select="$D.DELIM.EX.QUANT.SPECNONSING.T"/>
      </xsl:call-template>
    </entry> -->
  </family> 


  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    
  <!-- FAMILY: determiner.demonstrative                 -->
  <!-- Sign:   np[sg,PERS]:T                       -->
  <!--         @T(*DEFAULT* ^ <Delimitation>unique      -->
  <!--           ^ <Quantification>specific_singular)   -->
  <!-- Sign:   np[pl,PERS]:T                       -->
  <!--         @T(*DEFAULT* ^ <Delimitation>unique      -->
  <!--           ^ <Quantification>specific_nonsingular)-->
  <!--							-->
  <!-- NOTE: spatial proximity needs to be set by macro --> 
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    

  <family name="determiner.demonstrative.sg" pos="DET" indexRel="Delimitation" closed="true">
    <entry name="n.sg">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.sg)/*"/>
        <xsl:with-param name="ext" select="$D.DELIM.UNQ.QUANT.SPECSING.T"/>
      </xsl:call-template>
    </entry>
	<entry name="n.pl">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.pl)/*"/>
        <xsl:with-param name="ext" select="$D.DELIM.UNQ.QUANT.SPECSING.T"/>
      </xsl:call-template>
    </entry>
  </family>

  <family name="determiner.demonstrative.pl" pos="DET" indexRel="Delimitation" closed="true">
    <entry name="n.sg">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.sg)/*"/>
        <xsl:with-param name="ext" select="$D.DELIM.UNQ.QUANT.SPECNONSING.T"/>
      </xsl:call-template>
    </entry>
	<entry name="n.pl">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.pl)/*"/>
        <xsl:with-param name="ext" select="$D.DELIM.UNQ.QUANT.SPECNONSING.T"/>
      </xsl:call-template>
    </entry>
  </family>
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    
  <!-- FAMILY: determiner.possessive                    -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->
      
  <family name="determiner.possessive.sg-owner" pos="DET" indexRel="Delimitation" closed="true">
    <entry name="n.sg">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.sg)/*"/>
        <xsl:with-param name="ext" select="$D.GENOWNER.SINGULAR"/>
      </xsl:call-template>
    </entry>
	<entry name="n.pl">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.pl)/*"/>
        <xsl:with-param name="ext" select="$D.GENOWNER.SINGULAR"/>
      </xsl:call-template>
    </entry>
  </family>
  
  
  <family name="determiner.possessive.pl-owner" pos="DET" indexRel="Delimitation" closed="true">
    <entry name="n.sg">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.sg)/*"/>
        <xsl:with-param name="ext" select="$D.GENOWNER.PLURAL"/>
      </xsl:call-template>
    </entry>
	<entry name="n.pl">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.pl)/*"/>
        <xsl:with-param name="ext" select="$D.GENOWNER.PLURAL"/>
      </xsl:call-template>
    </entry>
  </family>

    
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    
  <!-- FAMILY: determiner.negative                      -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    
  
  
  <family name="determiner.negative" pos="DET" indexRel="Delimitation" closed="true">
    <entry name="n.sg">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.def.sg)/*"/>
        <xsl:with-param name="ext" select="$D.DELIM.EX.QUANT.NEGATIVE.T"/>
      </xsl:call-template>
    </entry>
    <entry name="n.pl">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.def.pl)/*"/>
        <xsl:with-param name="ext" select="$D.DELIM.EX.QUANT.NEGATIVE.T"/>
      </xsl:call-template>
    </entry>
  </family>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    
  <!-- FAMILY: determiner.quantity (some, all, etc)      -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->   
  
  
 <!-- I want some balls -->
  <family name="determiner.quantity" pos="DET" indexRel="Delimitation" closed="true">
    <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.pl)/*"/>
        <xsl:with-param name="ext" select="$D.QUANTITY"/>
      </xsl:call-template>
    </entry>
  </family>

  <family name="determiner.quantity.sg" pos="DET" indexRel="Delimitation" closed="true">
    <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.sg)/*"/>
        <xsl:with-param name="ext" select="$D.QUANTITY"/>
      </xsl:call-template>
    </entry>
  </family>
 
  <!-- I want lots of balls -->
    <family name="determiner-of-np.quantity" pos="DET" indexRel="Delimitation" closed="true">
    <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.of-np)/*"/>
        <xsl:with-param name="ext" select="$D.QUANTITY"/>
      </xsl:call-template>
    </entry>
  </family>
  
  <!-- I want all the balls -->
  <family name="determiner-np.quantity" pos="DET" indexRel="Delimitation" closed="true">
    <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.np.pl)/*"/>
        <xsl:with-param name="ext" select="$D.QUANTITY"/>
      </xsl:call-template>
    </entry>
  </family>
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->    
  <!-- FAMILY: determiner.quantity 
            (a lot, a few a couple) when used non-compositionally  -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  
 <family name="determiner.quantity_W" pos="DET" indexRel="Delimitation" closed="true">
    <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.pl_W)/*"/>
        <xsl:with-param name="ext" select="$D.QUANTITY"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="determiner-of-np.quantity_W" pos="DET" indexRel="Delimitation" closed="true">
    <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.of-np_W)/*"/>
        <xsl:with-param name="ext" select="$D.QUANTITY"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="determiner-np.quantity_W" pos="DET" indexRel="Delimitation" closed="true">
    <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.np.pl_W)/*"/>
        <xsl:with-param name="ext" select="$D.QUANTITY"/>
      </xsl:call-template>
    </entry>
  </family>


<!-- this was for " a lot of" but has no been subsumed by "a lot".of np -->
<family name="determiner.quantity_W_W" pos="DET" indexRel="Delimitation" closed="true">
    <entry name="basic">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($det.pl_W_W)/*"/>
        <xsl:with-param name="ext" select="$D.QUANTITY"/>
      </xsl:call-template>
    </entry>
  </family>

</xsl:template>
</xsl:transform>