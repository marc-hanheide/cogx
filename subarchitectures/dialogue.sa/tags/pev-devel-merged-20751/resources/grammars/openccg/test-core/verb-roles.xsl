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
  <!--             SV1NTACTIC SLOTS                      -->
  <!-- for np, obl, adj, vp, sent used as verb comps    -->
  <!-- ################################################ -->

<!-- CONTROLLED!!  These are all associated with a specific position relative the 
				   controlling verb (ex. subj, obj, indobj, etc)   
				   
				   Each slot is either a np or an oblique (ex. with a book, to the girl)
				   OBLIQUES are simply "case-marked" np's. They are arguments of the verb.
				   Thus, they are different from from standard prep-phrases which are MODIFIERS
				   and have internal category structure
	
				   NOTE: Right now, the actual "prep" (pform) which attaches to obliques are set
						 using macros in the dict. An alternative would be to have obl.V1.with, ...to, etc.
					     This would of course having many many more verb types ex.verb.with-obj, verb.to-obj, ..
				   
				   
         NOTE: Each NUM and PERS feature are also marked with the number of the slot (2 for subj, etc)
			   so that these features are shared by the args across the verb.f
			   Without this, "the girl kicked the balls" wouldn't parse because when "kicked the balls" combined
               the NUM of balls would also set the NUM of the yet un-combined subject slot.
				   
-->


<!-- used as primary subject, restr -->  
<!-- ID 0, variable X -->
  
  <xsl:variable name="np.subj">
	<atomcat type="np">
      <fs id="0">
        <feat attr="index"><lf><nomvar name="X"/></lf></feat>
    	<feat attr="num"><featvar name="NUM0:num-vals"/></feat>
        <feat attr="pers"><featvar name="PERS0:pers-vals"/></feat>
		<feat attr="case" val="nom"/> 
        <feat attr="nform" val="basic"/> 
      </fs>
    </atomcat>
  </xsl:variable>
 
  <xsl:variable name="with-subj">
     <slash dir="\" mode="&lt;"/>  
     <xsl:copy-of select="$np.subj"/>
  </xsl:variable>  

  <xsl:variable name="ACTOR">
     <diamond mode="Actor"><nomvar name="X:entity"/></diamond>
  </xsl:variable>

  <xsl:variable name="RESTR">
     <diamond mode="Restr"><nomvar name="X"/></diamond>
  </xsl:variable>

<!-- role 1. -->  
<!-- ID 1, variable V1 -->
 
  <xsl:variable name="np.V1">
	<atomcat type="np">
      <fs id="1">
       <feat attr="index"><lf><nomvar name="V1"/></lf></feat>
	   <feat attr="case" val="acc"/>
	   <feat attr="nform" val="basic"/> 
      </fs>
    </atomcat>
  </xsl:variable>
 
  <xsl:variable name="pp.V1">
	<atomcat type="pp">
      <fs id="1">
       <feat attr="index"><lf><nomvar name="V1"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>

  <xsl:variable name="adj.V1">
	<atomcat type="adj">
      <fs id="1">
       <feat attr="index"><lf><nomvar name="V1"/></lf></feat>
      </fs>
    </atomcat>
  </xsl:variable>


  <xsl:variable name="PATIENT">
    <diamond mode="Patient"><nomvar name="V1:entity"/></diamond>
  </xsl:variable>

  <xsl:variable name="SCOPE">
    <diamond mode="Scope"><nomvar name="V1"/></diamond> 
  </xsl:variable> 



<!-- slot V2, ind-obj, resultant, etc -->

 <xsl:variable name="pp.V2">
	<atomcat type="pp">
      <fs id="2">
       <feat attr="index"><lf><nomvar name="V2"/></lf></feat>
      </fs>
    </atomcat>
 </xsl:variable>

 <xsl:variable name="np.V2">
	<atomcat type="np">
      <fs id="2">
       <feat attr="index"><lf><nomvar name="V2"/></lf></feat>
       <feat attr="case" val="acc"/>
	   <feat attr="nform" val="basic"/> 
      </fs>
    </atomcat>
  </xsl:variable>

 <xsl:variable name="adj.V2">
	<atomcat type="adj">
      <fs id="2">
       <feat attr="index"><lf><nomvar name="V2"/></lf></feat>
      </fs>
    </atomcat>
 </xsl:variable>

 <xsl:variable name="RECIPIENT">
   <diamond mode="Recipient"><nomvar name="V2:entity"/></diamond>
 </xsl:variable>

 <xsl:variable name="RESULTANT">
   <diamond mode="Resultant"><nomvar name="V2:quality"/></diamond>
 </xsl:variable>

 <xsl:variable name="WHEREFROM">
   <diamond mode="Dir:WhereFrom"><nomvar name="V2:from"/></diamond>
 </xsl:variable>




<!-- slot V3, senodary-direction (wherefrom etc), etc -->

 <xsl:variable name="pp.V3">
	<atomcat type="pp">
      <fs id="3">
       <feat attr="index"><lf><nomvar name="V3"/></lf></feat>
      </fs>
    </atomcat>
 </xsl:variable>

 <xsl:variable name="WHERETO">
    <diamond mode="Dir:WhereTo"><nomvar name="V3:to"/></diamond>
 </xsl:variable>

</xsl:transform>
