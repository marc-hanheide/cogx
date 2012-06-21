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
  <!-- INTRANSITIVES                                                                        -->
  <!-- ==================================================================================== -->

  <!--Vanilla Intransitive (no complements: BASE)--> 
 
  <xsl:variable name="verb">
    <complexcat>
	  <xsl:copy-of select="$s.main"/>
      <slash dir="\" mode="&lt;"/>   <!-- should this be . or ^-->
	  <xsl:copy-of select="$np.subj"/>
	</complexcat>
  </xsl:variable>  
  
  <xsl:variable name="verb.NO-SUBJ">
    <xsl:copy-of select="$s.main"/>
  </xsl:variable>  
  
  <!-- Note: because verb.NO-SUBJ is an atomic type (unlike ), and 
             xsl:..."extend" inserts between outermost xml tags,
             we must introduce this <complexcat> here for use in extension -->
 
  <xsl:variable name="no-subj.base">
    <complexcat>
      <xsl:copy-of select="$s.main"/>
    </complexcat>
  </xsl:variable>  


  <!--Intransitive with Particle ( BASE/part ) --> 
  
  <xsl:variable name="verb.prt">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$prt.vcomp"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  
  <xsl:variable name="verb.prt.NO-SUBJ">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$prt.vcomp"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

<!-- used to introduce references into discourse, for example There is a boy, Here we got a ball -->
 <xsl:variable name="verb.obj.dummy-deic-subj">    
    <complexcat>
	  <xsl:copy-of select="$s.main"/>
      <slash dir="\" mode="."/>   
	  <xsl:copy-of select="$np.dummy-deic-subj"/>
      <slash dir="/" mode="."/>
	  <xsl:copy-of select="$np.obj"/> 
	</complexcat>
 </xsl:variable>
 
 <xsl:variable name="verb.obj.dummy-deic-subj.inverted">    
  <complexcat>
	  <xsl:copy-of select="$s.main"/>
      <slash dir="/" mode="."/>
	  <xsl:copy-of select="$np.obj"/> 
	  <slash dir="/" mode="."/>   
	  <xsl:copy-of select="$np.dummy-deic-subj"/>
   </complexcat>
 </xsl:variable>

<xsl:variable name="verb.obj.dummy-pers-subj">    
    <complexcat>
	  <xsl:copy-of select="$s.main"/>
      <slash dir="\" mode="."/>  
	  <xsl:copy-of select="$np.dummy-pers-subj"/>
      <slash dir="/" mode="."/>
	  <xsl:copy-of select="$np.obj"/> 
	</complexcat>
 </xsl:variable>



  <!-- ==================================================================================== -->
  <!-- TRANSITIVES                                                                          -->
  <!-- ==================================================================================== -->

  <!--Vanilla Transitive ( BASE/np )--> 

  <xsl:variable name="verb.obj">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$np.obj"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
 
   <xsl:variable name="verb.obj_W">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb.obj)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="*"/>
 	    <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

    
  <xsl:variable name="verb.obj.NO-SUBJ">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$np.obj"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  
   <xsl:variable name="verb.obj.NO-SUBJ_W">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.NO-SUBJ)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="*"/>
 	    <xsl:copy-of select="$word.1"/> 
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

 
  <!-- TV with Verb Particles-  ex pick up the book, pick the book up -->
  <!-- Use setarg to handle variable ordering of part and obj         -->
  <!--       ( BASE/{/np /part}  )                                      -->
  
  <xsl:variable name="verb.obj.prt">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	  <setarg>
		<slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$np.obj"/>
		<slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$prt.vcomp"/>
	  </setarg>
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>  
  
  <xsl:variable name="verb.obj.prt.NO-SUBJ">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">
	  <setarg>
		<slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$np.obj"/>
		<slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$prt.vcomp"/>
	  </setarg>
	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>  

  <!-- TV whose object is oblique-  ex play with the ball -->
  <!--          ( BASE/obl  )                              -->


 <xsl:variable name="verb.at-obj">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$at.obj"/> 
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  <xsl:variable name="verb.at-obj.NO-SUBJ">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$at.obj"/> 
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

<xsl:variable name="verb.with-obj">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$with.obj"/> 
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  <xsl:variable name="verb.with-obj.NO-SUBJ">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$with.obj"/> 
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>


 <xsl:variable name="verb.to-obj">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$to.obj"/> 
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  <xsl:variable name="verb.to-obj.NO-SUBJ">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$to.obj"/> 
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

<!-- Verb with Adj obj  (I feel good, etc) -->
 
 <xsl:variable name="verb.adj">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$adj.obj"/>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
 <xsl:variable name="verb.adj.NO-SUBJ">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$adj.obj"/>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>


 <!-- Verb with obj and adj (used in resultant)-  ex wipe the table clean -->
 <!--          ( BASE/obj/adj  )                              -->
 
 <xsl:variable name="verb.obj.adj">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$adj.indobj"/>
		<slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$np.obj"/>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
 <xsl:variable name="verb.obj.adj.NO-SUBJ">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$adj.indobj"/>
		<slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$np.obj"/>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

  <!-- ================================================ -->  
  <!-- CAUSED MOTION                                    -->
  <!-- ================================================ -->    
  
  <!-- Vanilla Put like verb  
          ( BASE/np/pp )             -->  
  
  <xsl:variable name="verb.obj.pp">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$pp.indobj"/>
		<slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$np.obj"/>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

 <xsl:variable name="verb.obj.pp.NO-SUBJ">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$pp.indobj"/>
		<slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$np.obj"/>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

  <!-- ================================================ -->  
  <!-- DITRANSITIVE                                     -->
  <!-- ================================================ -->    
  
  <!-- Vanilla Ditransitive 
          ( BASE/np/np )             -->  
  
  <xsl:variable name="verb.obj.indobj">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb.obj)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$np.indobj"/>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

  <xsl:variable name="verb.obj.indobj.NO-SUBJ">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.NO-SUBJ)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$np.indobj"/>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  
  
  <!-- Ditransitive with Oblique indirect object (ex. give him the book ) 
                 (BASE/obj/np          -->
  
   <xsl:variable name="verb.obj.to-indobj">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	   <setarg>
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$to.indobj"/> 
        <slash dir="/" mode="&gt;"/> 
 	    <xsl:copy-of select="$np.obj"/> 
	  </setarg>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  <xsl:variable name="verb.obj.to-indobj.NO-SUBJ">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">
      <setarg>
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$to.indobj"/> 
        <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$np.obj"/> 
   	   </setarg>
     </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

 <xsl:variable name="verb.obj.for-indobj">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	   <setarg>
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$for.indobj"/> 
        <slash dir="/" mode="&gt;"/> 
 	    <xsl:copy-of select="$np.obj"/> 
	  </setarg>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  <xsl:variable name="verb.obj.for-indobj.NO-SUBJ">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">
      <setarg>
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$for.indobj"/> 
        <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$np.obj"/> 
   	   </setarg>
     </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>


 <xsl:variable name="verb.obj.at-indobj">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	   <setarg>
	     <slash dir="/" mode="&gt;"/>
 	     <xsl:copy-of select="$at.indobj"/> 
         <slash dir="/" mode="&gt;"/>
 	     <xsl:copy-of select="$np.obj"/> 
	   </setarg>
       </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  <xsl:variable name="verb.obj.at-indobj.NO-SUBJ">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">
	   <setarg>
          <slash dir="/" mode="&gt;"/>
 	      <xsl:copy-of select="$at.indobj"/> 
          <slash dir="/" mode="&gt;"/>
 	      <xsl:copy-of select="$np.obj"/>
  	   </setarg>
       </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

<xsl:variable name="verb.obj.about-indobj">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$about.indobj"/> 
        <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$np.obj"/> 
       </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  <xsl:variable name="verb.obj.about-indobj.NO-SUBJ">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$about.indobj"/> 
        <slash dir="/" mode="&gt;"/>
 	    <xsl:copy-of select="$np.obj"/> 
       </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>


  <!-- ================================================ -->  
  <!-- VERBS WITH  SENTENCE, VP, INF, COMPLEMENTS       -->
  <!-- ================================================ --> 

  
  <!--   Verb with a sentence complement 
                    BASE/s                              -->  

  <xsl:variable name="verb.sentence">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
	      <xsl:copy-of select="$s.vcomp.fin"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

 <xsl:variable name="verb.sentence.NO-SUBJ">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
	      <xsl:copy-of select="$s.vcomp.fin"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>


 <xsl:variable name="verb.deictic-sentence">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
	      <xsl:copy-of select="$s.vcomp.deictic"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

 <xsl:variable name="verb.deictic-sentence.NO-SUBJ">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
	      <xsl:copy-of select="$s.vcomp.deictic"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

 
  <!--   Verb with a non-imperative sentence complement 
                    BASE/s                              -->  


<xsl:variable name="verb.non-imp-sentence">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
	      <xsl:copy-of select="$s.vcomp.non-imp"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

<xsl:variable name="verb.non-imp-sentence.NO-SUBJ">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
	      <xsl:copy-of select="$s.vcomp.non-imp"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>


<!--   Verb with object and sentence comp 
                    BASE/s/np                         -->  

  <xsl:variable name="verb.obj.sentence">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
	    <xsl:copy-of select="$s.vcomp.fin"/> 
		<slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$np.obj"/>   
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
 
   <xsl:variable name="verb.obj.sentence.NO-SUBJ">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
	    <xsl:copy-of select="$s.vcomp.fin"/> 
		<slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$np.obj"/>   
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
 
  <!-- Verb with an infinitive subject controlled vp complement 
                     BASE/(s\np)                     
					 
					 **********
					 -->  

  
  <xsl:variable name="verb.inf-verb">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$vp.vcomp.subj-controlled.inf"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

 <xsl:variable name="verb.inf-verb.NO-SUBJ">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$vp.vcomp.subj-controlled.inf"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>


  <!-- Verb with a non-finite subject controlled vp complement 
                     BASE/(s\np)                     -->  

  <xsl:variable name="verb.non-fin-verb">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
	      <xsl:copy-of select="$vp.vcomp.subj-controlled.non-fin"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  <xsl:variable name="verb.non-fin-verb_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb.non-fin-verb)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="*"/>
	      <xsl:copy-of select="$word.1"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

  <xsl:variable name="verb.non-fin-verb.NO-SUBJ">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
	      <xsl:copy-of select="$vp.vcomp.subj-controlled.non-fin"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  <xsl:variable name="verb.non-fin-verb.NO-SUBJ_W">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb.non-fin-verb.NO-SUBJ)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="*"/>
	      <xsl:copy-of select="$word.1"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>


<!-- Verb with a  progressive subject controlled vp complement 
                     BASE/(s\np)                     
		e.x. go running           	-->  

  <xsl:variable name="verb.progr-verb">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
	      <xsl:copy-of select="$vp.vcomp.subj-controlled.progr"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

  <xsl:variable name="verb.progr-verb.NO-SUBJ">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="*"/>
	      <xsl:copy-of select="$vp.vcomp.subj-controlled.progr"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>


<!-- Verb with an object and non-finite subject controlled vp complement 
                     BASE/(s\np)                     -->  


  <xsl:variable name="verb.obj.non-fin-verb">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
	    <xsl:copy-of select="$vp.vcomp.obj-controlled.non-fin"/>
		<slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$np.obj"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

 <xsl:variable name="verb.obj.non-fin-verb.NO-SUBJ">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
	    <xsl:copy-of select="$vp.vcomp.obj-controlled.non-fin"/>
		<slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$np.obj"/> 
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>


 <!-- Verb with an object and infinitive subject controlled vp complement 
                     BASE/(s\np)                     -->  

 <xsl:variable name="verb.obj.inf-verb">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
	    <xsl:copy-of select="$vp.vcomp.obj-controlled.inf"/> 
		<slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$np.obj"/>   
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

 <xsl:variable name="verb.obj.inf-verb.NO-SUBJ">
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($no-subj.base)/*"/>
      <xsl:with-param name="ext">    
	    <slash dir="/" mode="&gt;"/>
	    <xsl:copy-of select="$vp.vcomp.obj-controlled.inf"/> 
		<slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$np.obj"/>   
 	  </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>


<!-- MWE Categories. Actual word slots chosen via macros -->

<!-- take_a_LOOK at the ball -->

<xsl:variable name="verb.at-obj_W_W">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb.at-obj)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="*"/>
        <xsl:copy-of select="$word.2"/>
	    <slash dir="/" mode="*"/>
	    <xsl:copy-of select="$word.1"/>   
      </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>
  
<xsl:variable name="verb.at-obj.NO-SUBJ_W_W">    
    <xsl:call-template name="extend">
      <xsl:with-param name="elt" select="xalan:nodeset($verb.at-obj.NO-SUBJ)/*"/>
      <xsl:with-param name="ext">
	    <slash dir="/" mode="*"/>
        <xsl:copy-of select="$word.2"/>
	    <slash dir="/" mode="*"/>
	    <xsl:copy-of select="$word.1"/>   
     </xsl:with-param>
    </xsl:call-template>
  </xsl:variable>

 </xsl:transform>
 
