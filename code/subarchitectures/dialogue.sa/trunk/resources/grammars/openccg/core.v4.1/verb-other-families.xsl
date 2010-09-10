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


<xsl:template name="add-verb-other-families">


  <!-- ==================================================================================== -->
  <!-- INTRANSITIVES                                                                        -->
  <!-- ==================================================================================== -->

 <family name="iv" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR"/>
      </xsl:call-template>
    </entry>
 </family>
 <family name="iv.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR"/>
      </xsl:call-template>
    </entry>
 </family>

 <family name="iv.prt" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.prt)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PARTICLE"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="iv.prt.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.prt.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PARTICLE"/>
      </xsl:call-template>
    </entry>
  </family>

  <!-- ==================================================================================== -->
  <!-- TRANSITIVES                                                                          -->
  <!-- ==================================================================================== -->

  <family name="tv" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="tv.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT"/>
      </xsl:call-template>
    </entry>
  </family>
  
   <family name="tv_W" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj_W)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="tv.NO-SUBJ_W" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.NO-SUBJ_W)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT"/>
      </xsl:call-template>
    </entry>
  </family>


<!-- with oblique object -->
 <family name="tv.at-obj" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.at-obj)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT"/>
      </xsl:call-template>
    </entry>
  </family>
 <family name="tv.at-obj.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.at-obj.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT"/>
      </xsl:call-template>
    </entry>
  </family>
 
  <family name="tv.at-obj_W_W" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.at-obj_W_W)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="tv.at-obj.NO-SUBJ_W_W" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.at-obj.NO-SUBJ_W_W)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT"/>
      </xsl:call-template>
    </entry>
  </family> 
 
 <family name="tv.to-obj" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.to-obj)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT"/>
      </xsl:call-template>
    </entry>
  </family>
 <family name="tv.to-obj.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.to-obj.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT"/>
      </xsl:call-template>
    </entry>
  </family>

 
 <family name="tv.with-obj" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.with-obj)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT"/>
      </xsl:call-template>
    </entry>
  </family>
 <family name="tv.with-obj.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.with-obj.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT"/>
      </xsl:call-template>
    </entry>
  </family>

 <!--semantic restrictions on object -->
 <family name="tv.physical-obj" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT-PHYSICAL"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="tv.physical-obj.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT-PHYSICAL"/>
      </xsl:call-template>
    </entry>
  </family>
 
 <family name="tv.animate-obj" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT-ANIMATE"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="tv.animate-obj.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT-ANIMATE"/>
      </xsl:call-template>
    </entry>
  </family>



 <family name="tv.prt" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.prt)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.PARTICLE"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="tv.prt.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.prt.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.PARTICLE"/>
      </xsl:call-template>
    </entry>
  </family>


  <!-- ==================================================================================== -->
  <!-- CAUSED MOTION                                                                        -->
  <!-- ==================================================================================== -->

  <family name="caused-motion" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.pp)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.WHERETO"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="caused-motion.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.pp.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.WHERETO"/>
      </xsl:call-template>
    </entry>
  </family>



  <!-- ==================================================================================== -->
  <!-- DITRANSITIVES                                                                        -->
  <!-- ==================================================================================== -->

  <family name="dtv" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.indobj)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.RECIPIENT"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="dtv.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.indobj.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.RECIPIENT"/>
      </xsl:call-template>
    </entry>
  </family>

  <family name="dtv.animate-indobj" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.indobj)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.RECIPIENT-ANIMATE"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="dtv.animate-indobj.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.indobj.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.RECIPIENT-ANIMATE"/>
      </xsl:call-template>
    </entry>
  </family>
 
  <family name="dtv.to-indobj" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.to-indobj)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.RECIPIENT"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="dtv.to-indobj.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.to-indobj.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.RECIPIENT"/>
      </xsl:call-template>
    </entry>
  </family>

 <family name="dtv.for-indobj" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.for-indobj)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.BENEFACTOR"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="dtv.for-indobj.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.for-indobj.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.BENEFACTOR"/>
      </xsl:call-template>
    </entry>
  </family>


 <family name="dtv.about-indobj" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.about-indobj)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.RECIPIENT"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="dtv.about-indobj.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.about-indobj.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.RECIPIENT"/>
      </xsl:call-template>
    </entry>
  </family>


<!--    
  <family name="dtv.deverbal-obj.at-indobj" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.dtv.at-indobj)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.DEVERBAL-OBJ"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="dtv.deverbal-obj.at-indobj.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.dtv.at-indobj.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.DEVERBAL-OBJ"/>
      </xsl:call-template>
    </entry>
  </family>
-->

  <!-- ==================================================================================== -->
  <!-- ADJECTIVE OBJECT VERBS                                                               -->
  <!-- ==================================================================================== -->

 <family name="v.adj-comp" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.adj)/*"/>
        <xsl:with-param name="ext" select="$E.RESTR.SCOPE"/>
      </xsl:call-template>
    </entry>
  </family>
 <family name="v.adj-comp.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.adj.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.RESTR.SCOPE"/>
      </xsl:call-template>
    </entry>
  </family>


  <!-- ==================================================================================== -->
  <!-- RESULTANT VERBS                                                                      -->
  <!-- ==================================================================================== -->

 <family name="resultant-verb" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.adj)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.RESULTANT"/>
      </xsl:call-template>
    </entry>
  </family>
 <family name="resultant-verb.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.adj.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.RESULTANT"/>
      </xsl:call-template>
    </entry>
  </family>


  <!-- ==================================================================================== -->
  <!-- VERB COMPLEMENTS (DO THESE EXIST, NON MODAL                                          -->
  <!-- ==================================================================================== -->

  <family name="v.sentence-comp" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.sentence)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="v.sentence-comp.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.sentence.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>
  
   <family name="v.deictic-sentence-comp" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.deictic-sentence)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="v.non-fin-sentence-comp.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.deictic-sentence.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>


 <family name="v.non-imp-sentence-comp" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.non-imp-sentence)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="v.non-imp-sentence-comp.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.non-imp-sentence.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>
  
  
  <family name="v.obj.sentence-comp" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.sentence)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="v.obj.sentence-comp.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.sentence.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>
  

  <family name="v.subj-controlled-inf-verb-comp" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.inf-verb)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="v.subj-controlled-inf-verb-comp.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.inf-verb.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>
 
 
  <family name="v.subj-controlled-non-fin-verb-comp" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.non-fin-verb)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="v.subj-controlled-non-fin-verb-comp.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.non-fin-verb.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>
 <family name="v.subj-controlled-non-fin-verb-comp_W" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.non-fin-verb_W)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="v.subj-controlled-non-fin-verb-comp.NO-SUBJ_W" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.non-fin-verb.NO-SUBJ_W)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>


 <family name="v.subj-controlled-progr-verb-comp" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.progr-verb)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="v.subj-controlled-progr-verb-comp.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.progr-verb.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>

  
  <family name="v.obj-controlled-non-fin-verb-comp" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.non-fin-verb)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="v.obj-controlled-non-fin-verb-comp.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.non-fin-verb.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.VCOMP"/>
      </xsl:call-template>
    </entry>	
  </family>
  
  
  <family name="v.obj-controlled-inf-verb-comp" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.inf-verb)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>
  <family name="v.obj-controlled-inf-verb-comp.NO-SUBJ" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.inf-verb.NO-SUBJ)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.VCOMP"/>
      </xsl:call-template>
    </entry>
  </family>

  	 
  <!-- ==================================================================================== -->
  <!-- "Existence" verb with dummy subject (you   got a ball on the table)                  -->
  <!-- ==================================================================================== -->

  <family name="existence.dummy-deic-subject" pos="V"  indexRel="CtxtRef" closed="true">
   <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.dummy-deic-subj)/*"/>
        <xsl:with-param name="ext" select="$E.EXISTENCE"/>
      </xsl:call-template>
    </entry>
  </family>

  <family name="existence.dummy-deic-subject.inverted" indexRel="CtxtRef" pos="V" closed="true">
   <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.dummy-deic-subj.inverted)/*"/>
        <xsl:with-param name="ext" select="$E.EXISTENCE"/>
      </xsl:call-template>
    </entry>
  </family>

  <family name="existence.dummy-pers-subject" indexRel="CtxtRef" pos="V" closed="true">
   <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.obj.dummy-pers-subj)/*"/>
        <xsl:with-param name="ext" select="$E.EXISTENCE"/>
      </xsl:call-template>
    </entry>
  </family>
  
</xsl:template>

</xsl:transform>

 