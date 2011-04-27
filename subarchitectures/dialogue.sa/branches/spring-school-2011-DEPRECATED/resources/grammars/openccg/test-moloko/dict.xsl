<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">

  <!-- ================================================ -->  
  <!-- OUTPUT OPTIONS                                   -->
  <!-- ================================================ -->  

  <xsl:import href="../test-core/macros.xsl"/>
  <xsl:output indent="yes" xalan2:indent-amount="2"/>
  <xsl:strip-space elements="*"/> 


  <!-- ================================================ -->  
  <!-- DICTIONARY DEFINITION                            -->
  <!-- ================================================ -->  

  <xsl:template match="/">
  <dictionary name="moloko"
    xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="../dict.xsd"
  >

  <!-- ================================================ -->
  <!-- ================================================ -->  
  <!-- =====									 ====== --> 
  <!-- =====          LEXICAL ENTRIES            =====  -->
  <!-- =====									 ====== --> 
  <!-- ================================================ --> 
  <!-- ================================================ -->
 
  <entry pred="and" stem="and__coord" pos="COORD">	
     <word form="and"/>
	 <member-of family="coord.noun.pl-result"/>
 	 <member-of family="coord.noun"/>
	 <member-of family="coord.np.pl-result"/>
 	 <member-of family="coord.np"/>
  	 <member-of family="coord.prep"/>
	 <member-of family="coord.adj"/>
  </entry>

 <entry pred="not" stem="not__coord" pos="MOD">	
     <word form="not"/>
	 <member-of family="negation.adj"/>
 	 <member-of family="negation.prep"/>
	 <member-of family="negation.adj2"/>
  </entry>


  <!-- noun -->
  <entry pred="ball" stem="ball" pos="N">	
     <word form="ball" macros="@num.sg"/>
	 <word form="balls" macros="@num.pl"/>
 	 <member-of family="noun"/>
  </entry>
  <entry pred="mug" stem="mug" pos="N">	
	 <word form="mug" macros="@num.sg"/>
	 <word form="mugs" macros="@num.pl"/>
 	 <member-of family="noun"/>
  </entry>
  <entry pred="table" stem="table" pos="N">	
	 <word form="table" macros="@num.sg"/>
	 <word form="tables" macros="@num.pl"/>
 	 <member-of family="noun"/>
  </entry>

  <entry pred="I" stem="I" pos="N">	
	 <word form="I" macros="@num.sg @nom"/>
	 <word form="me" macros="@num.sg @acc"/>
 	 <member-of family="bare-np"/>
  </entry>
  
  <entry pred="he" stem="he" pos="N">	
	 <word form="he" macros="@num.sg @nom"/>
	 <word form="him" macros="@num.sg @acc"/>
 	 <member-of family="bare-np"/>
  </entry>




  <!-- prep -->
  <entry pred="on" stem="on" pos="PREP" class="to">	
     <word form="on"/>
 	 <member-of family="prep"/>
  </entry>
  
  <entry pred="in" stem="in" pos="PREP">	
     <word form="in"/>
 	 <member-of family="prep"/>
  </entry>
  
  <entry pred="from" stem="from" pos="PREP" class="from">	
     <word form="from"/>
 	 <member-of family="prep"/>
  </entry>

  
  <entry pred="here" stem="here" pos="PREP">	
     <word form="here"/>
 	 <member-of family="pro-prep"/>
  </entry>
  <entry pred="there" stem="there" pos="PREP">	
     <word form="there"/>
 	 <member-of family="pro-prep"/>
  </entry>


  <!-- prep mods -->
  <entry pred="over" stem="over" pos="MOD">	
     <word form="over"/>
 	 <member-of family="mod-right.prep"/>
  </entry>
  <entry pred="up" stem="up" pos="MOD">	
     <word form="up"/>
 	 <member-of family="mod-right.prep"/>
  </entry>

 <!-- adj -->
  <entry pred="big" stem="big" pos="ADJ">	
     <word form="big"/>
 	 <member-of family="adj"/>
  </entry>
  <entry pred="blue" stem="blue" pos="ADJ">	
     <word form="blue"/>
 	 <member-of family="adj"/>
  </entry>
  <entry pred="red" stem="red" pos="ADJ">	
     <word form="red"/>
 	 <member-of family="adj"/>
  </entry>


 <!-- adj mods -->
  <entry pred="really" stem="really" pos="MOD">	
     <word form="really"/>
 	 <member-of family="mod-right.adj"/>
  </entry>
  <entry pred="very" stem="very" pos="MOD">	
     <word form="very"/>
 	 <member-of family="mod-right.adj"/>
  </entry>


<!-- dets -->
 <entry pred="the" stem="the" pos="DET">	
     <word form="the"/>
 	 <member-of family="det"/>
  </entry>
 <entry pred="that" stem="that" pos="DET">	
     <word form="that"/>
 	 <member-of family="det"/>
  </entry>
  <entry pred="this" stem="this" pos="DET">	
     <word form="this"/>
 	 <member-of family="det"/>
  </entry>
 <entry pred="five" stem="five" pos="DET">	
     <word form="five"/>
 	 <member-of family="det"/>
  </entry> 
  <entry pred="a" stem="a" pos="DET">	
     <word form="a"/>
 	 <member-of family="det"/>
  </entry>

 
 <!-- det mods -->
  <entry pred="around" stem="around" pos="MOD">	
     <word form="around"/>
 	 <member-of family="mod-right.det"/>
  </entry>
  <entry pred="nearly" stem="nearly" pos="MOD">	
     <word form="nearly"/>
 	 <member-of family="mod-right.det"/>
  </entry>

  
  
  <entry pred="be" stem="be" pos="V">	
     <word form="is"/>
     <word form="am"/>
	 <member-of family="copular.np"/>
  	 <member-of family="copular.pp"/>
	 <member-of family="copular.adj"/>
  </entry>
  
    
  <entry pred="hit" stem="hit" pos="V">	
     <word form="hit"/>
 	 <member-of family="tv"/>
  </entry>
 
  <entry pred="put" stem="put" pos="V">	
     <word form="put"/>
 	 <member-of family="caused-motion.to"/>
  </entry>

  <entry pred="move" stem="move" pos="V">	
     <word form="move"/>
 	 <member-of family="caused-motion.from.to"/>
  </entry>

  <entry pred="made" stem="made" pos="V">	
     <word form="made"/>
 	 <member-of family="resultant"/>
  </entry>



  <!-- ADD MACROS -->
  <xsl:call-template name="add-macros"/>
 


  </dictionary>
</xsl:template>

</xsl:transform>


