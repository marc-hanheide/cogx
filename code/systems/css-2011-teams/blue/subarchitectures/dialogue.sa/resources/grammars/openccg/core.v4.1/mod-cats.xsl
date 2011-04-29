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



 <xsl:variable name="with-dollar">
   <slash/>
   <dollar name="1"/>
</xsl:variable>   


<!-- Mod Sentences and Vps  -->


<!-- these are the tight connections 

	THIS NEEDS TO BE INVESTIGATED. WE'RE NOT GETTING SOME GOOD READINGS

-->

 <xsl:variable name="mod-s.left">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <xsl:copy-of select="$with-dollar"/>
          <slash dir="\" mode="*"/>
		  <xsl:copy-of select="$s.generic"/>
	  </complexcat>
 </xsl:variable>   

 <xsl:variable name="mod-vp.left">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$np.subj"/>
		  <xsl:copy-of select="$with-dollar"/>
		  <slash dir="\" mode="^"/>
		  <xsl:copy-of select="$vp.generic"/>         
       </complexcat>
 </xsl:variable>   

<xsl:variable name="mod-s.right">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <xsl:copy-of select="$with-dollar"/>
          <slash dir="/" mode="^"/>
		  <xsl:copy-of select="$s.generic"/>
	  </complexcat>
 </xsl:variable>   

 <xsl:variable name="mod-vp.right">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$np.subj"/>
		  <xsl:copy-of select="$with-dollar"/>
		  <slash dir="/" mode="^"/>
		  <xsl:copy-of select="$vp.generic"/>         
       </complexcat>
 </xsl:variable>   


 <xsl:variable name="mod-loose-s.left">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <xsl:copy-of select="$with-dollar"/>
          <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$s.generic"/>
	  </complexcat>
 </xsl:variable>   

 <xsl:variable name="mod-loose-vp.left">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$np.subj"/>
		  <xsl:copy-of select="$with-dollar"/>
		  <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$vp.generic"/>         
       </complexcat>
 </xsl:variable>   
 

 <xsl:variable name="mod-loose-vp.left.no-dollar"> 
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$np.subj"/>
		  <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$vp.generic"/>         
       </complexcat>
 </xsl:variable>   


<xsl:variable name="mod-loose-s.right">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <xsl:copy-of select="$with-dollar"/>
          <slash dir="/" mode="&gt;"/>
		  <xsl:copy-of select="$s.generic"/>
	  </complexcat>
 </xsl:variable>   

 <xsl:variable name="mod-loose-vp.right">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$np.subj"/>
		  <xsl:copy-of select="$with-dollar"/>
		  <slash dir="/" mode="&gt;"/>
		  <xsl:copy-of select="$vp.generic"/>         
      </complexcat>
 </xsl:variable>   


<!-- Mod Nouns and Np

	MAJOR CHANGE! Nov. 7
			moved $1 from after top n to after second. 

SO THIS .....
			
<xsl:variable name="mod-n.left">
	<complexcat>
	      <xsl:copy-of select="$n.from-generic"/>
		  <xsl:copy-of select="$with-dollar"/>
          <slash dir="\" mode="*"/>
		  <xsl:copy-of select="$n.generic"/>         
     </complexcat>
 </xsl:variable>    


CHANGES TO THIS .....
			
<xsl:variable name="mod-n.left">
	<complexcat>
	      <xsl:copy-of select="$n.from-generic"/>
          <slash dir="\" mode="*"/>
		  <xsl:copy-of select="$n.generic"/>         
		  <xsl:copy-of select="$with-dollar"/>
     </complexcat>
 </xsl:variable>    
			
	Reason:
			when parsed 'the really' got type np/adj/n b/c 'args' of the pushed to end. Then , 'the really big'
			would just push this same n further, meaning we get 'still waiting for adj' reading

	See Below, bad readings
	
		
	THE REALLY
Parse 2: np{case=CASE1, form=definite, index=T_1:entity, modifiable=-, nform=basic, num=pl, pers=PERS:pers-vals}/adj<5>{aform=AFORM25, index=M0_67:quality, modifiable=+}/^n<160>{index=T_1:entity, nform=basic, num=pl, pers=PERS:pers-vals} : 
  @x1:entity(
             <Delimitation>unique ^ 
             <Quantification>specific_non-singular ^ 
             <Property>(x2:quality ^ 
                        <Modifier>(r1:m-intensity ^ really)))

(lex)  the :- np{case=CASE1, form=definite, index=T_1:entity, modifiable=-, nform=basic, num=pl, pers=PERS:pers-vals}/^n<2>{case=CASE1, index=T_1:entity, nform=basic, num=pl, pers=PERS:pers-vals} : (@T_1:entity(<Delimitation>unique) ^ @T_1:entity(<Quantification>specific_non-singular))
(lex)  really :- adj{aform=AFORM25, index=M_4, modifiable=+}/adj<5>{aform=AFORM25, index=M_4, modifiable=+} : (@M_4(<Modifier>MM_4:m-intensity) ^ @MM_4:m-intensity(really))
(gram) adj: adj<26>{complexcat-type=s-left, index=M0_0:quality, mod-type=s-dynamic, modifiable=+}$1 => n{index=T_0:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals}$1/^n<1>{index=T_0:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals} : @T_0:entity(<Property>M0_0:quality)
(adj)  really :- n{index=T_67:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals}/adj<5>{aform=AFORM25, index=M0_67:quality, modifiable=+}/^n<160>{index=T_67:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals} : (@M0_67:quality(<Modifier>MM_4:m-intensity) ^ @MM_4:m-intensity(really) ^ @T_67:entity(<Property>M0_67:quality))
(>B)   the really :- np{case=CASE1, form=definite, index=T_1:entity, modifiable=-, nform=basic, num=pl, pers=PERS:pers-vals}/adj<5>{aform=AFORM25, index=M0_67:quality, modifiable=+}/^n<160>{index=T_1:entity, nform=basic, num=pl, pers=PERS:pers-vals} : (@M0_67:quality(<Modifier>MM_4:m-intensity) ^ @MM_4:m-intensity(really) ^ @T_1:entity(<Delimitation>unique) ^ @T_1:entity(<Quantification>specific_non-singular) ^ @T_1:entity(<Property>M0_67:quality))

THE REALLY BIG 
Parse 3: np{case=CASE1, form=definite, index=T_1:entity, modifiable=-, nform=basic, num=pl, pers=PERS:pers-vals}/^n<314>{index=T_1:entity, nform=basic, num=pl, pers=PERS:pers-vals} : 
  @x1:entity(
             <Delimitation>unique ^ 
             <Quantification>specific_non-singular ^ 
             <Property>(b1:q-size ^ big ^ 
                        <Modifier>(r1:m-intensity ^ really)))

(lex)  the :- np{case=CASE1, form=definite, index=T_1:entity, modifiable=-, nform=basic, num=pl, pers=PERS:pers-vals}/^n<2>{case=CASE1, index=T_1:entity, nform=basic, num=pl, pers=PERS:pers-vals} : (@T_1:entity(<Delimitation>unique) ^ @T_1:entity(<Quantification>specific_non-singular))
(lex)  really :- adj{aform=AFORM25, index=M_4, modifiable=+}/adj<5>{aform=AFORM25, index=M_4, modifiable=+} : (@M_4(<Modifier>MM_4:m-intensity) ^ @MM_4:m-intensity(really))
(lex)  big :- adj<6>{aform=aform-base, index=M_5:q-size, modifiable=+} : @M_5:q-size(big)
(>)    really big :- adj{aform=aform-base, index=M_5:q-size, modifiable=+} : (@M_5:q-size(big) ^ @M_5:q-size(<Modifier>MM_4:m-intensity) ^ @MM_4:m-intensity(really))
(gram) adj: adj<26>{complexcat-type=s-left, index=M0_0:quality, mod-type=s-dynamic, modifiable=+}$1 => n{index=T_0:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals}$1/^n<1>{index=T_0:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals} : @T_0:entity(<Property>M0_0:quality)
(adj)  really big :- n{index=T_130:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals}/^n<314>{index=T_130:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals} : (@M_5:q-size(big) ^ @M_5:q-size(<Modifier>MM_4:m-intensity) ^ @MM_4:m-intensity(really) ^ @T_130:entity(<Property>M_5:q-size))
(>B)   the really big :- np{case=CASE1, form=definite, index=T_1:entity, modifiable=-, nform=basic, num=pl, pers=PERS:pers-vals}/^n<314>{index=T_1:entity, nform=basic, num=pl, pers=PERS:pers-vals} : (@M_5:q-size(big) ^ @M_5:q-size(<Modifier>MM_4:m-intensity) ^ @MM_4:m-intensity(really) ^ @T_1:entity(<Delimitation>unique) ^ @T_1:entity(<Quantification>specific_non-singular) ^ @T_1:entity(<Property>M_5:q-size))

Parse 4: np{case=CASE1, form=definite, index=T_0:entity, modifiable=-, nform=basic, num=sg, pers=PERS:pers-vals}/adj<5>{aform=AFORM25, index=M0_68:quality, modifiable=+}/^n<236>{index=T_0:entity, nform=basic, num=sg, pers=PERS:pers-vals} : 
  @x1:entity(
             <Delimitation>unique ^ 
             <Quantification>specific_singular ^ 
             <Property>(b1:q-size ^ big) ^ 
             <Property>(x2:quality ^ 
                        <Modifier>(r1:m-intensity ^ really)))

(lex)  the :- np{case=CASE1, form=definite, index=T_0:entity, modifiable=-, nform=basic, num=sg, pers=PERS:pers-vals}/^n<1>{case=CASE1, index=T_0:entity, nform=basic, num=sg, pers=PERS:pers-vals} : (@T_0:entity(<Delimitation>unique) ^ @T_0:entity(<Quantification>specific_singular))
(lex)  really :- adj{aform=AFORM25, index=M_4, modifiable=+}/adj<5>{aform=AFORM25, index=M_4, modifiable=+} : (@M_4(<Modifier>MM_4:m-intensity) ^ @MM_4:m-intensity(really))
(gram) adj: adj<26>{complexcat-type=s-left, index=M0_0:quality, mod-type=s-dynamic, modifiable=+}$1 => n{index=T_0:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals}$1/^n<1>{index=T_0:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals} : @T_0:entity(<Property>M0_0:quality)
(adj)  really :- n{index=T_68:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals}/adj<5>{aform=AFORM25, index=M0_68:quality, modifiable=+}/^n<161>{index=T_68:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals} : (@M0_68:quality(<Modifier>MM_4:m-intensity) ^ @MM_4:m-intensity(really) ^ @T_68:entity(<Property>M0_68:quality))
(lex)  big :- adj<6>{aform=aform-base, index=M_5:q-size, modifiable=+} : @M_5:q-size(big)
(gram) adj: adj<26>{complexcat-type=s-left, index=M0_0:quality, mod-type=s-dynamic, modifiable=+}$1 => n{index=T_0:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals}$1/^n<1>{index=T_0:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals} : @T_0:entity(<Property>M0_0:quality)
(adj)  big :- n{index=T_99:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals}/^n<236>{index=T_99:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals} : (@M_5:q-size(big) ^ @T_99:entity(<Property>M_5:q-size))
(>B)   really big :- n{index=T_68:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals}/adj<5>{aform=AFORM25, index=M0_68:quality, modifiable=+}/^n<236>{index=T_68:entity, nform=basic, num=NUM:num-vals, pers=PERS:pers-vals} : (@M_5:q-size(big) ^ @M0_68:quality(<Modifier>MM_4:m-intensity) ^ @MM_4:m-intensity(really) ^ @T_68:entity(<Property>M_5:q-size) ^ @T_68:entity(<Property>M0_68:quality))
(>B)   the really big :- np{case=CASE1, form=definite, index=T_0:entity, modifiable=-, nform=basic, num=sg, pers=PERS:pers-vals}/adj<5>{aform=AFORM25, index=M0_68:quality, modifiable=+}/^n<236>{index=T_0:entity, nform=basic, num=sg, pers=PERS:pers-vals} : (@M_5:q-size(big) ^ @M0_68:quality(<Modifier>MM_4:m-intensity) ^ @MM_4:m-intensity(really) ^ @T_0:entity(<Delimitation>unique) ^ @T_0:entity(<Quantification>specific_singular) ^ @T_0:entity(<Property>M_5:q-size) ^ @T_0:entity(<Property>M0_68:quality))

	
			
					NOTE: PRE-CHANGE, Lefts had slash type *, but now won't work....		

 -->


<!--  Used in, e.g., adj ( big/n)  ** these two have been changed -->
 <xsl:variable name="mod-n.right">
	<complexcat>
	      <xsl:copy-of select="$n.from-generic"/>
          <slash dir="/" mode="^"/>
		  <xsl:copy-of select="$n.generic"/>         
		  <xsl:copy-of select="$with-dollar"/>  <!-- dollar is after n, i.e. deal with args first, then find modified noun -->
     </complexcat>
 </xsl:variable>    
 
 <xsl:variable name="mod-np.right">
	<complexcat>
	      <xsl:copy-of select="$np.from-generic"/>
          <slash dir="/" mode="^"/>
		  <xsl:copy-of select="$np.generic.modifiable"/>         
		  <xsl:copy-of select="$with-dollar"/>
     </complexcat>
 </xsl:variable>    

<!--  Used in, e.g., pp (on/np\n)  ** these two have NOT been changed -->

 <xsl:variable name="mod-n.left">
	<complexcat>
	      <xsl:copy-of select="$n.from-generic"/>
	      <xsl:copy-of select="$with-dollar"/>  <!-- before n, i.e. find deal with args after finding modified noun -->
	 	  <slash dir="\" mode="*"/>
		  <xsl:copy-of select="$n.generic"/>         
        
	  </complexcat>
 </xsl:variable>    
 
 <xsl:variable name="mod-np.left">
	<complexcat>
	      <xsl:copy-of select="$np.from-generic"/>
          <xsl:copy-of select="$with-dollar"/>
          <slash dir="\" mode="*"/>
		  <xsl:copy-of select="$np.generic.modifiable"/>         
		 
     </complexcat>
 </xsl:variable>    


<!-- WHERE IS MOD and MOD.ARG?? -->
 <xsl:variable name="MOD.ECOMP">
    <lf>
       <satop nomvar="M">
	     <prop name="[*DEFAULT*]"/>
		 <xsl:copy-of select="$M-ECOMP"/> 
	   </satop>
    </lf>
  </xsl:variable>


<!-- SEMANTIC ROLES -->
<xsl:variable name="XLOCATION">
	<diamond mode="Location"><nomvar name="M0:modifier"/></diamond> 
</xsl:variable>    
<xsl:variable name="XTIME">
	<diamond mode="Time"><nomvar name="M0:modifier"/></diamond> 
</xsl:variable>    
<xsl:variable name="XDYNAMIC">
	<diamond mode="Dynamic"><nomvar name="M0:modifier"/></diamond> 
</xsl:variable>    
<xsl:variable name="XMODIFIER">
	<diamond mode="Modifier"><nomvar name="M0:modifier"/></diamond> 
</xsl:variable>    
<xsl:variable name="XPROPERTY">
	<diamond mode="Property"><nomvar name="M0:quality"/></diamond> 
</xsl:variable>  

<!-- MODIFY EVENT AND ENTITY -->

<!-- property (for adj)-->
<xsl:variable name="MOD-T-XPROPERTY">
	<lf>
      <satop nomvar="T:entity">
	     <xsl:copy-of select="$XPROPERTY"/>         
	  </satop>
    </lf> 	
</xsl:variable>


<!-- generic modifier -->
<xsl:variable name="MOD-T-XMODIFIER">
	<lf>
      <satop nomvar="T:entity">
	     <xsl:copy-of select="$XMODIFIER"/>         
	  </satop>
    </lf> 	
</xsl:variable>

<xsl:variable name="MOD-E-XMODIFIER">
	<lf>
      <satop nomvar="E0:m-modifier">
	     <xsl:copy-of select="$XMODIFIER"/>         
	  </satop>
    </lf> 	
</xsl:variable>

<!-- location -->
<xsl:variable name="MOD-T-XLOCATION">
	<lf>
      <satop nomvar="T:entity">
	     <xsl:copy-of select="$XLOCATION"/>         
	  </satop>
    </lf> 	
</xsl:variable>

<xsl:variable name="MOD-E-XLOCATION">
	<lf>
      <satop nomvar="E0:m-location">
	     <xsl:copy-of select="$XLOCATION"/>         
	  </satop>
    </lf> 	
</xsl:variable>


<xsl:variable name="MOD-T-XCOMPARISON">
	<lf>
      <satop nomvar="T:entity">
	     <xsl:copy-of select="$XMODIFIER"/>         
	  </satop>
    </lf> 	
</xsl:variable>

<xsl:variable name="MOD-E-XCOMPARISON">
	<lf>
      <satop nomvar="E0:m-comparison">
	     <xsl:copy-of select="$XMODIFIER"/>         
	  </satop>
    </lf> 	
</xsl:variable>



<xsl:variable name="MOD-E-XDYNAMIC">
	<lf>
      <satop nomvar="E0:m-dynamic">
	     <xsl:copy-of select="$XDYNAMIC"/>         
	  </satop>
    </lf> 	
</xsl:variable>    

<xsl:variable name="MOD-E-XTIME">
	<lf>
      <satop nomvar="E0:m-time">
	     <xsl:copy-of select="$XTIME"/>         
	  </satop>
    </lf> 	
</xsl:variable>        

<xsl:variable name="MOD-E-XPROBABILITY">
	<lf>
      <satop nomvar="E0:m-probability">
	     <xsl:copy-of select="$XMODIFIER"/>         
	  </satop>
    </lf> 	
</xsl:variable>

<xsl:variable name="MOD-E-XMANNER">
	<lf>
      <satop nomvar="E0:m-manner">
	     <xsl:copy-of select="$XMODIFIER"/>         
	  </satop>
    </lf> 	
</xsl:variable>

<xsl:variable name="MOD-E-XCOMMENT">
	<lf>
      <satop nomvar="E0:m-comment">
	     <xsl:copy-of select="$XMODIFIER"/>         
	  </satop>
    </lf> 	
</xsl:variable>



</xsl:transform>


<!-- BACK UP MOD S and VP.... Done before testing new dollar positioning (learned from n)

 <xsl:variable name="mod-s.left">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <xsl:copy-of select="$with-dollar"/>
          <slash dir="\" mode="*"/>
		  <xsl:copy-of select="$s.generic"/>
	  </complexcat>
 </xsl:variable>   

 <xsl:variable name="mod-vp.left">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$np.subj"/>
		  <xsl:copy-of select="$with-dollar"/>
		  <slash dir="\" mode="^"/>
		  <xsl:copy-of select="$vp.generic"/>         
       </complexcat>
 </xsl:variable>   

<xsl:variable name="mod-s.right">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <xsl:copy-of select="$with-dollar"/>
          <slash dir="/" mode="^"/>
		  <xsl:copy-of select="$s.generic"/>
	  </complexcat>
 </xsl:variable>   

 <xsl:variable name="mod-vp.right">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$np.subj"/>
		  <xsl:copy-of select="$with-dollar"/>
		  <slash dir="/" mode="^"/>
		  <xsl:copy-of select="$vp.generic"/>         
       </complexcat>
 </xsl:variable>   


 <xsl:variable name="mod-loose-s.left">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <xsl:copy-of select="$with-dollar"/>
          <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$s.generic"/>
	  </complexcat>
 </xsl:variable>   

 <xsl:variable name="mod-loose-vp.left">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$np.subj"/>
		  <xsl:copy-of select="$with-dollar"/>
		  <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$vp.generic"/>         
       </complexcat>
 </xsl:variable>   
 

 <xsl:variable name="mod-loose-vp.left.no-dollar"> 
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$np.subj"/>
		  <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$vp.generic"/>         
       </complexcat>
 </xsl:variable>   


<xsl:variable name="mod-loose-s.right">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <xsl:copy-of select="$with-dollar"/>
          <slash dir="/" mode="&gt;"/>
		  <xsl:copy-of select="$s.generic"/>
	  </complexcat>
 </xsl:variable>   

 <xsl:variable name="mod-loose-vp.right">
       <complexcat>
	      <xsl:copy-of select="$s.generic"/>
		  <slash dir="\" mode="&lt;"/>
		  <xsl:copy-of select="$np.subj"/>
		  <xsl:copy-of select="$with-dollar"/>
		  <slash dir="/" mode="&gt;"/>
		  <xsl:copy-of select="$vp.generic"/>         
      </complexcat>
 </xsl:variable>   


-->
