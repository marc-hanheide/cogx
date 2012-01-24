<?xml version="1.0"?>

<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">

  <xsl:import href="../core.v4.1/lexicon.xsl"/>  <!-- allows access to xsl:variables -->
  
<xsl:template name="add-unary-rules">

	<xsl:call-template name="add-prep-rules"/>
	<xsl:call-template name="add-adv-rules"/>
	<xsl:call-template name="add-adj-rules"/>

</xsl:template>
</xsl:transform>



<!-- adj[P] => n[T]/n[T] : @T<Mod>P 

this rule changes adjs from basic-cat to the complex n/n cat allowing 
combination with nouns. This is done to allow advs like "really" to take 
type adj/adj and hence get an index to the adj itself, but still then be
able to combine freely.
i.e., together with  
           really :- adj[M]/adj[M] : @P<Mod>D ^ @D(really)
      you could then derive
           really big table :- n[T] : @T(table ^ <Mod>(M ^ big ^ <Mod>(D ^ really))) 


<typechanging name="adj"> 
   <arg> 
    <complexcat>
      <xsl:copy-of select="$adj.basic-cat"/>
      <slash/>
      <dollar name="1"/>
	</complexcat>
   </arg> 
   <result> 
	  <complexcat>
		<xsl:copy-of select="$n.from-generic"/>
		<slash dir="/" mode="&gt;"/>
		<xsl:copy-of select="$n.generic"/>	
	    <slash/>
        <dollar name="1"/>
		<lf>
          <satop nomvar="T:entity">
	         <diamond mode="Property"><nomvar name="M"/></diamond> 
	       </satop>
         </lf> 	
	  </complexcat>
   </result> 
</typechanging>

-->


<!-- NOTE: This does NOT! allow the adj to restrict its modified noun! This is currently impossible
           in OpenCCG, except for the ugly solution of creating a syntactic! category modifier-type 
		   which the type-change rule selects for and then builds the corresponding semantic restrition 
		   
					ex.    <arg>  adj with mod-type=animate
						   <result>  @T:animate(...<property>M.)

		   The same holds for advs and prepositions
		   
	  NOTE: By not having basic cat adv & prepositions, we cannot
	  
			1) handle modification like "really quickly" or "over on the table"
			2) allow verbs to choose pp & adv in a nice way (see how copular is being handled)
				 this includes PUT which currently DOES NOT select for its pp-argument...			
-->


<!-- s.non-fin[E0]\np.closed[X] => np[T] : nominalized-event, also sets X (ie subj) to generic 

	 this rule nominalizes progr verbs
	 
-->


<!--!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

 NOT WORKING PROPERLY. CAN't get $ to stack over (S\NP)....
		i want S\NP $ ..., 
		note: can enumerate possibilites..but not very nice ..
		
	

<typechanging name="verb-nom"> 
   <arg> 
      <complexcat>
         <xsl:copy-of select="$s.generic.progr"/>
         <slash/>
         <dollar name="1"/>
         <slash dir="\" mode="&gt;"/>
         <xsl:copy-of select="$np.subj"/> 
      </complexcat>
   </arg> 

   <result> 
     <atomcat type="np">
          <fs id="1">
          <feat attr="num" val="sg"/>
          <feat attr="pers" val="3rd"/>
          <feat attr="index"><lf><nomvar name="T:entity"/></lf></feat>
		  <feat attr="nform" val="basic"/>
		  <feat attr="modifiable" val="-"/>
        </fs>  
   <lf>
      <satop nomvar="X:entity"><prop name="generic"/></satop> 
	  <satop nomvar="T:entity">
	     <prop name="nominalized-event"/>
	        <diamond mode="Event"><nomvar name="E0"/></diamond>
	     </satop>
   </lf>
   
	</atomcat>
   </result> 
</typechanging>

-->

<!-- s.non-fin[E0]\np[X] => s.fin[E0] : @E<Mood>imp 

this rule is used to turn non-fin vp into imp sentences


<typechanging name="imp"> 
   <arg>
     <complexcat>
         <xsl:copy-of select="$s.generic.non-fin"/>
         <slash/>
         <dollar name="1"/>
         <slash dir="\" mode="&gt;"/>
         <xsl:copy-of select="$np.subj"/> 
     </complexcat>
   </arg> 
   <result>
      <complexcat>
      <xsl:copy-of select="$s.from-generic.fin.imp"/>
	  <slash/>
	  <dollar name="1"/>
	      <lf>
		    <satop nomvar="X"><prop name="addressee"/></satop>      
            <satop nomvar="E0">
	           <diamond mode="Mood"><prop name="imp"/></diamond> 
            </satop>
         </lf>
      </complexcat>
   </result> 
</typechanging> -->

<!--	currently accepts to much .  Should only handle addressables like GJ, Robot, girls, etc.
	
<typechanging name="command">
    <arg>
      <xsl:copy-of select="$np.subj.addressable"/>
    </arg>
  
	<result>
      <complexcat>
        <xsl:copy-of select="$s.main.fronted"/>   
        <slash dir="/" mode="*"/>
			<complexcat>
				<xsl:copy-of select="$s.generic.non-fin"/>
		        <slash dir="\" mode="&gt;"/>
				<xsl:copy-of select="$np.subj"/> 
			</complexcat>
         <lf>
            <satop nomvar="E0">
	           <diamond mode="Mood"><prop name="imp"/></diamond> 
            </satop>
        </lf>
      </complexcat>	
    </result>
  </typechanging>  -->


<!-- To avoid un-necessary building, make this more like IMP above, ie build
	only when you have a np & an imp verb... 
	
	
<typechanging name="command">
    <arg>
      <xsl:copy-of select="$np.generic"/>
    </arg>
  
	<result>
      <complexcat>
        <xsl:copy-of select="$s.main.fronted"/>   
        <slash dir="/" mode="*"/>
     	<xsl:copy-of select="$s.generic.fin.imp"/>
	    <lf>
          <satop nomvar="E:relational">
		     <prop name="command"/>  
	         <diamond mode="Addressee"><nomvar name="T:animate"/></diamond>
	         <diamond mode="Scope"><nomvar name="E0"/></diamond>
          </satop>
        </lf>
      </complexcat>	
    </result>
  </typechanging> -->


<!-- np[T] => s/(vp | np[T])  : @T<Fronted> + 

This handles the extraction arbitrary NP-comp from the main clause, which
is then marked as fronted 


 <typechanging name="front">
    <arg>
      <xsl:copy-of select="$np.generic"/>
    </arg>
    <result>
	  <complexcat>
        <xsl:copy-of select="$s.from-main.fronted"/>
        <slash dir="/" mode="."/>
		<complexcat>
          <xsl:copy-of select="$s.main.fin"/>
          <slash dir="/" mode="&gt;"/>
		  <xsl:copy-of select="$np.generic"/> 
	    </complexcat>
		  <lf>
            <satop nomvar="T">
              <diamond mode="Fronted"><prop name="+"/></diamond>
            </satop>
         </lf>
	  </complexcat> 
    </result>
  </typechanging>

-->


 
<!-- np[T] => s.fronted[E]/s.fin[E0]  :  

This handles the fronting of any arbitrary np, followed by any finite sentence.
This is meant to handle presentational & topical left dislocations such as
	"This ball go and give IT to GJ"
	"This guy HE came over and said..."
	"That table, I put my things on IT yesterday"
	"The modern family, the mom and dad both work"
The NP is considered a restrictor taking scope over the sentence, but is not explicitly
co-indexed. This must be done as post processing. (note: in cases like the third there is no
direct co-indexation within the main clause. It is a more abstract topic relation)

	NOTE: currently dissallow imps, cuz must then figure out a way to block 
			I want a ball as being interpreted as a left-dis I and imp want



 <typechanging name="left-dis">
    <arg>
      <xsl:copy-of select="$np.generic"/>
    </arg>
  
	<result>
      <complexcat>
        <xsl:copy-of select="$s.main.fronted"/>   
        <slash dir="/" mode="*"/>
     	<xsl:copy-of select="$s.generic.fin.ind"/>
	    <lf>
          <satop nomvar="E:relational">
		     <prop name="left-dislocation"/>  
	         <diamond mode="Restr"><nomvar name="T"/></diamond>
	         <diamond mode="Scope"><nomvar name="E0"/></diamond>
          </satop>
        </lf>
      </complexcat>	
    </result>
  </typechanging> 

<typechanging name="right-dis">
    <arg>
      <xsl:copy-of select="$np.generic"/>
    </arg>
  	<result>
      <complexcat>
        <xsl:copy-of select="$s.main.fronted"/>   
        <slash dir="\" mode="*"/>
     	<xsl:copy-of select="$s.generic.fin.ind"/>
	    <lf>
          <satop nomvar="E:relational">
		     <prop name="right-dislocation"/>  
	         <diamond mode="Restr"><nomvar name="T"/></diamond>
	         <diamond mode="Scope"><nomvar name="E0"/></diamond>
          </satop>
        </lf>
      </complexcat>	
    </result>
  </typechanging> 

-->

 <!-- vp[EMA/T] => n[T]/n[T]  : @T<Restr> EMA 

  This handles reduced relative clauses like "The girl I met"
  
  CURRENTLY WORKS, BUT MUCH TO POWERFUL.

  <typechanging name="rrel">
    <arg>
      <complexcat>
        <xsl:copy-of select="$s.mod-arg.fin"/>
        <slash dir="/"/>
        <xsl:copy-of select="$np.generic"/>
      </complexcat>
    </arg>
    <result>
      <complexcat>
        <xsl:copy-of select="$n.from-generic"/>
        <slash dir="\" mode="*"/>
        <xsl:copy-of select="$n.from-generic"/>
        <xsl:copy-of select="$MOD-T.RESTR"/>
     </complexcat>
	</result>
  </typechanging>

-->


<!--
 <typechanging name="detached">
    <arg>
      <xsl:copy-of select="$np.subj"/>    
    </arg>
  
	<result>
    
	  <complexcat>
      
		<atomcat type="s">
          <fs inheritsFrom="11"/>
        </atomcat>
		
		<slash dir="/" mode="&gt;"/>
        <xsl:copy-of select="$vp.main.fin"/>
		<slash dir="/" mode="&gt;"/>
	    <xsl:copy-of select="$np.subj"/>        
		
		<lf>
          <satop nomvar="X">
            <diamond mode="Introduced"><prop name="+"/></diamond>
          </satop>
        </lf>
      </complexcat>
    </result>
  </typechanging> 
-->

<!--
   <typechanging name="topic">
    <arg>
      <atomcat type="np">
        <fs id="2">
          <feat attr="index"><lf><nomvar name="X"/></lf></feat>
        </fs>
      </atomcat>
    </arg>
    <result>
      <complexcat>
        <atomcat type="np"><fs inheritsFromI="2"/></atomcat>
        <slash dir="/" mode="&gt;"/>
        <atomcat type="np"><fs id="2"/></atomcat>
        <lf>
            <satop nomvar="X"><diamond mode="topic"><prop name="+"/></diamond></satop>
         </lf>
       </complexcat>
    </result>
  </typechanging>
-->

