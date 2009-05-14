<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">

  <!-- ================================================ -->  
  <!-- OUTPUT OPTIONS                                   -->
  <!-- ================================================ -->  

  <xsl:import href="../core.v4/macros.xsl"/>
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
 
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- COORDINATION										-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <entry pred="and" stem="and__coord" pos="COORD">	
     <word form="and"/>
	 <word form=","/>
	 <word form="then"/>
	 <member-of family="coord.np.pl-result"/>
 	 <member-of family="coord.n.pl-result"/>
	 <member-of family="coord.adj"/>	  
   </entry>

  <entry pred="or" stem="or__coord" pos="COORD">	
     <word form="or"/>
	 <member-of family="coord.np"/>
 	 <member-of family="coord.n"/>
	 <member-of family="coord.adj"/>	  
   </entry>

<!-- sentence ones set as type relational to block modifiers & reduce # parses... is this correct? -->
  <entry pred="or" stem="or__coord" pos="COORD" class="relational">	
     <word form="or"/>
 	 <member-of family="coord.s"/>
   </entry>

  <entry pred= "and" stem="then__coord" pos="COORD" class="relational">	
	 <word form="and"/>
	 <word form=","/>
	 <word form="then"/>
	 <word form="but"/> 
   	<member-of family="coord.s"/>
  </entry>



<!--
   <entry pred="and" stem="and__coord" pos="COORD" class="relational">	
	<word form="and"/>
	<word form=","/>
	<member-of family="COORD.conj.pl"/> 
    <member-of family="COORD.conj.incl.pl"/>
	<member-of family="COORD.s"/>
	<member-of family="COORD.n"/>
	<member-of family="COORD.conj.pl.leftw.ext"/>  
	<member-of family="COORD.conj.pl.incl.leftw.ext"/>
  </entry>

 <entry pred="or" stem="or__coord" pos="COORD" class="relational">	
	<word form="or"/>  
	<member-of family="COORD.conj.pl"/> 
    <member-of family="COORD.conj.incl.pl"/>
	<member-of family="COORD.s"/>
	<member-of family="COORD.n"/>
	<member-of family="COORD.conj.pl.leftw.ext"/>  
	<member-of family="COORD.conj.pl.incl.leftw.ext"/>
  </entry>

  <entry pred= "then" stem="then__coord" pos="COORD">	
    <word form="then"/> 
   	<member-of family="COORD.s"/>
  </entry>
-->
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- DETERMINERS                   					-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <entry pred="a" stem="a__det" pos="DET">
    <word form="a"/>
	<word form="an"/>
	<member-of family="determiner.indefinite"/>
  </entry>

<!-- semantics?? -->
 <entry pred="another" stem="another__det" pos="DET">
    <word form="another"/>
	<member-of family="determiner.indefinite"/>
  </entry>

  <entry pred="the" stem="the__det" pos="DET">
      <word form="the"/>
    <member-of family="determiner.definite"/>
  </entry>

  <entry pred="these" stem="these__det" pos="DET">
    <word form="these" macros="@proximity.proximal"/>
	<member-of family="determiner.demonstrative.pl"/>
  </entry>

  <entry pred="this" stem="this__det" pos="DET">
    <word form="this" macros="@proximity.proximal"/>
	<member-of family="determiner.demonstrative.sg"/>
  </entry>

  <entry pred="those" stem="those__det" pos="DET">
    <word form="those" macros="@proximity.distal"/>
	<member-of family="determiner.demonstrative.pl"/>
  </entry>
  
  <entry pred="that" stem="that__det" pos="DET">
    <word form="that" macros="@proximity.distal"/>
	<member-of family="determiner.demonstrative.sg"/>
  </entry>
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- QUANTITY WORDS                   			    -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->   

  <entry pred="all" stem="all__det"  pos="DET">
    <word form="all"/>  
	<member-of family="determiner.quantity"/>
	<member-of family="determiner-np.quantity"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 
  
  <entry pred="some" stem="some__det"  pos="DET">
    <word form="some"/>
	<member-of family="determiner.quantity"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 

  <entry pred="none" stem="none__det"  pos="DET">
    <word form="none"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 
  
  <entry pred="none" stem="no__det"  pos="DET">
    <word form="no"/>  
	<member-of family="determiner.quantity"/>
  </entry> 
  
  <entry pred="any" stem="any__det" pos="DET">
    <word form="any"/>  
	<member-of family="determiner.quantity"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 

 <entry pred="many" stem="many__det"  pos="DET">
    <word form="many"/>
	<word form="much"/>
	<member-of family="determiner.quantity"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 

 <entry pred="more" stem="more__det"  pos="DET">
    <word form="more"/>
	<member-of family="determiner.quantity"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 

 <entry pred="less" stem="less__det"  pos="DET">
    <word form="less"/>
	<member-of family="determiner.quantity"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 

  <entry pred="a_lot" stem="a_lot__det" pos="DET">
	<word form="a" macros="@mwe1-lot"/>
	<word form="a" macros="@mwe1-bunch"/>
	<word form="a" macros="@mwe1-load"/>
	<member-of family="determiner-of-np.quantity_W"/>
  </entry> 
  <entry pred="a_lot" stem="lots__det" pos="DET">
	<word form="lots"/>
	<word form="bunches"/>
	<word form="loads"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry>
  
  <entry pred="a_few" stem="a_few__det" pos="DET">
	<word form="a" macros="@mwe1-few"/>
	<member-of family="determiner.quantity_W"/>
	<member-of family="determiner-of-np.quantity_W"/>
  </entry> 
  
  <entry pred="a_couple" stem="a_couple__det" pos="DET">
	<word form="a" macros="@mwe1-couple"/>
	<member-of family="determiner.quantity_W"/>
	<member-of family="determiner-of-np.quantity_W"/>
  </entry> 
  
  <!-- Numbers (ordinal done as adj, found in dict.list) -->
 
 
    <entry pred="one" stem="one__det" pos="DET">
    <word form="one"/>  
	<member-of family="determiner.quantity.sg"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 
  <entry pred="two" stem="two__det" pos="DET">
    <word form="two"/>  
	<member-of family="determiner.quantity"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 
  <entry pred="three" stem="three__det" pos="DET">
    <word form="three"/>  
	<member-of family="determiner.quantity"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 
  <entry pred="four" stem="four__det" pos="DET">
    <word form="four"/>  
	<member-of family="determiner.quantity"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 
  <entry pred="five" stem="five__det" pos="DET">
    <word form="five"/>  
	<member-of family="determiner.quantity"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 
   <entry pred="six" stem="six__det" pos="DET">
    <word form="six"/>  
	<member-of family="determiner.quantity"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 
  <entry pred="seven" stem="seven__det" pos="DET">
    <word form="seven"/>  
	<member-of family="determiner.quantity"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 
   <entry pred="eight" stem="eight__det" pos="DET">
    <word form="eight"/>  
	<member-of family="determiner.quantity"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 
   <entry pred="nine" stem="nine__det" pos="DET">
    <word form="nine"/>  
	<member-of family="determiner.quantity"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 
   <entry pred="ten" stem="ten__det" pos="DET">
    <word form="ten"/>  
	<member-of family="determiner.quantity"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 
   <entry pred="eleven" stem="eleven__det" pos="DET">
    <word form="eleven"/>  
	<member-of family="determiner.quantity"/>
	<member-of family="determiner-of-np.quantity"/>
  </entry> 
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- PRONOUN: RELATIVE                      			-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

   <entry pred="that" stem="that__relpro" pos="MARKER">
    <word form="that"/>  
    <word form="who"/>
	<word form="whom"/>
	<word form="which"/>  
	<member-of family="rel-pronoun"/>
  </entry>
   
  <entry pred="to" stem="to__n-mod-purpose" pos="MARKER">
    <word form="to"/>  
	<member-of family="n-mod.purpose.non-fin-verb"/>
  </entry>
  
  
  
  <entry pred="for" stem="for__n-mod-purpose" pos="MARKER">
    <word form="for"/>  
	<member-of family="n-mod.purpose.progr-verb"/>
  </entry>
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- PRONOUNS: POSSESSIVE                   			-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <entry stem="my" pred="I" pos="DET" class="person">
	<member-of family="determiner.possessive.sg-owner"/>
  </entry>
  <entry stem="our" pred="I" pos="DET" class="person">
	<member-of family="determiner.possessive.pl-owner"/>
  </entry>

  <entry stem="your" pred="you" pos="DET" class="person" >
	<member-of family="determiner.possessive.sg-owner"/>
  </entry>
  <entry stem="your" pred="you" pos="DET" class="person">
	<member-of family="determiner.possessive.pl-owner"/>
  </entry>

  <entry stem="his" pred="he" pos="DET" class="person">
	<member-of family="determiner.possessive.sg-owner"/>
  </entry>
  <entry stem="her" pred="she" pos="DET" class="person">
	<member-of family="determiner.possessive.sg-owner"/>
  </entry>
  <entry stem="its" pred="it" pos="DET" class="thing">
	<member-of family="determiner.possessive.sg-owner"/>
  </entry>
  <entry stem="their" pred="they" pos="DET" class="entity">
	<member-of family="determiner.possessive.pl-owner"/>
  </entry>
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- POSSESSIVE of and 's                    			-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <!-- Must be "book of John 'S", without it screws everything up (like to the left of)
  
	<entry pred="owner" stem="of__owner"  pos="PREP" class="m-owner">
    <word form="of"/>
	<member-of family="prep-mod-n.owner"/>
  </entry> -->

  <entry pred="owner" stem="'s__owner"  pos="MARKER" class="m-owner">
    <word form="'s"/>
	<member-of family="possessive-'s"/>
  </entry>

  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- PRONOUNS: PERSONAL                   			-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <entry stem="I" pred="I" pos="N" class="person">
	<word form="I" macros="@pers.1st"/>
	<word form="me" macros="@pers.1st"/>
	<member-of family="personal-pronoun.sg"/>
  </entry>
    <entry stem="we" pos="N" class="person">
	<word form="we" macros="@pers.1st" />
	<word form="us" macros="@pers.1st"/>
	<member-of family="personal-pronoun.pl"/>
  </entry>

  <entry stem="you" pred="you" pos="N" class="person">
	<word form="you" macros="@pers.2nd"/>
	<member-of family="personal-pronoun.sg"/>	
  </entry>
  <entry stem="you-pl" pred="you" pos="N" class="person">
	<word form="you" macros="@pers.2nd"/>
	<member-of family="personal-pronoun.pl"/>	
  </entry>

  <entry stem="he" pred="he" pos="N" class="person">
	<word form="he" macros="@pers.3rd"/>
	<word form="him" macros="@pers.3rd"/>
	<member-of family="personal-pronoun.sg"/>
  </entry>
  <entry stem="she" pred="she" pos="N" class="person">
	<word form="she" macros="@pers.3rd"/>
	<word form="her" macros="@pers.3rd"/>
	<member-of family="personal-pronoun.sg"/>
  </entry>
  <entry stem="it" pred="it" pos="N" class="thing">
	<word form="it" macros="@pers.3rd"/>
	<member-of family="personal-pronoun.sg"/>
  </entry>
  <entry stem="they" pred="they" pos="N" class="entity">
	<word form="they" macros="@pers.3rd"/>
	<word form="them" macros="@pers.3rd"/>
	<member-of family="personal-pronoun.pl"/>
  </entry>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- PRONOUNS: POSSESSED                   			-->
  <!-- NOTE: Here, the possessed entity itself is not marked for number "mine are/is big" -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <entry stem="mine" pred="I" pos="N" class="person">
	<word form="mine"/>
	<member-of family="possessed-pronoun.sg"/>
  </entry>
  <entry stem="ours" pred="we" pos="N" class="person">
	<word form="ours"/>
	<member-of family="possessed-pronoun.pl"/>
  </entry>

  <entry stem="yours" pred="you" pos="N" class="person">
	<word form="yours"/>
	<member-of family="possessed-pronoun.sg"/>	
  </entry>
  <entry stem="yours-pl" pred="you" pos="N" class="person">
	<word form="yours"/>
	<member-of family="possessed-pronoun.pl"/>	
  </entry>

  <entry stem="his-possessed" pred="he" pos="N" class="person">
	<word form="his"/>
	<member-of family="possessed-pronoun.sg"/>
  </entry>
  <entry stem="hers" pred="she" pos="N" class="person">
	<word form="hers"/>
	<member-of family="possessed-pronoun.sg"/>
  </entry>
  <entry stem="its-possessed" pred="it" pos="N" class="thing">
	<word form="its"/>
	<member-of family="possessed-pronoun.sg"/>
  </entry>
  <entry stem="theirs" pred="they" pos="N" class="entity">
	<word form="theirs"/>
	<member-of family="possessed-pronoun.pl"/>
  </entry>


  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- QUESTION WORD NP									-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <entry stem="what-obj" pos="N" class="thing">
	<word form="what"/>
 	<member-of family="question-word.obj"/>
  </entry>

  <entry stem="what-event" pos="N" class="event">
	<word form="what"/>
 	<member-of family="question-word.scomp"/>
  </entry>

  <entry stem="which-obj" pos="N" class="thing">
	<word form="which"/>
 	<member-of family="question-word.obj"/>
  </entry>

  <entry stem="who-obj" pos="N" class="person">
	<word form="who"/>
	<member-of family="question-word.obj"/>
  </entry>
    
  <entry stem="who-subj" pos="N" class="person">
	<word form="who" macros="@nform-who-subj"/>
 	<member-of family="bare-np"/>
  </entry>
  
  <entry stem="where-location" pos="N" class="region">
	<word form="where"/>
 	<member-of family="question-word.mod"/>
  </entry>
    
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- PRONOUNS: DEICTIC                				-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  
  <entry pred="this" stem="this__deictic" pos="N" class="thing">
	<word form="this" macros="@proximity.proximal @sem-num.sg @del.unique @quant.sg "/>
 	<member-of family="deictic-pronoun"/>
  </entry>
  
  <entry pred="that" stem="that__deictic" pos="N" class="thing">
	<word form="that" macros="@proximity.distal @sem-num.sg @del.unique @quant.sg"/>	
 	<member-of family="deictic-pronoun"/>
  </entry>

  <entry pred="these" stem="these__deictic" pos="N" class="thing">
	<word form="these" macros="@proximity.proximal @sem-num.pl @del.unique @quant.nonsg"/>
 	<member-of family="deictic-pronoun"/>
  </entry>

  <entry pred="those" stem="those__deictic" pos="N" class="thing">
	<word form="those" macros="@proximity.distal @sem-num.pl @del.unique @quant.nonsg"/>
 	<member-of family="deictic-pronoun"/>
  </entry>
  
   <entry pred="one" stem="one-sg__deictic" pos="N" class="thing">
   	<word form="one" macros="@num.sg-second"/>
	<member-of family="one-pronoun.n"/>
    <member-of family="one-pronoun.np"/>
  </entry>
    
  <entry pred="one" stem="one-pl__deictic" pos="N" class="thing">
  	<word form="ones" macros="@num.pl-second"/>
	<member-of family="one-pronoun.n"/>
	<member-of family="one-pronoun.np"/>
  </entry>
  
   <entry pred="this" stem="this__pro-sent" pos="S" class="event">
	<word form="this"/>
	<word form="that"/>
	<word form="it"/>
	<member-of family="pro-sentence"/>
  </entry>
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- MORE PRONOUNS									-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  
  <!-- need to sort out DIAMOND values for these... -->
  <entry stem="something" pos="N" class="thing">
   	<word form="anything" macros="@num.sg @del.exist @quant.sg"/>
	<word form="something" macros="@num.sg @del.exist @quant.sg"/>
	<member-of family="bare-np.modifiable"/>
  </entry>
 
  <entry stem="someone" pos="N" class="person">
   	<word form="anyone" macros="@num.sg @del.exist @quant.sg"/>
	<word form="someone" macros="@num.sg @del.exist @quant.sg"/>
	<member-of family="bare-np.modifiable"/>
  </entry>
  
  <entry stem="everyone" pos="N" class="person">
   	<word form="everyone"/>
	<member-of family="bare-np.modifiable"/>
  </entry>
  <entry stem="everything" pos="N" class="thing">
   	<word form="everything"/>
	<member-of family="bare-np.modifiable"/>
  </entry>


 <!--  Handled in Copular now via preposition
  <entry pred="here" stem="here-barenp" pos="N" class="thing">
	<word form="here"/>
  	<member-of family="bare-np"/>
  </entry>
  

  <entry pred="there" stem="there__deictic" pos="N" class="thing">
	<word form="there" macros="@proximity.distal @sem-num.sg @del.unique @quant.sg"/>	
 	<member-of family="deictic-pronoun"/>
  </entry>
  -->
  
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- SOME MWE/COMPOUND NOUNS							-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  
  <entry pred="mobile_phone" stem="mobile_phone_W-sg" pos="N" class="thing">
    <word form="mobile" macros="@mwe1-phone"/>
    <word form="cell" macros="@mwe1-phone"/>
    <member-of family="noun_W"/>
  </entry>
  <entry pred="mobile_phone" stem="mobile_phone_W-pl" pos="N" class="thing">
    <word form="mobile" macros="@mwe1-phones"/>
    <word form="cell" macros="@mwe1-phones"/>
    <member-of family="noun.plural_W"/>
  </entry>
  <entry pred="mobile_phone" stem="mobile_phone-sg" pos="N" class="thing">
    <word form="mobile"/>
    <word form="cell"/>
	<word form="phone"/>
	<word form="handy"/>
    <member-of family="noun"/>
  </entry>
  <entry pred="mobile_phone" stem="mobile_phone-pl" pos="N" class="thing">
    <word form="mobiles"/>
    <word form="cells"/>
	<word form="phones"/>
	<word form="handys"/>
    <member-of family="noun.plural"/>
  </entry>

 <entry pred="charging_station" stem="charging_station_W-sg" pos="N" class="e-location">
    <word form="charging" macros="@mwe1-station"/>
    <word form="charging" macros="@mwe1-point"/>
    <word form="battery-charging" macros="@mwe1-station"/>
    <word form="battery-charging" macros="@mwe1-point"/>
    <member-of family="noun_W"/>
  </entry>
  <entry pred="charging_station" stem="charger-sg" pos="N" class="e-location">
    <word form="charger"/>
    <member-of family="noun"/>
  </entry>
  <entry pred="charging_station" stem="charging_station_W-pl" pos="N" class="e-location">
    <word form="charging" macros="@mwe1-stations"/>
    <word form="charging" macros="@mwe1-points"/>
    <word form="battery-charging" macros="@mwe1-stations"/>
    <word form="battery-charging" macros="@mwe1-points"/>
    <member-of family="noun.plural_W"/>
  </entry>
  <entry pred="charging_station" stem="charger-pl" pos="N" class="e-location">
    <word form="chargers"/>
    <member-of family="noun.plural"/>
  </entry>
 
  <entry pred="tea_kettle" stem="tea_kettle_W-sg" pos="N" class="thing">
    <word form="tea" macros="@mwe1-kettle"/>
    <member-of family="noun_W"/>
  </entry>
  <entry pred="tea_kettle" stem="tea_kettle_W-pl" pos="N" class="thing">
    <word form="tea" macros="@mwe1-kettles"/>
    <member-of family="noun.plural_W"/>
  </entry>
  <entry pred="tea_kettle" stem="tea_kettle-sg" pos="N" class="thing">
    <word form="kettle"/>
    <member-of family="noun"/>
  </entry>
  <entry pred="tea_kettle" stem="tea_kettle-pl" pos="N" class="thing">
    <word form="kettles"/>
    <member-of family="noun.plural"/>
  </entry>
  
  <entry pred="coffee_machine" stem="coffee_machine_W-sg" pos="N" class="thing">
    <word form="coffee" macros="@mwe1-machine"/>
    <word form="coffee" macros="@mwe1-maker"/>
    <member-of family="noun_W"/>
  </entry>
  <entry pred="coffee_machine" stem="coffee_machine_W-pl" pos="N" class="thing">
    <word form="coffee" macros="@mwe1-machines"/>
    <word form="coffee" macros="@mwe1-makers"/>
    <member-of family="noun.plural_W"/>
  </entry>
  
  <entry pred="fax_machine" stem="fax_machine_W-sg" pos="N" class="thing">
    <word form="fax" macros="@mwe1-machine"/>
    <member-of family="noun_W"/>
  </entry>
  <entry pred="fax_machine" stem="fax_machine_W-pl" pos="N" class="thing">
    <word form="fax" macros="@mwe1-machines"/>
    <member-of family="noun.plural_W"/>
  </entry>

  
  <entry pred="living_room" stem="living_room_W-sg" pos="N" class="e-location">
    <word form="living" macros="@mwe1-room"/>
    <member-of family="noun_W"/>
  </entry>
  <entry pred="living_room" stem="living_room_W-pl" pos="N" class="e-location">
    <word form="living" macros="@mwe1-rooms"/>
    <member-of family="noun.plural_W"/>
  </entry>

  <entry pred="dining_room" stem="dining_room_W-sg" pos="N" class="e-location">
    <word form="dining" macros="@mwe1-room"/>
    <member-of family="noun_W"/>
  </entry>
  <entry pred="dining_room" stem="dining_room_W-pl" pos="N" class="e-location">
    <word form="dining" macros="@mwe1-rooms"/>
    <member-of family="noun.plural_W"/>
  </entry>

 
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- ROOMS & IDS										-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  
  <entry pred="room" stem="room_id" pos="N" class="e-location">
	<word form="room" macros="@num.sg @pers.3rd"/>
	<word form="Room" macros="@num.sg @pers.3rd"/>
	<member-of family="compound-id-head"/>
    <member-of family="compound-head-number"/>
  </entry>

  <entry pred="leibniz" stem="leibniz_id" pos="N" class="loc-name">
	<word form="Leibniz" macros="@pers.3rd" />
	<member-of family="bare-np"/>
  </entry>
  <entry pred="turing" stem="turing_id" pos="N" class="loc-name">
	<word form="Turing" macros="@pers.3rd"/>
	<member-of family="bare-np"/>
  </entry>


  <entry stem="[*NUM*]" pos="NUM" class="number">
	<member-of family="number"/>
  </entry>
  <entry stem="100" pos="NUM" class="number">
	<member-of family="number"/>
  </entry>
  <entry stem="200" pos="NUM" class="number">
	<member-of family="number"/>
  </entry>
 
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- MEASURE WORDS    								-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  
   <entry pred="pile" stem="pile__measure" pos="N">
	<word form="pile"/>
	<member-of family="measure-word"/>
  </entry>
   <entry pred="few" stem="few__measure" pos="N">
	<word form="few"/>
	<member-of family="measure-word"/>
  </entry> 
  <entry pred="group" stem="group__measure" pos="N">
	<word form="group"/>  
	<member-of family="measure-word"/>
  </entry>
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- OBLIQUES      									-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <entry stem="oblique-of" pos="PREP">
    <word form="of" macros="@pform-of"/>
    <member-of family="prep-oblique"/>
  </entry>
  <entry stem="oblique-at" pos="PREP">
    <word form="at" macros="@pform-at"/>
	<member-of family="prep-oblique"/>
  </entry>
  <entry stem="oblique-to" pos="PREP">
    <word form="to" macros="@pform-to"/>
	<member-of family="prep-oblique"/>
  </entry>
  <entry stem="oblique-for" pos="PREP">
    <word form="for" macros="@pform-for"/>
	<member-of family="prep-oblique"/>
  </entry>

  <entry stem="oblique-with" pos="PREP">
    <word form="with" macros="@pform-with"/>
	<member-of family="prep-oblique"/>
  </entry>
  <entry stem="oblique-about" pos="PREP">
    <word form="about" macros="@pform-about"/>
	<member-of family="prep-oblique"/>
  </entry>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- PARTICLES (not really be handled properly yet	-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <entry pred="up" stem="up__prt" pos="PRT">
    <word form="up"/>
	<member-of family="particle"/>
  </entry>	


  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- COMPARATIVE										-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <entry pred="than" stem="than__comparative-adj" pos="MARKER">
    <word form="than"/>
	<member-of family="comparative-adj-than"/>
  </entry>	

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- SERIAL VERB CONSTRCUTIONS						-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  


<!-- This meant to handle the rare-type, but frequent-token, serial constructions like
		"go get me my book"  &	"come and sit over here" & "run pick that up for me"-->
  <entry pred="serial-come" stem="come__serial-verb" pos="V" class="relational">
   <word form="come" macros="@finite @pres @imp "/>
   <word form="come" macros="@finite @pres @ind @num.pl-agr"/>
   <word form="come" macros="@nonfin"/>
   <member-of family="v.subj-controlled-non-fin-verb-comp" />
  </entry>	
  <entry pred="serial-come" stem="come__serial-verb_NO-SUBJ" pos="V" class="relational">
   <word form="come" macros="@finite @pres @imp"/>
   <member-of family="v.subj-controlled-non-fin-verb-comp.NO-SUBJ" />
  </entry>	

  <entry pred="serial-go" stem="go__serial-verb" pos="V" class="relational">
    <word form="go" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr"/>
    <word form="go" macros="@finite @pres @ind @num.pl-agr"/>
    <word form="go" macros="@nonfin"/>
	<member-of family="v.subj-controlled-non-fin-verb-comp"/>
  </entry>
  <entry pred="serial-go" stem="go__serial-verb_NO-SUBJ" pos="V" class="relational">
   <word form="go" macros="@finite @pres @imp"/>
   <member-of family="v.subj-controlled-non-fin-verb-comp.NO-SUBJ" />
  </entry>	
  
   <entry pred="serial-run" stem="run__serial-verb" pos="V" class="relational">
    <word form="run" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr"/>
    <word form="run" macros="@finite @pres @ind @num.pl-agr"/>
    <word form="run" macros="@nonfin"/>
	<member-of family="v.subj-controlled-non-fin-verb-comp"/>
  </entry>
  <entry pred="serial-run" stem="run__serial-verb_NO-SUBJ" pos="V" class="relational">
   <word form="run" macros="@finite @pres @imp"/>
   <member-of family="v.subj-controlled-non-fin-verb-comp.NO-SUBJ" />
  </entry>	
	
  
  <entry pred="serial-come" stem="come__serial-verb_W" pos="V" class="relational">
   <word form="come" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr @mwe1-and"/>
   <word form="come" macros="@finite @pres @ind @num.pl-agr @mwe1-and"/>
   <word form="come" macros="@nonfin @mwe1-and"/>
	<member-of family="v.subj-controlled-non-fin-verb-comp_W"/>
  </entry>	
  <entry pred="serial-go" stem="go__serial-verb_W" pos="V" class="relational">
    <word form="go" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr  @mwe1-and"/>
    <word form="go" macros="@finite @pres @ind @num.pl-agr  @mwe1-and"/>
    <word form="go" macros="@nonfin"/>
	<member-of family="v.subj-controlled-non-fin-verb-comp_W"/>
  </entry>
  <entry pred="serial-run" stem="run__serial-verb_W" pos="V" class="relational">
    <word form="run" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr  @mwe1-and"/>
    <word form="run" macros="@finite @pres @ind @num.pl-agr  @mwe1-and"/>
    <word form="run" macros="@nonfin  @mwe1-and"/>
	<member-of family="v.subj-controlled-non-fin-verb-comp_W"/>
  </entry>
<!-- OLD
 <entry pred="come" stem="come__serial-verb" pos="V">
    <word form="come"/>
	<member-of family="serial-verb"/>
  </entry>	
  <entry pred="go" stem="go__serial-verb" pos="V">
    <word form="go"/>
	<member-of family="serial-verb"/>
  </entry>
  
  <entry pred="come" stem="come__serial-verb" pos="V">
    <word form="come"  macros="@mwe1-and"/>
	<member-of family="serial-verb_W"/>
  </entry>	
  <entry pred="go" stem="go__serial-verb" pos="V">
    <word form="go" macros="@mwe1-and" />
	<member-of family="serial-verb_W"/>
  </entry>
  
   <entry pred="run" stem="run__serial-verb" pos="V">
    <word form="run"/>
	<member-of family="serial-verb"/>
  </entry>	
  <entry pred="go" stem="go__serial-verb" pos="V">
    <word form="go"/>
	<member-of family="serial-verb"/>
  </entry>
-->

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- VERBS: COPULAR    						        -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

  <xsl:variable name="be">
    <word form="be" macros="@nonfin"/>
	<word form="being" macros="@vform.progr"/>
   	<word form="is" macros="@finite @pres @ind   @pers.3rd-agr @num.sg-agr"/>
	<word form="are" macros="@finite @pres @ind  @pers.2nd-agr @num.sg-agr"/>
	<word form="are" macros="@finite @pres @ind                @num.pl-agr"/>
	<word form="am" macros="@finite @pres @ind   @pers.1st-agr @num.sg-agr"/>
	<word form="was" macros="@finite @past @ind  @pers.1st-agr @num.sg-agr"/>
	<word form="was" macros="@finite @past @ind  @pers.3rd-agr @num.sg-agr"/>
	<word form="were" macros="@finite @past @ind @pers.2nd-agr"/>
	<word form="were" macros="@finite @past @ind               @num.pl-agr"/>
  </xsl:variable>
 
  <xsl:variable name="be.int">   
	<word form="is" macros="@finite @pres @int   @pers.3rd-agr @num.sg-agr"/>
	<word form="are" macros="@finite @pres @int  @pers.2nd-agr @num.sg-agr"/>
	<word form="are" macros="@finite @pres @int                @num.pl-agr"/>
	<word form="am" macros="@finite @pres @int   @pers.1st-agr @num.sg-agr"/>
	<word form="was" macros="@finite @past @int  @pers.1st-agr @num.sg-agr"/>
	<word form="was" macros="@finite @past @int  @pers.3rd-agr @num.sg-agr"/>
	<word form="were" macros="@finite @past @int @pers.2nd-agr"/>
	<word form="were" macros="@finite @past @int               @num.pl-agr"/>
  </xsl:variable>
  
 <entry pred="be" stem="be-ind__ascription"  pos="V" class="ascription">
     <xsl:copy-of select="$be"/>
     <member-of family="copular"/>
 </entry>	
 
<entry pred="be" stem="be-NON-SUBJ__ascription" pos="V" class="ascription">
	<word form="be" macros="@finite @imp"/>
	<member-of family="copular.NO-SUBJ"/>
 </entry>

 <entry pred ="be" stem="be-int__ascription" pos="V" class="ascription">
    <xsl:copy-of select="$be.int"/>
	<member-of family="copular.inverted"/>
	<member-of family="copular.fronted-question"/>
 </entry>	 

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- VERBS: PRO-VERB/ELLIPTICAL DO                    -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

<entry pred="pro-verb-do" stem="do__pro-form" pos="V">
   <word form="do" macros="@nonfin"/>
   <word form="doing" macros="@vform.progr"/>
   <word form="do" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr"/>
   <word form="do" macros="@finite @pres @ind @num.pl-agr"/>
   <word form="does" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
   <word form="did" macros="@finite @past @ind" />
   <member-of family="tv"/>
</entry>  


  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- VERBS: AUXILLARY & MODAL  				        -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  
  <xsl:variable name="AUX-FAMILES">
   <member-of family="aux"/>
   <member-of family="aux.polar-question"/>
   <member-of family="aux.fronted-obj-question"/>
   <member-of family="aux.fronted-scomp-question"/> 
   <member-of family="aux.fronted-location-question"/>
   <member-of family="aux.fronted-whereto-question"/>
  </xsl:variable>

  
  
  <!-- handle AT LEAST the modaly verbs via dict.list.... -->
  
 <entry pred="do" stem="do__aux" pos="V" class="modal">
   <word form="do" macros="@finite @pres @pers.non-3rd-agr @num.sg-agr"/>
   <word form="do" macros="@finite @pres @num.pl-agr"/>
   <word form="does" macros="@finite @pres @pers.3rd-agr @num.sg-agr"/>
   <word form="did" macros="@finite @past" />
    <xsl:copy-of select="$AUX-FAMILES"/>
 </entry>  

 <entry pred="will" stem="will__aux" pos="V" class="modal">
   <word form="will" macros="@finite @future"/>
    <xsl:copy-of select="$AUX-FAMILES"/>
 </entry>

 <entry pred="would" stem="would__aux" pos="V" class="modal">
   <word form="would" macros="@finite @future"/>
     <xsl:copy-of select="$AUX-FAMILES"/>
 </entry>

<entry pred="should" stem="should__modal" pos="V" class="modal">
   <word form="should" macros="@finite @pres"/>
   <xsl:copy-of select="$AUX-FAMILES"/>
</entry>

<entry pred="shall" stem="shall__modal" pos="V" class="modal">
   <word form="shall" macros="@finite @pres"/>
   <xsl:copy-of select="$AUX-FAMILES"/>
</entry>
 
 <entry pred="may" stem="may__modal" pos="V" class="modal">
   <word form="may" macros="@finite @pres"/>
   <xsl:copy-of select="$AUX-FAMILES"/>
</entry>
 
  <entry pred="might" stem="might__modal" pos="V" class="modal">
   <word form="might" macros="@finite @pres"/>
   <xsl:copy-of select="$AUX-FAMILES"/>
</entry>
	
 <entry pred="can" stem="can__modal" pos="V" class="modal">
	<word form="can" macros="@finite @pres"/>
    <word form="could" macros="@finite @past"/>
    <xsl:copy-of select="$AUX-FAMILES"/>
</entry>

<!--for progressive
<entry pred="be" stem="be__aux" pos="V" class="modal">
	<word form="is" macros="@finite @pres @progr   @pers.3rd-agr @num.sg-agr"/>
	<word form="are" macros="@finite @pres @progr  @pers.2nd-agr @num.sg-agr"/>
	<word form="are" macros="@finite @pres @progr                @num.pl-agr"/>
	<word form="am" macros="@finite @pres @progr   @pers.1st-agr @num.sg-agr"/>
	<word form="was" macros="@finite @past @progr  @pers.1st-agr @num.sg-agr"/>
	<word form="was" macros="@finite @past @progr  @pers.3rd-agr @num.sg-agr"/>
	<word form="were" macros="@finite @past @progr @pers.2nd-agr"/>
	<word form="were" macros="@finite @past @progr               @num.pl-agr"/>
    <member-of family="aux.progr"/>
	<member-of family="aux.progr.polar-question"/>
	<member-of family="aux.progr.fronted-obj-question"/>
    <member-of family="aux.progr.fronted-scomp-question"/>
    <member-of family="aux.progr.fronted-location-question"/>
    <member-of family="aux.progr.fronted-whereto-question"/>
</entry> -->

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- VERBS: EXISTENTIAL 						        -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

<!-- these COULD be handled as MWE. i.e. they choose a word
	of wform= dummy-subj -->

 <entry pred="be" stem="be__existence-ind" pos="V" class="existence">
     <xsl:copy-of select="$be"/>
     <member-of family="existence.dummy-deic-subject"/>
  </entry>	
 <entry pred="be" stem="be__existence-int" pos="V" class="existence">
     <xsl:copy-of select="$be.int"/>
     <member-of family="existence.dummy-deic-subject.inverted"/>
  </entry>	
  
 <entry pred="have" stem="have__existence" pos="V" class="existence">
   <word form="have" macros="@finite @pres @ind"/>
   <member-of family="existence.dummy-pers-subject"/>
 </entry>   		

<entry pred="got" stem="got__existence" pos="V" class="existence">
   <word form="got" macros="@finite @pres @ind"/>
   <member-of family="existence.dummy-pers-subject"/>
 </entry>   	

 <entry stem="dummy-existential-subject" pos="N" class="entity">
	<word form="here" macros= "@dummy-deic"/>
	<word form="there" macros= "@dummy-deic"/>
	<!--<word form="there"  macros="@syn-num.sg @pers.3rd @dummy-deic"/>
	<word form="there"  macros="@syn-num.pl @pers.3rd @dummy-deic"/>
	<word form="here"  macros="@syn-num.sg @pers.3rd @dummy-deic"/>
	<word form="here"  macros="@syn-num.pl @pers.3rd @dummy-deic"/>-->
	<word form="you" macros= "@dummy-pers"/>
    <word form="we" macros= "@dummy-pers" />
	<member-of family="dummy-np"/>
  </entry>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- VERBS: MULTI-WORD     							-->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  


<!-- take_a_LOOK at the ball -->
<entry pred="look" stem="take_a_look__perception" pos="V" class="perception">
   <word form="take" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr @mwe1-a @mwe2-look"/>
   <word form="take" macros="@finite @pres @ind @num.pl-agr @mwe1-a @mwe2-look"/>
   <word form="takes" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr @mwe1-a @mwe2-look"/>
   <word form="took" macros="@finite @past @ind @mwe1-a @mwe2-look"/>
   <word form="take" macros="@nonfin @mwe1-a @mwe2-look"/>
   <word form="taking" macros="@vform.progr @pres @ind @mwe1-a @mwe2-look"/>  
   <member-of family="tv.at-obj_W_W"/>
</entry>  

 <entry pred="look" stem="take_a_look-NO-SUBJ" pos="V" class="perception">
   <word form="take" macros="@finite @pres @imp @mwe1-a @mwe2-look"/>
   <member-of family="tv.at-obj.NO-SUBJ_W_W"/>
</entry>  

<!-- look around -->
<entry pred="explore" stem="look_around__perception" pos="V" class="perception">
   <word form="look" macros="@nonfin @mwe1-around "/>
   <word form="looking" macros="@vform.progr @pres @ind @mwe1-around "/> 
   <word form="look" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr @mwe1-around "/>
   <word form="look" macros="@finite @pres @ind @num.pl-agr @mwe1-around "/>
   <word form="looks" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr @mwe1-around "/>
   <word form="looked" macros="@finite @past @ind @mwe1-around "/>
   <member-of family="tv_W"/>
</entry>  
<entry pred="explore" stem="look_around__perception" pos="V" class="perception">
   <word form="look" macros="@finite @pres @imp @mwe1-around "/>
   <member-of family="tv.NO-SUBJ_W"/>
</entry>  

  <!-- +++++++++++++++++++++++++++++++++++++++++++++++++++ -->
  <!-- Other Important Stuff                               -->
  <!-- +++++++++++++++++++++++++++++++++++++++++++++++++++ -->


<!-- "NOT" IS NOT REALIZING !!! -->
  <entry pred="-" stem="not__vp-negation" pos="ADV"> 
    <word form="not"/>   
   <member-of family="aux-negation"/> 
	 <member-of family="copular-negation"/> 
  </entry>
  
  <entry stem="to__inf-marker" pos="MARKER">
    <word form="to" />
  	<member-of family="infinitive-to"/>
  </entry>
  	
<!-- must constrain to select unmarked sentence (look at HPSG marker feature)
	 If added, allows infinite recursion and hence NO GENERATION works -->

  <entry pred="that" stem="that__s-marker" pos="MARKER">
    <word form="that"/>
  	<member-of family="sentence-marker"/>
  </entry>
  <entry pred="if" stem="if__s-marker" pos="MARKER">
    <word form="if"/>
  	<member-of family="sentence-marker"/>
  </entry>
  <entry pred="whether" stem="whether__s-marker" pos="MARKER">
    <word form="whether"/>
  	<member-of family="sentence-marker"/>
  </entry>


  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- DISCOURSE UNITS      					        -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  


<entry pred = "good" stem="evaluative-good" pos="DU" class="evaluative">
     <word form="good" macro="@mwe1-job"/>
     <word form="well" macro="@mwe1-done"/>
     <member-of family="dis-unit_W"/>
</entry>

<entry stem="hello" pred = "hello" pos="DU" class="greeting">
     <word form="hello"/>
     <word form="hi"/>
	 <word form="hey"/>
     <member-of family="dis-unit"/>
	 <member-of family="dis-unit.addressee"/>
</entry>


<entry pred="let's_see" stem="discourse-connective_W" pos="DU" class="dis-connective"> 
	  <word form="let's" macro="@mwe1-see"/>
      <member-of family="dis-unit.connective_W"/>
</entry>

<entry stem="let's" pred="let's" pos="DU" class="suggestion">
   <member-of family="dis-unit.suggestion"/>  
</entry>


<entry stem="let's" pred="let's" pos="DU" class="suggestion">
   <member-of family="dis-unit.suggestion"/>  
</entry>

<entry pred="yes" stem="yes__cue" pos="DU" class="cue">
     <word form="yes"/>
     <word form="yeah"/>
     <word form="sure"/>
	 <word form="ok"/>
	 <word form="okay"/>
	 <member-of family="cue.positive"/>
	 <member-of family="sentence-polarity.positive"/>
</entry>
<entry pred="no" stem="no__cue" pos="DU" class="cue"> 
     <word form="no"/>
     <word form="nope"/>
     <member-of family="cue.negative"/>
	 <member-of family="sentence-polarity.negative"/>
</entry>

  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- MWE ADVERBS            					        -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

<!-- comment adverbs -->
<entry pred="I_think" stem="I_think__m-comment" pos="ADV" class="m-comment">
  <word form="I" macros="@mwe1-think"/>
  <member-of family="adv.comment_W"/>
</entry>

<entry pred="I_hope" stem="I_hope__m-comment" pos="ADV" class="m-comment">
  <word form="I" macros="@mwe1-hope"/>
  <member-of family="adv.comment_W"/>
</entry>
<entry pred="I_guess" stem="I_guess__m-comment" pos="ADV" class="m-comment">
  <word form="I" macros="@mwe1-guess"/>
  <member-of family="adv.comment_W"/>
</entry>

<entry pred="you_know" stem="you_know__m-comment" pos="ADV" class="m-comment">
  <word form="you" macros="@mwe1-know"/>
  <member-of family="adv.comment_W"/>
</entry>

<!-- to the right and to the left as directionals ... -->
<entry pred="right" stem="to_the_right__m-direction" pos="ADV" class="m-direction">
  <word form="to" macros="@mwe1-the @mwe1-right"/>
  <member-of family="adv.direction_W_W"/>
</entry>

<entry pred="left" stem="to_the_left__m-direction" pos="ADV" class="m-direction">
  <word form="to" macros="@mwe1-the @mwe1-left"/>
  <member-of family="adv.direction_W_W"/>
</entry>



  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- ADVERBS mod adj        					        -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  

<entry pred="really" stem="really__mod-adj" pos="ADV">
  <word form="really"/>
  <word form="very"/>
  <word form="extremely"/>
  <member-of family="adj-modifier"/>
</entry>


<!-- NOTE: these two have two different entries only because of current 
		   means of handling of unary-rule for complex cat -->
		   
<!-- forward instead of normal backward (i.e. too big but big enough)-->
<entry pred="enough" stem="enough__mod-adj" pos="ADV">
  <word form="enough"/>
  <member-of family="adj-modifier.forward.to-verb"/>
  <member-of family="adj-modifier.forward"/>
</entry>

<entry pred="too" stem="too__mod-adj" pos="ADV">
  <word form="too"/>
  <member-of family="adj-modifier.to-verb"/>
  <member-of family="adj-modifier"/>  
</entry>


<!-- ADJ with OBL args -->

<entry pred="proud" stem="proud" pos="ADJ" class="quality">
  <word form="proud" />
  <member-of family="adj.of-np.property"/>
</entry>

<entry pred="careful" stem="careful" pos="ADJ" class="quality">
  <word form="careful" />
  <member-of family="adj.with-np.property"/>
</entry>


<!-- ADJ with verb args -->

<entry pred="allowed" stem="allowed_adj" pos="ADJ" class="quality">
  <word form="allowed" />
  <member-of family="adj.to-verb.property"/>
</entry>

<entry pred="able" stem="able_adj" pos="ADJ" class="quality">
  <word form="able" />
  <member-of family="adj.to-verb.property"/>
</entry>


  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
  <!-- MWE PREPOSITIONS       					        -->
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ -->  
 
<!-- these two are used for copular "I am here" "the guy there says ..." etc 
	 also over here and over there as well -->
 <entry pred="here" stem="here-mod" pos="PREP">
	<word form="here"/>
  	<member-of family="here-and-there"/>
  </entry>
  <entry pred="there" stem="there-mod" pos="PREP">
	<word form="there"/>
  	<member-of family="here-and-there"/>
  </entry>
  
 <entry pred="here" stem="here-mod_W" pos="PREP">
	<word form="over" macros="@mwe1-here"/>
  	<member-of family="here-and-there_W"/>
 </entry>
 <entry pred="there" stem="there-mod_W" pos="PREP">
	<word form="over" macros="@mwe1-there"/>
  	<member-of family="here-and-there_W"/>
 </entry>
   
	 
<!-- NOTE: using type heirarchy for w-form could "generalize" these 
     but could we get the good pred...... dunno -->

<entry pred="next_to" stem="next_to__m-whereto" pos="PREP" class="m-whereto">
  <word form="next" macros="@mwe1-to"/>
  <member-of family="prep-mod-s.whereto_W"/>
</entry>
<entry pred="next_to" stem="next_to__m-location" pos="PREP" class="m-location">
  <word form="next" macros="@mwe1-to"/>
  <member-of family="prep-mod-n.location_W"/>
  <member-of family="prep-mod-s.location_W"/>
</entry>

<entry pred="connected_to" stem="connected_to__m-location" pos="PREP" class="m-location">
  <word form="connected" macros="@mwe1-to"/>
  <member-of family="prep-mod-n.location_W"/>
  <member-of family="prep-mod-s.location_W"/>
</entry>
<entry pred="surrounded_by" stem="surrounded_by__m-location" pos="PREP" class="m-location">
  <word form="surrounded" macros="@mwe1-by"/>
  <member-of family="prep-mod-n.location_W"/>
  <member-of family="prep-mod-s.location_W"/>  
</entry>

<entry pred="close_to" stem="close_to__m-whereto" pos="PREP" class="m-whereto">
  <word form="close" macros="@mwe1-to"/>
  <member-of family="prep-mod-s.whereto_W"/>
</entry>
<entry pred="close_to" stem="close_to__m-location" pos="PREP" class="m-location">
  <word form="close" macros="@mwe1-to"/>
  <member-of family="prep-mod-n.location_W"/>
  <member-of family="prep-mod-s.location_W"/>
</entry>


<entry pred="in_front" stem="in_front_of__m-whereto" pos="PREP" class="m-whereto">
  <word form="in" macros="@mwe1-front"/>
  <member-of family="prep-mod-s.of-np.whereto_W"/>
</entry>
<entry pred="in_front" stem="in_front_of__m-location" pos="PREP" class="m-location">
  <word form="in" macros="@mwe1-front"/>
  <member-of family="prep-mod-n.of-np.location_W"/>
  <member-of family="prep-mod-s.of-np.location_W"/>
</entry>

<entry pred="in_front" stem="in_front__m-whereto" pos="PREP" class="m-whereto">
  <word form="in_front"/>
  <member-of family="prep-mod-s.of-np.whereto"/>
</entry>
<entry pred="in_front" stem="in_front__m-location" pos="PREP" class="m-location">
  <word form="in_front"/>
  <member-of family="prep-mod-n.of-np.location"/>
  <member-of family="prep-mod-s.of-np.location"/>
</entry>


<entry pred="in_back" stem="in_back_of__m-whereto" pos="PREP" class="m-whereto">
  <word form="in" macros="@mwe1-back"/>
  <member-of family="prep-mod-s.of-np.whereto_W"/>
</entry>
<entry pred="in_back" stem="in_back_of__m-location" pos="PREP" class="m-location">
  <word form="in" macros="@mwe1-back"/>
  <member-of family="prep-mod-n.of-np.location_W"/>
  <member-of family="prep-mod-s.of-np.location_W"/>
</entry>

<entry pred="on_back" stem="on_back_of__m-whereto" pos="PREP" class="m-whereto">
  <word form="on" macros="@mwe1-back"/>
  <member-of family="prep-mod-s.of-np.whereto_W"/>
</entry>
<entry pred="on_back" stem="on_back_of__m-location" pos="PREP" class="m-location">
  <word form="on" macros="@mwe1-back"/>
  <member-of family="prep-mod-n.of-np.location_W"/>
  <member-of family="prep-mod-s.of-np.location_W"/>
</entry>

<entry pred="on_front" stem="on_front_of__m-whereto" pos="PREP" class="m-whereto">
  <word form="on" macros="@mwe1-front"/>
  <member-of family="prep-mod-s.of-np.whereto_W"/>
</entry>
<entry pred="on_front" stem="on_front_of__m-location" pos="PREP" class="m-location">
  <word form="on" macros="@mwe1-front"/>
  <member-of family="prep-mod-n.of-np.location_W"/>
  <member-of family="prep-mod-s.of-np.location_W"/>
</entry>  

<entry pred="on_bottom" stem="on_bottom_of__m-whereto" pos="PREP" class="m-whereto">
  <word form="on" macros="@mwe1-bottom"/>
  <member-of family="prep-mod-s.of-np.whereto_W"/>
</entry>
<entry pred="on_bottom" stem="on_bottom_of__m-location" pos="PREP" class="m-location">
  <word form="on" macros="@mwe1-bottom"/>
  <member-of family="prep-mod-n.of-np.location_W"/>
  <member-of family="prep-mod-s.of-np.location_W"/>
</entry>  

<!-- "to the left of" & "to the right of" treated as identical to "right of" -->
<entry pred="right" stem="to_the_right_of__m-whereto" pos="PREP" class="m-whereto">
  <word form="to" macros="@mwe1-the @mwe2-right"/>
  <member-of family="prep-mod-s.of-np.whereto_W_W"/>
</entry>
<entry pred="right" stem="to_the_right_of__m-location" pos="PREP" class="m-location">
  <word form="to" macros="@mwe1-the @mwe2-right"/>
  <member-of family="prep-mod-n.of-np.location_W_W"/>
  <member-of family="prep-mod-s.of-np.location_W_W"/>
</entry> 

<entry pred="left" stem="to_the_left_of__m-whereto" pos="PREP" class="m-whereto">
  <word form="to" macros="@mwe1-the @mwe2-left"/>
  <member-of family="prep-mod-s.of-np.whereto_W_W"/>
</entry>
<entry pred="left" stem="to_the_left_of__m-location" pos="PREP" class="m-location">
  <word form="to" macros="@mwe1-the @mwe2-left"/>
  <member-of family="prep-mod-n.of-np.location_W_W"/>
  <member-of family="prep-mod-s.of-np.location_W_W"/>
</entry>  
 
<entry pred="front" stem="to_the_front_of__m-whereto" pos="PREP" class="m-whereto">
  <word form="to" macros="@mwe1-the @mwe2-front"/>
  <member-of family="prep-mod-s.of-np.whereto_W_W"/>
</entry>
<entry pred="front" stem="to_the_front_of__m-location" pos="PREP" class="m-location">
  <word form="to" macros="@mwe1-the @mwe2-front"/>
  <member-of family="prep-mod-n.of-np.location_W_W"/>
  <member-of family="prep-mod-s.of-np.location_W_W"/>
</entry>  

<entry pred="back" stem="to_the_back_of__m-whereto" pos="PREP" class="m-whereto">
  <word form="to" macros="@mwe1-the @mwe2-back"/>
  <member-of family="prep-mod-s.of-np.whereto_W_W"/>
</entry>
<entry pred="back" stem="to_the_back_of__m-location" pos="PREP" class="m-location">
  <word form="to" macros="@mwe1-the @mwe2-back"/>
  <member-of family="prep-mod-n.of-np.location_W_W"/>
  <member-of family="prep-mod-s.of-np.location_W_W"/>
</entry>  

<entry pred="away_from" stem="away_from_of__m-whereto" pos="PREP" class="m-whereto">
  <word form="away" macros="@mwe1-from"/>
  <member-of family="prep-mod-s.wherefrom_W"/>
</entry>


<!-- RANDOMLY, THIS WON'T WORK.
		It appears in all other on readings......

<entry pred="on_top" stem="on_top_of__m-whereto" pos="PREP" class="m-whereto">
  <word form="on" macros="@mwe1-top"/>
  <member-of family="prep-mod-s.of-np.whereto_W"/>
</entry>
<entry pred="on_top" stem="on_top_of__m-location" pos="PREP" class="m-location">
  <word form="on" macros="@mwe1-top"/>
  <member-of family="prep-mod-n.of-np.location_W"/>
  <member-of family="prep-mod-s.of-np.location_W"/>
</entry>  
-->



  
  <!-- ================================================ -->
  <!-- =====									 ====== --> 
  <!-- =====  ADD ALL GENERATED LEX ITEMS        ====== -->
  <!-- =====		BETWEEN	HERE     			 ====== --> 
  <!-- =====									 ====== --> 
  <!-- ================================================ -->
  
  
  <!-- __INSERT__HERE__ -->
  

  <!-- ================================================ -->
  <!-- =====	  AND HERE						 ====== --> 
  <!-- ================================================ -->


  <xsl:call-template name="add-macros"/>
 

  </dictionary>
</xsl:template>

</xsl:transform>


