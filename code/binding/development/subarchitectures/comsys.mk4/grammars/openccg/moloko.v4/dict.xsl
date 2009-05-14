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
  
  
  <!-- ++++++++++++++++++++++++++++++++++++++++++++++++ 
     +  ADJ
     ++++++++++++++++++++++++++++++++++++++++++++++++ -->


<entry pred="good" stem="good__q-attitude"  pos="ADJ" class="q-attitude">
  <word form="good" macros="@degree.base"/>
  <word form="better" macros="@degree.comparative"/>
  <word form="best" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="bad" stem="bad__q-attitude"  pos="ADJ" class="q-attitude">
  <word form="bad" macros="@degree.base"/>
  <word form="worse" macros="@degree.comparative"/>
  <word form="worst" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="guilty" stem="guilty__q-attitude"  pos="ADJ" class="q-attitude">
  <word form="guilty" macros="@degree.base"/>
  <word form="guiltier" macros="@degree.comparative"/>
  <word form="guiltiest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="frustrated" stem="frustrated__q-attitude"  pos="ADJ" class="q-attitude">
  <word form="frustrated" macros="@degree.base"/>
  <word form="more_frustrated" macros="@degree.comparative"/>
  <word form="most_frustrated" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="sure" stem="sure__q-attitude"  pos="ADJ" class="q-attitude">
  <word form="sure" macros="@degree.base"/>
  <word form="more_sure" macros="@degree.comparative"/>
  <word form="most_sure" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="sorry" stem="sorry__q-attitude"  pos="ADJ" class="q-attitude">
  <word form="sorry" macros="@degree.base"/>
  <word form="sorrier" macros="@degree.comparative"/>
  <word form="sorriest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="ok" stem="ok__q-attitude"  pos="ADJ" class="q-attitude">
  <word form="ok" macros="@degree.base"/>
  <word form="---" macros="@degree.comparative"/>
  <word form="---" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="cool" stem="cool__q-attitude"  pos="ADJ" class="q-attitude">
  <word form="cool" macros="@degree.base"/>
  <word form="cooler" macros="@degree.comparative"/>
  <word form="coolest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="important" stem="important__q-attitude"  pos="ADJ" class="q-attitude">
  <word form="important" macros="@degree.base"/>
  <word form="more_important" macros="@degree.comparative"/>
  <word form="most_important" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="big" stem="big__q-size"  pos="ADJ" class="q-size">
  <word form="big" macros="@degree.base"/>
  <word form="bigger" macros="@degree.comparative"/>
  <word form="biggest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="small" stem="small__q-size"  pos="ADJ" class="q-size">
  <word form="small" macros="@degree.base"/>
  <word form="smaller" macros="@degree.comparative"/>
  <word form="smallest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="tiny" stem="tiny__q-size"  pos="ADJ" class="q-size">
  <word form="tiny" macros="@degree.base"/>
  <word form="tiny" macros="@degree.comparative"/>
  <word form="tiniest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="red" stem="red__q-color"  pos="ADJ" class="q-color">
  <word form="red" macros="@degree.base"/>
  <word form="redder" macros="@degree.comparative"/>
  <word form="reddest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="blue" stem="blue__q-color"  pos="ADJ" class="q-color">
  <word form="blue" macros="@degree.base"/>
  <word form="bluer" macros="@degree.comparative"/>
  <word form="bluest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="green" stem="green__q-color"  pos="ADJ" class="q-color">
  <word form="green" macros="@degree.base"/>
  <word form="greener" macros="@degree.comparative"/>
  <word form="greenest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="yellow" stem="yellow__q-color"  pos="ADJ" class="q-color">
  <word form="yellow" macros="@degree.base"/>
  <word form="yellower" macros="@degree.comparative"/>
  <word form="yellowest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="pink" stem="pink__q-color"  pos="ADJ" class="q-color">
  <word form="pink" macros="@degree.base"/>
  <word form="pinker" macros="@degree.comparative"/>
  <word form="pinkest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="black" stem="black__q-color"  pos="ADJ" class="q-color">
  <word form="black" macros="@degree.base"/>
  <word form="blacker" macros="@degree.comparative"/>
  <word form="blackest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="white" stem="white__q-color"  pos="ADJ" class="q-color">
  <word form="white" macros="@degree.base"/>
  <word form="whiter" macros="@degree.comparative"/>
  <word form="whitest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="round" stem="round__q-shape"  pos="ADJ" class="q-shape">
  <word form="round" macros="@degree.base"/>
  <word form="rounder" macros="@degree.comparative"/>
  <word form="roundest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="square" stem="square__q-shape"  pos="ADJ" class="q-shape">
  <word form="square" macros="@degree.base"/>
  <word form="more_square" macros="@degree.comparative"/>
  <word form="squarest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="long" stem="long__q-shape"  pos="ADJ" class="q-shape">
  <word form="long" macros="@degree.base"/>
  <word form="longer" macros="@degree.comparative"/>
  <word form="longest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="short" stem="short__q-shape"  pos="ADJ" class="q-shape">
  <word form="short" macros="@degree.base"/>
  <word form="shorter" macros="@degree.comparative"/>
  <word form="shortest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="oval" stem="oval__q-shape"  pos="ADJ" class="q-shape">
  <word form="oval" macros="@degree.base"/>
  <word form="more_oval" macros="@degree.comparative"/>
  <word form="most_oval" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="open" stem="open__q-physical"  pos="ADJ" class="q-physical">
  <word form="open" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="closed" stem="closed__q-physical"  pos="ADJ" class="q-physical">
  <word form="closed" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="cluttered" stem="cluttered__q-physical"  pos="ADJ" class="q-physical">
  <word form="cluttered" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="accessible" stem="accessible__q-physical"  pos="ADJ" class="q-physical">
  <word form="accessible" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="inaccessible" stem="inaccessible__q-physical"  pos="ADJ" class="q-physical">
  <word form="inaccessible" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="reachable" stem="reachable__q-physical"  pos="ADJ" class="q-physical">
  <word form="reachable" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="unreachable" stem="unreachable__q-physical"  pos="ADJ" class="q-physical">
  <word form="unreachable" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="visible" stem="visible__q-physical"  pos="ADJ" class="q-physical">
  <word form="visible" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="invisible" stem="invisible__q-physical"  pos="ADJ" class="q-physical">
  <word form="invisible" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="graspable" stem="graspable__q-physical"  pos="ADJ" class="q-physical">
  <word form="graspable" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="ungraspable" stem="ungraspable__q-physical"  pos="ADJ" class="q-physical">
  <word form="ungraspable" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="right" stem="right__q-location"  pos="ADJ" class="q-location">
  <word form="right" macros="@degree.base"/>
  <word form="more_to_the_right" macros="@degree.comparative"/>
  <word form="right_most" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="left" stem="left__q-location"  pos="ADJ" class="q-location">
  <word form="left" macros="@degree.base"/>
  <word form="more_to_the_left" macros="@degree.comparative"/>
  <word form="left_most" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="top" stem="top__q-location"  pos="ADJ" class="q-location">
  <word form="top" macros="@degree.base"/>
  <word form="more_to_the_top" macros="@degree.comparative"/>
  <word form="top_most" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="bottom" stem="bottom__q-location"  pos="ADJ" class="q-location">
  <word form="bottom" macros="@degree.base"/>
  <word form="more_to_the_bottom" macros="@degree.comparative"/>
  <word form="bottom_most" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="middle" stem="middle__q-location"  pos="ADJ" class="q-location">
  <word form="middle" macros="@degree.base"/>
  <word form="more_to_the_middle" macros="@degree.comparative"/>
  <word form="middle_most" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="close" stem="close__q-location"  pos="ADJ" class="q-location">
  <word form="close" macros="@degree.base"/>
  <word form="closer" macros="@degree.comparative"/>
  <word form="closest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="near" stem="near__q-location"  pos="ADJ" class="q-location">
  <word form="near" macros="@degree.base"/>
  <word form="nearer" macros="@degree.comparative"/>
  <word form="nearest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="far" stem="far__q-location"  pos="ADJ" class="q-location">
  <word form="far" macros="@degree.base"/>
  <word form="farther" macros="@degree.comparative"/>
  <word form="farthest" macros="@degree.superlative"/>
  <word form="far" macros="@degree.base"/>
  <word form="further" macros="@degree.comparative"/>
  <word form="furtherest" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="first" stem="first__q-number"  pos="ADJ" class="q-number">
  <word form="first" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="second" stem="second__q-number"  pos="ADJ" class="q-number">
  <word form="second" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="third" stem="third__q-number"  pos="ADJ" class="q-number">
  <word form="third" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="fourth" stem="fourth__q-number"  pos="ADJ" class="q-number">
  <word form="fourth" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="fifth" stem="fifth__q-number"  pos="ADJ" class="q-number">
  <word form="fifth" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="sixth" stem="sixth__q-number"  pos="ADJ" class="q-number">
  <word form="sixth" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="seventh" stem="seventh__q-number"  pos="ADJ" class="q-number">
  <word form="seventh" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="eighth" stem="eighth__q-number"  pos="ADJ" class="q-number">
  <word form="eighth" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="ninth" stem="ninth__q-number"  pos="ADJ" class="q-number">
  <word form="ninth" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="tenth" stem="tenth__q-number"  pos="ADJ" class="q-number">
  <word form="tenth" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="eleventh" stem="eleventh__q-number"  pos="ADJ" class="q-number">
  <word form="eleventh" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="first" stem="first__q-position"  pos="ADJ" class="q-position">
  <word form="first" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="last" stem="last__q-position"  pos="ADJ" class="q-position">
  <word form="last" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="next" stem="next__q-position"  pos="ADJ" class="q-position">
  <word form="next" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="previous" stem="previous__q-position"  pos="ADJ" class="q-position">
  <word form="previous" macros="@degree.base"/>
  <word form="--" macros="@degree.comparative"/>
  <word form="--" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<entry pred="other" stem="other__q-relational"  pos="ADJ" class="q-relational">
  <word form="other" macros="@degree.base"/>
  <word form="---" macros="@degree.comparative"/>
  <word form="---" macros="@degree.superlative"/>
  <member-of family="adj.property"/>
</entry>

<!-- ++++++++++++++++++++++++++++++++++++++++++++++++ 
     +  N
     ++++++++++++++++++++++++++++++++++++++++++++++++ -->


<entry pred="boy" stem="boy-sg"  pos="N" class="person">
  <word form="boy" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="boy" stem="boy-pl"  pos="N" class="person">
  <word form="boys" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="girl" stem="girl-sg"  pos="N" class="person">
  <word form="girl" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="girl" stem="girl-pl"  pos="N" class="person">
  <word form="girls" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="man" stem="man-sg"  pos="N" class="person">
  <word form="man" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="man" stem="man-pl"  pos="N" class="person">
  <word form="men" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="woman" stem="woman-sg"  pos="N" class="person">
  <word form="woman" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="woman" stem="woman-pl"  pos="N" class="person">
  <word form="women" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="person" stem="person-sg"  pos="N" class="person">
  <word form="person" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="person" stem="person-pl"  pos="N" class="person">
  <word form="people" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="guy" stem="guy-sg"  pos="N" class="person">
  <word form="guy" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="guy" stem="guy-pl"  pos="N" class="person">
  <word form="guys" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="child" stem="child-sg"  pos="N" class="person">
  <word form="child" macros="@pers.3rd"/>
  <word form="kid" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="child" stem="child-pl"  pos="N" class="person">
  <word form="children" macros="@pers.3rd"/>
  <word form="kids" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="secretary" stem="secretary-sg"  pos="N" class="person">
  <word form="secretary" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="secretary" stem="secretary-pl"  pos="N" class="person">
  <word form="secretaries" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="janitor" stem="janitor-sg"  pos="N" class="person">
  <word form="janitor" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="janitor" stem="janitor-pl"  pos="N" class="person">
  <word form="janitors" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="student" stem="student-sg"  pos="N" class="person">
  <word form="student" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="student" stem="student-pl"  pos="N" class="person">
  <word form="students" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="researcher" stem="researcher-sg"  pos="N" class="person">
  <word form="researcher" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="researcher" stem="researcher-pl"  pos="N" class="person">
  <word form="researchers" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="pirate" stem="pirate-sg"  pos="N" class="person">
  <word form="pirate" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="pirate" stem="pirate-pl"  pos="N" class="person">
  <word form="pirates" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="GJ" stem="GJ-bare__person"  pos="N" class="person">
  <word form="GJ" macros="@pers.3rd"/>
  <word form="gj" macros="@pers.3rd"/>
  <member-of family="bare-np"/>
</entry>

<entry pred="Trevor" stem="Trevor-bare__person"  pos="N" class="person">
  <word form="Trevor" macros="@pers.3rd"/>
  <word form="trevor" macros="@pers.3rd"/>
  <member-of family="bare-np"/>
</entry>

<entry pred="Rob" stem="Rob-bare__person"  pos="N" class="person">
  <word form="Rob" macros="@pers.3rd"/>
  <word form="rob" macros="@pers.3rd"/>
  <member-of family="bare-np"/>
</entry>

<entry pred="Hendrik" stem="Hendrik-bare__person"  pos="N" class="person">
  <word form="Hendrik" macros="@pers.3rd"/>
  <word form="hendrik" macros="@pers.3rd"/>
  <member-of family="bare-np"/>
</entry>

<entry pred="Henrik" stem="Henrik-bare__person"  pos="N" class="person">
  <word form="Henrik" macros="@pers.3rd"/>
  <word form="henrik" macros="@pers.3rd"/>
  <member-of family="bare-np"/>
</entry>

<entry pred="Patric" stem="Patric-bare__person"  pos="N" class="person">
  <word form="Patric" macros="@pers.3rd"/>
  <word form="patric" macros="@pers.3rd"/>
  <member-of family="bare-np"/>
</entry>

<entry pred="Nick" stem="Nick-bare__person"  pos="N" class="person">
  <word form="Nick" macros="@pers.3rd"/>
  <word form="nick" macros="@pers.3rd"/>
  <member-of family="bare-np"/>
</entry>

<entry pred="Michael" stem="Michael-bare__person"  pos="N" class="person">
  <word form="Michael" macros="@pers.3rd"/>
  <word form="michael" macros="@pers.3rd"/>
  <member-of family="bare-np"/>
</entry>

<entry pred="Oscar" stem="Oscar-bare__person"  pos="N" class="person">
  <word form="Oscar" macros="@pers.3rd"/>
  <word form="oscar" macros="@pers.3rd"/>
  <member-of family="bare-np"/>
</entry>

<entry pred="Pierre" stem="Pierre-bare__person"  pos="N" class="person">
  <word form="Pierre" macros="@pers.3rd"/>
  <word form="pierre" macros="@pers.3rd"/>
  <member-of family="bare-np"/>
</entry>

<entry pred="Daniel" stem="Daniel-bare__person"  pos="N" class="person">
  <word form="Daniel" macros="@pers.3rd"/>
  <word form="daniel" macros="@pers.3rd"/>
  <member-of family="bare-np"/>
</entry>

<entry pred="Maria" stem="Maria-bare__person"  pos="N" class="person">
  <word form="Maria" macros="@pers.3rd"/>
  <word form="maria" macros="@pers.3rd"/>
  <member-of family="bare-np"/>
</entry>

<entry pred="Jeremy" stem="Jeremy-bare__person"  pos="N" class="person">
  <word form="Jeremy" macros="@pers.3rd"/>
  <word form="jeremy" macros="@pers.3rd"/>
  <member-of family="bare-np"/>
</entry>

<entry pred="Chandana" stem="Chandana-bare__person"  pos="N" class="person">
  <word form="Chandana" macros="@pers.3rd"/>
  <word form="chandana" macros="@pers.3rd"/>
  <member-of family="bare-np"/>
</entry>

<entry pred="Aaron" stem="Aaron-bare__person"  pos="N" class="person">
  <word form="Aaron" macros="@pers.3rd"/>
  <word form="aaron" macros="@pers.3rd"/>
  <member-of family="bare-np"/>
</entry>

<entry pred="dog" stem="dog-sg"  pos="N" class="animate">
  <word form="dog" macros="@pers.3rd"/>
  <word form="canine" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="dog" stem="dog-pl"  pos="N" class="animate">
  <word form="dogs" macros="@pers.3rd"/>
  <word form="canines" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="fish" stem="fish-sg"  pos="N" class="animate">
  <word form="fish" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="fish" stem="fish-pl"  pos="N" class="animate">
  <word form="fish" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="cow" stem="cow-sg"  pos="N" class="animate">
  <word form="cow" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="cow" stem="cow-pl"  pos="N" class="animate">
  <word form="cows" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="Robot" stem="Robot-bare__animate"  pos="N" class="animate">
  <word form="Robot" macros="@pers.3rd"/>
  <word form="robot" macros="@pers.3rd"/>
  <member-of family="bare-np"/>
</entry>

<entry pred="robot" stem="robot-sg"  pos="N" class="animate">
  <word form="robot" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="robot" stem="robot-pl"  pos="N" class="animate">
  <word form="robots" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="thing" stem="thing-sg"  pos="N" class="thing">
  <word form="thing" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="thing" stem="thing-pl"  pos="N" class="thing">
  <word form="things" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="book" stem="book-sg"  pos="N" class="thing">
  <word form="book" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="book" stem="book-pl"  pos="N" class="thing">
  <word form="books" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="ball" stem="ball-sg"  pos="N" class="thing">
  <word form="ball" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="ball" stem="ball-pl"  pos="N" class="thing">
  <word form="balls" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="block" stem="block-sg"  pos="N" class="thing">
  <word form="block" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="block" stem="block-pl"  pos="N" class="thing">
  <word form="blocks" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="box" stem="box-sg"  pos="N" class="thing">
  <word form="box" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="box" stem="box-pl"  pos="N" class="thing">
  <word form="boxes" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="bucket" stem="bucket-sg"  pos="N" class="thing">
  <word form="bucket" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="bucket" stem="bucket-pl"  pos="N" class="thing">
  <word form="buckets" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="pin" stem="pin-sg"  pos="N" class="thing">
  <word form="pin" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="pin" stem="pin-pl"  pos="N" class="thing">
  <word form="pins" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="cube" stem="cube-sg"  pos="N" class="thing">
  <word form="cube" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="cube" stem="cube-pl"  pos="N" class="thing">
  <word form="cubes" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="chocolate" stem="chocolate-sg"  pos="N" class="thing">
  <word form="chocolate" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="chocolate" stem="chocolate-pl"  pos="N" class="thing">
  <word form="chocolates" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="chocolate" stem="chocolate-mass__thing"  pos="N" class="thing">
  <word form="chocolate" macros="@pers.3rd"/>
  <member-of family="noun.mass"/>
</entry>

<entry pred="trophy" stem="trophy-sg"  pos="N" class="thing">
  <word form="trophy" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="trophy" stem="trophy-pl"  pos="N" class="thing">
  <word form="trophies" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="tooth" stem="tooth-sg"  pos="N" class="thing">
  <word form="tooth" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="tooth" stem="tooth-pl"  pos="N" class="thing">
  <word form="teeth" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="tea" stem="tea-sg"  pos="N" class="thing">
  <word form="tea" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="tea" stem="tea-pl"  pos="N" class="thing">
  <word form="teas" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="tea" stem="tea-mass__thing"  pos="N" class="thing">
  <word form="tea" macros="@pers.3rd"/>
  <member-of family="noun.mass"/>
</entry>

<entry pred="coffee" stem="coffee-sg"  pos="N" class="thing">
  <word form="coffee" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="coffee" stem="coffee-pl"  pos="N" class="thing">
  <word form="coffees" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="coffee" stem="coffee-mass__thing"  pos="N" class="thing">
  <word form="coffee" macros="@pers.3rd"/>
  <member-of family="noun.mass"/>
</entry>

<entry pred="mug" stem="mug-sg"  pos="N" class="thing">
  <word form="mug" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="mug" stem="mug-pl"  pos="N" class="thing">
  <word form="mugs" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="cup" stem="cup-sg"  pos="N" class="thing">
  <word form="cup" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="cup" stem="cup-pl"  pos="N" class="thing">
  <word form="cups" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="keyboard" stem="keyboard-sg"  pos="N" class="thing">
  <word form="keyboard" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="keyboard" stem="keyboard-pl"  pos="N" class="thing">
  <word form="keyboards" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="laptop" stem="laptop-sg"  pos="N" class="thing">
  <word form="laptop" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="laptop" stem="laptop-pl"  pos="N" class="thing">
  <word form="laptops" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="screen" stem="screen-sg"  pos="N" class="thing">
  <word form="screen" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="screen" stem="screen-pl"  pos="N" class="thing">
  <word form="screens" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="computer" stem="computer-sg"  pos="N" class="thing">
  <word form="computer" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="computer" stem="computer-pl"  pos="N" class="thing">
  <word form="computers" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="mp3-player" stem="mp3-player-sg"  pos="N" class="thing">
  <word form="mp3-player" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="mp3-player" stem="mp3-player-pl"  pos="N" class="thing">
  <word form="mp3-players" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="minidisc-recorder" stem="minidisc-recorder-sg"  pos="N" class="thing">
  <word form="minidisc-recorder" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="minidisc-recorder" stem="minidisc-recorder-pl"  pos="N" class="thing">
  <word form="minidisc-recorders" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="needle" stem="needle-sg"  pos="N" class="thing">
  <word form="needle" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="needle" stem="needle-pl"  pos="N" class="thing">
  <word form="needles" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="thread" stem="thread-mass__thing"  pos="N" class="thing">
  <word form="thread" macros="@pers.3rd"/>
  <member-of family="noun.mass"/>
</entry>

<entry pred="thread" stem="thread-sg"  pos="N" class="thing">
  <word form="thread" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="thread" stem="thread-pl"  pos="N" class="thing">
  <word form="threads" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="flower" stem="flower-sg"  pos="N" class="thing">
  <word form="flower" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="flower" stem="flower-pl"  pos="N" class="thing">
  <word form="flowers" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="scissors" stem="scissors-mass__thing"  pos="N" class="thing">
  <word form="scissors" macros="@pers.3rd"/>
  <member-of family="noun.mass"/>
</entry>

<entry pred="food" stem="food-mass__thing"  pos="N" class="thing">
  <word form="food" macros="@pers.3rd"/>
  <member-of family="noun.mass"/>
</entry>

<entry pred="phone" stem="phone-sg"  pos="N" class="thing">
  <word form="phone" macros="@pers.3rd"/>
  <word form="telephone" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="phone" stem="phone-pl"  pos="N" class="thing">
  <word form="phones" macros="@pers.3rd"/>
  <word form="telephones" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="letter" stem="letter-sg"  pos="N" class="thing">
  <word form="letter" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="letter" stem="letter-pl"  pos="N" class="thing">
  <word form="letters" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="mailbox" stem="mailbox-sg"  pos="N" class="thing">
  <word form="mailbox" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="mailbox" stem="mailbox-pl"  pos="N" class="thing">
  <word form="mailboxes" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="copier" stem="copier-sg"  pos="N" class="thing">
  <word form="copier" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="copier" stem="copier-pl"  pos="N" class="thing">
  <word form="copiers" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="fax" stem="fax-sg"  pos="N" class="thing">
  <word form="fax" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="fax" stem="fax-pl"  pos="N" class="thing">
  <word form="faxes" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="printer" stem="printer-sg"  pos="N" class="thing">
  <word form="printer" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="printer" stem="printer-pl"  pos="N" class="thing">
  <word form="printers" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="column" stem="column-sg"  pos="N" class="thing">
  <word form="column" macros="@pers.3rd"/>
  <word form="pillar" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="column" stem="column-pl"  pos="N" class="thing">
  <word form="columns" macros="@pers.3rd"/>
  <word form="pillars" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="furniture" stem="furniture-mass__thing"  pos="N" class="thing">
  <word form="furniture" macros="@pers.3rd"/>
  <member-of family="noun.mass"/>
</entry>

<entry pred="floor" stem="floor-sg"  pos="N" class="thing">
  <word form="floor" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="floor" stem="floor-pl"  pos="N" class="thing">
  <word form="floors" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="door" stem="door-sg"  pos="N" class="thing">
  <word form="door" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="door" stem="door-pl"  pos="N" class="thing">
  <word form="doors" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="couch" stem="couch-sg"  pos="N" class="thing">
  <word form="couch" macros="@pers.3rd"/>
  <word form="sofa" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="couch" stem="couch-pl"  pos="N" class="thing">
  <word form="couches" macros="@pers.3rd"/>
  <word form="sofas" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="tv" stem="tv-sg"  pos="N" class="thing">
  <word form="tv" macros="@pers.3rd"/>
  <word form="television" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="tv" stem="tv-pl"  pos="N" class="thing">
  <word form="tvs" macros="@pers.3rd"/>
  <word form="televisions" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="oven" stem="oven-sg"  pos="N" class="thing">
  <word form="oven" macros="@pers.3rd"/>
  <word form="stove" macros="@pers.3rd"/>
  <word form="cooker" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="oven" stem="oven-pl"  pos="N" class="thing">
  <word form="ovens" macros="@pers.3rd"/>
  <word form="stoves" macros="@pers.3rd"/>
  <word form="cookers" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="tabletop" stem="tabletop-sg"  pos="N" class="thing">
  <word form="tabletop" macros="@pers.3rd"/>
  <word form="desktop" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="tabletop" stem="tabletop-pl"  pos="N" class="thing">
  <word form="tabletops" macros="@pers.3rd"/>
  <word form="desktops" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="fridge" stem="fridge-sg"  pos="N" class="thing">
  <word form="fridge" macros="@pers.3rd"/>
  <word form="refrigerator" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="fridge" stem="fridge-pl"  pos="N" class="thing">
  <word form="fridges" macros="@pers.3rd"/>
  <word form="refrigerators" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="microwave" stem="microwave-sg"  pos="N" class="thing">
  <word form="microwave" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="microwave" stem="microwave-pl"  pos="N" class="thing">
  <word form="microwaves" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="shelf" stem="shelf-sg"  pos="N" class="thing">
  <word form="shelf" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="shelf" stem="shelf-pl"  pos="N" class="thing">
  <word form="shelves" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="cupboard" stem="cupboard-sg"  pos="N" class="thing">
  <word form="cupboard" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="cupboard" stem="cupboard-pl"  pos="N" class="thing">
  <word form="cupboards" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="closet" stem="closet-sg"  pos="N" class="thing">
  <word form="closet" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="closet" stem="closet-pl"  pos="N" class="thing">
  <word form="closets" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="desk" stem="desk-sg"  pos="N" class="thing">
  <word form="desk" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="desk" stem="desk-pl"  pos="N" class="thing">
  <word form="desks" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="table" stem="table-sg"  pos="N" class="thing">
  <word form="table" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="table" stem="table-pl"  pos="N" class="thing">
  <word form="tables" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="chair" stem="chair-sg"  pos="N" class="thing">
  <word form="chair" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="chair" stem="chair-pl"  pos="N" class="thing">
  <word form="chairs" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="competition" stem="competition-sg"  pos="N" class="e-event">
  <word form="competition" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="competition" stem="competition-pl"  pos="N" class="e-event">
  <word form="competitions" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="area" stem="area-sg"  pos="N" class="e-location">
  <word form="area" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="area" stem="area-pl"  pos="N" class="e-location">
  <word form="areas" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="location" stem="location-sg"  pos="N" class="e-location">
  <word form="location" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="location" stem="location-pl"  pos="N" class="e-location">
  <word form="locations" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="office" stem="office-sg"  pos="N" class="e-location">
  <word form="office" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="office" stem="office-pl"  pos="N" class="e-location">
  <word form="offices" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="room" stem="room-sg"  pos="N" class="e-location">
  <word form="room" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="room" stem="room-pl"  pos="N" class="e-location">
  <word form="rooms" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="hall" stem="hall-sg"  pos="N" class="e-location">
  <word form="hall" macros="@pers.3rd"/>
  <word form="hallway" macros="@pers.3rd"/>
  <word form="corridor" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="hall" stem="hall-pl"  pos="N" class="e-location">
  <word form="halls" macros="@pers.3rd"/>
  <word form="hallways" macros="@pers.3rd"/>
  <word form="corridors" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="bathroom" stem="bathroom-sg"  pos="N" class="e-location">
  <word form="bathroom" macros="@pers.3rd"/>
  <word form="washroom" macros="@pers.3rd"/>
  <word form="toilet" macros="@pers.3rd"/>
  <word form="wc" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="bathroom" stem="bathroom-pl"  pos="N" class="e-location">
  <word form="bathrooms" macros="@pers.3rd"/>
  <word form="washrooms" macros="@pers.3rd"/>
  <word form="toilets" macros="@pers.3rd"/>
  <word form="wcs" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="lab" stem="lab-sg"  pos="N" class="e-location">
  <word form="lab" macros="@pers.3rd"/>
  <word form="laboratory" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="lab" stem="lab-pl"  pos="N" class="e-location">
  <word form="labs" macros="@pers.3rd"/>
  <word form="laboratorys" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="kitchen" stem="kitchen-sg"  pos="N" class="e-location">
  <word form="kitchen" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="kitchen" stem="kitchen-pl"  pos="N" class="e-location">
  <word form="kitchens" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="library" stem="library-sg"  pos="N" class="e-location">
  <word form="library" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="library" stem="library-pl"  pos="N" class="e-location">
  <word form="librarys" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="secretariat" stem="secretariat-sg"  pos="N" class="e-location">
  <word form="secretariat" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="secretariat" stem="secretariat-pl"  pos="N" class="e-location">
  <word form="secretariats" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="lobby" stem="lobby-sg"  pos="N" class="e-location">
  <word form="lobby" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="lobby" stem="lobby-pl"  pos="N" class="e-location">
  <word form="lobbys" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="entrance" stem="entrance-sg"  pos="N" class="e-location">
  <word form="entrance" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="entrance" stem="entrance-pl"  pos="N" class="e-location">
  <word form="entrances" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="lift" stem="lift-sg"  pos="N" class="e-location">
  <word form="lift" macros="@pers.3rd"/>
  <word form="elevator" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="lift" stem="lift-pl"  pos="N" class="e-location">
  <word form="lifts" macros="@pers.3rd"/>
  <word form="elevators" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="staircase" stem="staircase-sg"  pos="N" class="e-location">
  <word form="staircase" macros="@pers.3rd"/>
  <word form="stairs" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="staircase" stem="staircase-pl"  pos="N" class="e-location">
  <word form="staircases" macros="@pers.3rd"/>
  <word form="--" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="floor" stem="floor-sg"  pos="N" class="e-location">
  <word form="floor" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="floor" stem="floor-pl"  pos="N" class="e-location">
  <word form="floors" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="way" stem="way-sg"  pos="N" class="e-location">
  <word form="way" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="way" stem="way-pl"  pos="N" class="e-location">
  <word form="ways" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="direction" stem="direction-sg"  pos="N" class="e-location">
  <word form="direction" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="direction" stem="direction-pl"  pos="N" class="e-location">
  <word form="directions" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="university" stem="university-sg"  pos="N" class="e-location">
  <word form="university" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="university" stem="university-pl"  pos="N" class="e-location">
  <word form="universities" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="top" stem="top-sg"  pos="N" class="part">
  <word form="top" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="top" stem="top-pl"  pos="N" class="part">
  <word form="tops" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="bottom" stem="bottom-sg"  pos="N" class="part">
  <word form="bottom" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="bottom" stem="bottom-pl"  pos="N" class="part">
  <word form="bottoms" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="front" stem="front-sg"  pos="N" class="part">
  <word form="front" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="front" stem="front-pl"  pos="N" class="part">
  <word form="fronts" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="back" stem="back-sg"  pos="N" class="part">
  <word form="back" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="back" stem="back-pl"  pos="N" class="part">
  <word form="backs" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="left" stem="left-sg"  pos="N" class="part">
  <word form="left" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="left" stem="left-pl"  pos="N" class="part">
  <word form="lefts" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="right" stem="right-sg"  pos="N" class="part">
  <word form="right" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="right" stem="right-pl"  pos="N" class="part">
  <word form="rights" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="side" stem="side-sg"  pos="N" class="part">
  <word form="side" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="side" stem="side-pl"  pos="N" class="part">
  <word form="sides" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="name" stem="name-sg"  pos="N" class="entity">
  <word form="name" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="name" stem="name-pl"  pos="N" class="entity">
  <word form="names" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="object" stem="object-sg"  pos="N" class="entity">
  <word form="object" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="object" stem="object-pl"  pos="N" class="entity">
  <word form="objects" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="color" stem="color-sg"  pos="N" class="entity">
  <word form="color" macros="@pers.3rd"/>
  <word form="colour" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="color" stem="color-pl"  pos="N" class="entity">
  <word form="colors" macros="@pers.3rd"/>
  <word form="colours" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="shape" stem="shape-sg"  pos="N" class="entity">
  <word form="shape" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="shape" stem="shape-pl"  pos="N" class="entity">
  <word form="shapes" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="size" stem="size-sg"  pos="N" class="entity">
  <word form="size" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="size" stem="size-pl"  pos="N" class="entity">
  <word form="sizes" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="time" stem="time-sg"  pos="N" class="e-time">
  <word form="time" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="time" stem="time-pl"  pos="N" class="e-time">
  <word form="times" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="name" stem="name-sg"  pos="N" class="entity">
  <word form="name" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="name" stem="name-pl"  pos="N" class="entity">
  <word form="names" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="object" stem="object-sg"  pos="N" class="entity">
  <word form="object" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="object" stem="object-pl"  pos="N" class="entity">
  <word form="objects" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<entry pred="color" stem="color-sg"  pos="N" class="entity">
  <word form="color" macros="@pers.3rd"/>
  <word form="colour" macros="@pers.3rd"/>
  <member-of family="noun"/>
</entry>
<entry pred="color" stem="color-pl"  pos="N" class="entity">
  <word form="colors" macros="@pers.3rd"/>
  <word form="colours" macros="@pers.3rd"/>
  <member-of family="noun.plural"/>
</entry>

<!-- ++++++++++++++++++++++++++++++++++++++++++++++++ 
     +  ADV
     ++++++++++++++++++++++++++++++++++++++++++++++++ -->


<entry pred="there" stem="there__m-location"  pos="ADV" class="m-location">
  <word form="there"/>
  <member-of family="adv.location"/>
</entry>

<entry pred="here" stem="here__m-location"  pos="ADV" class="m-location">
  <word form="here"/>
  <member-of family="adv.location"/>
</entry>

<entry pred="now" stem="now__m-time"  pos="ADV" class="m-time">
  <word form="now"/>
  <member-of family="adv.time"/>
</entry>

<entry pred="today" stem="today__m-time"  pos="ADV" class="m-time">
  <word form="today"/>
  <member-of family="adv.time"/>
</entry>

<entry pred="yesterday" stem="yesterday__m-time"  pos="ADV" class="m-time">
  <word form="yesterday"/>
  <member-of family="adv.time"/>
</entry>

<entry pred="earlier" stem="earlier__m-time"  pos="ADV" class="m-time">
  <word form="earlier"/>
  <member-of family="adv.time"/>
</entry>

<entry pred="later" stem="later__m-time"  pos="ADV" class="m-time">
  <word form="later"/>
  <member-of family="adv.time"/>
</entry>

<entry pred="soon" stem="soon__m-time"  pos="ADV" class="m-time">
  <word form="soon"/>
  <member-of family="adv.time"/>
</entry>

<entry pred="still" stem="still__m-time"  pos="ADV" class="m-time">
  <word form="still"/>
  <member-of family="adv.time"/>
</entry>

<entry pred="already" stem="already__m-time"  pos="ADV" class="m-time">
  <word form="already"/>
  <member-of family="adv.time"/>
</entry>

<entry pred="yet" stem="yet__m-time"  pos="ADV" class="m-time">
  <word form="yet"/>
  <member-of family="adv.time"/>
</entry>

<entry pred="right" stem="right__m-direction"  pos="ADV" class="m-direction">
  <word form="right"/>
  <member-of family="adv.direction"/>
</entry>

<entry pred="left" stem="left__m-direction"  pos="ADV" class="m-direction">
  <word form="left"/>
  <member-of family="adv.direction"/>
</entry>

<entry pred="forward" stem="forward__m-direction"  pos="ADV" class="m-direction">
  <word form="forward"/>
  <word form="forwards"/>
  <member-of family="adv.direction"/>
</entry>

<entry pred="straight" stem="straight__m-direction"  pos="ADV" class="m-direction">
  <word form="straight"/>
  <word form="straightforward"/>
  <member-of family="adv.direction"/>
</entry>

<entry pred="backward" stem="backward__m-direction"  pos="ADV" class="m-direction">
  <word form="backward"/>
  <word form="back"/>
  <word form="backwards"/>
  <member-of family="adv.direction"/>
</entry>

<entry pred="around" stem="around__m-direction"  pos="ADV" class="m-direction">
  <word form="around"/>
  <member-of family="adv.direction"/>
</entry>

<entry pred="away" stem="away__m-direction"  pos="ADV" class="m-direction">
  <word form="away"/>
  <member-of family="adv.direction"/>
</entry>

<entry pred="close" stem="close__m-direction"  pos="ADV" class="m-direction">
  <word form="close"/>
  <member-of family="adv.direction"/>
</entry>

<entry pred="proximal" stem="proximal__m-direction"  pos="ADV" class="m-direction">
  <word form="proximal"/>
  <word form="up"/>
  <word form="right"/>
  <member-of family="adv.direction"/>
</entry>

<entry pred="there" stem="there__m-whereto"  pos="ADV" class="m-whereto">
  <word form="there"/>
  <member-of family="adv.whereto"/>
</entry>

<entry pred="here" stem="here__m-whereto"  pos="ADV" class="m-whereto">
  <word form="here"/>
  <member-of family="adv.whereto"/>
</entry>

<entry pred="quickly" stem="quickly__m-manner"  pos="ADV" class="m-manner">
  <word form="quickly"/>
  <word form="quick"/>
  <member-of family="adv.manner"/>
</entry>

<entry pred="slowly" stem="slowly__m-manner"  pos="ADV" class="m-manner">
  <word form="slowly"/>
  <word form="slow"/>
  <member-of family="adv.manner"/>
</entry>

<entry pred="over" stem="over__m-manner"  pos="ADV" class="m-manner">
  <word form="over"/>
  <member-of family="adv.manner"/>
</entry>

<entry pred="maybe" stem="maybe__m-probability"  pos="ADV" class="m-probability">
  <word form="maybe"/>
  <member-of family="adv.probability"/>
</entry>

<entry pred="probably" stem="probably__m-probability"  pos="ADV" class="m-probability">
  <word form="probably"/>
  <member-of family="adv.probability"/>
</entry>

<entry pred="perhaps" stem="perhaps__m-probability"  pos="ADV" class="m-probability">
  <word form="perhaps"/>
  <member-of family="adv.probability"/>
</entry>

<entry pred="certainly" stem="certainly__m-probability"  pos="ADV" class="m-probability">
  <word form="certainly"/>
  <member-of family="adv.probability"/>
</entry>

<entry pred="please" stem="please__m-comment"  pos="ADV" class="m-comment">
  <word form="please"/>
  <member-of family="adv.comment"/>
</entry>

<entry pred="kindly" stem="kindly__m-comment"  pos="ADV" class="m-comment">
  <word form="kindly"/>
  <member-of family="adv.comment"/>
</entry>

<entry pred="actually" stem="actually__m-comment"  pos="ADV" class="m-comment">
  <word form="actually"/>
  <member-of family="adv.comment"/>
</entry>

<entry pred="then" stem="then__m-comment"  pos="ADV" class="m-comment">
  <word form="then"/>
  <member-of family="adv.comment"/>
</entry>

<entry pred="really" stem="really__m-comment"  pos="ADV" class="m-comment">
  <word form="really"/>
  <member-of family="adv.comment"/>
</entry>

<entry pred="also" stem="also__m-comment"  pos="ADV" class="m-comment">
  <word form="also"/>
  <word form="too"/>
  <member-of family="adv.comment"/>
</entry>

<!-- ++++++++++++++++++++++++++++++++++++++++++++++++ 
     +  MARKER
     ++++++++++++++++++++++++++++++++++++++++++++++++ -->


<entry pred="if" stem="if__m-condition"  pos="MARKER" class="m-condition">
  <word form="if"/>
  <member-of family="dep-clause-marker.condition"/>
</entry>

<entry pred="because" stem="because__m-cause"  pos="MARKER" class="m-cause">
  <word form="because"/>
  <word form="cause"/>
  <word form="cuz"/>
  <member-of family="dep-clause-marker.cause"/>
</entry>

<entry pred="so" stem="so__m-result"  pos="MARKER" class="m-result">
  <word form="so"/>
  <member-of family="dep-clause-marker.result"/>
</entry>

<entry pred="to" stem="to__m-purpose"  pos="MARKER" class="m-purpose">
  <word form="to"/>
  <member-of family="dep-clause-marker.purpose"/>
</entry>

<!-- ++++++++++++++++++++++++++++++++++++++++++++++++ 
     +  V
     ++++++++++++++++++++++++++++++++++++++++++++++++ -->


<entry pred="stop" stem="stop__action-motion"  pos="V" class="action-motion">
  <word form="stop" macros="@nonfin"/>
  <word form="stop" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="stopped" macros="@vform.progr"/>
  <word form="stop" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="stop" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="stops" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="stopping" macros="@finite @past @ind"/>
  <member-of family="iv"/>
</entry>
<entry pred="stop" stem="stop-NO-SUBJ__action-motion"  pos="V" class="action-motion">
  <word form="stop" macros="@finite @pres @imp-addressee"/>
  <member-of family="iv.NO-SUBJ"/>
</entry>

<entry pred="go" stem="go__action-motion"  pos="V" class="action-motion">
  <word form="go" macros="@nonfin"/>
  <word form="go" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="going" macros="@vform.progr"/>
  <word form="go" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="go" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="goes" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="went" macros="@finite @past @ind"/>
  <member-of family="iv"/>
</entry>
<entry pred="go" stem="go-NO-SUBJ__action-motion"  pos="V" class="action-motion">
  <word form="go" macros="@finite @pres @imp-addressee"/>
  <member-of family="iv.NO-SUBJ"/>
</entry>

<entry pred="come" stem="come__action-motion"  pos="V" class="action-motion">
  <word form="come" macros="@nonfin"/>
  <word form="come" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="coming" macros="@vform.progr"/>
  <word form="come" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="come" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="comes" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="came" macros="@finite @past @ind"/>
  <member-of family="iv"/>
</entry>
<entry pred="come" stem="come-NO-SUBJ__action-motion"  pos="V" class="action-motion">
  <word form="come" macros="@finite @pres @imp-addressee"/>
  <member-of family="iv.NO-SUBJ"/>
</entry>

<entry pred="return" stem="return__action-motion"  pos="V" class="action-motion">
  <word form="return" macros="@nonfin"/>
  <word form="return" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="returning" macros="@vform.progr"/>
  <word form="return" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="return" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="returns" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="returned" macros="@finite @past @ind"/>
  <member-of family="iv"/>
</entry>
<entry pred="return" stem="return-NO-SUBJ__action-motion"  pos="V" class="action-motion">
  <word form="return" macros="@finite @pres @imp-addressee"/>
  <member-of family="iv.NO-SUBJ"/>
</entry>

<entry pred="walk" stem="walk__action-motion"  pos="V" class="action-motion">
  <word form="walk" macros="@nonfin"/>
  <word form="walk" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="walking" macros="@vform.progr"/>
  <word form="walk" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="walk" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="walks" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="walked" macros="@finite @past @ind"/>
  <member-of family="iv"/>
</entry>
<entry pred="walk" stem="walk-NO-SUBJ__action-motion"  pos="V" class="action-motion">
  <word form="walk" macros="@finite @pres @imp-addressee"/>
  <member-of family="iv.NO-SUBJ"/>
</entry>

<entry pred="turn" stem="turn__action-motion"  pos="V" class="action-motion">
  <word form="turn" macros="@nonfin"/>
  <word form="turn" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="turning" macros="@vform.progr"/>
  <word form="turn" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="turn" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="turns" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="turned" macros="@finite @past @ind"/>
  <member-of family="iv"/>
  <member-of family="tv"/>
</entry>
<entry pred="turn" stem="turn-NO-SUBJ__action-motion"  pos="V" class="action-motion">
  <word form="turn" macros="@finite @pres @imp-addressee"/>
  <member-of family="iv.NO-SUBJ"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="follow" stem="follow__action-motion"  pos="V" class="action-motion">
  <word form="follow" macros="@nonfin"/>
  <word form="follow" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="following" macros="@vform.progr"/>
  <word form="follow" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="follow" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="follows" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="followed" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="follow" stem="follow-NO-SUBJ__action-motion"  pos="V" class="action-motion">
  <word form="follow" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="run" stem="run__action-motion"  pos="V" class="action-motion">
  <word form="run" macros="@nonfin"/>
  <word form="run" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="running" macros="@vform.progr"/>
  <word form="run" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="run" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="runs" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="ran" macros="@finite @past @ind"/>
  <member-of family="iv"/>
</entry>
<entry pred="run" stem="run-NO-SUBJ__action-motion"  pos="V" class="action-motion">
  <word form="run" macros="@finite @pres @imp-addressee"/>
  <member-of family="iv.NO-SUBJ"/>
</entry>

<entry pred="move" stem="move__action-motion"  pos="V" class="action-motion">
  <word form="move" macros="@nonfin"/>
  <word form="move" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="moving" macros="@vform.progr"/>
  <word form="move" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="move" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="moves" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="moved" macros="@finite @past @ind"/>
  <member-of family="iv"/>
  <member-of family="tv"/>
</entry>
<entry pred="move" stem="move-NO-SUBJ__action-motion"  pos="V" class="action-motion">
  <word form="move" macros="@finite @pres @imp-addressee"/>
  <member-of family="iv.NO-SUBJ"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="kick" stem="kick__action-motion"  pos="V" class="action-motion">
  <word form="kick" macros="@nonfin"/>
  <word form="kick" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="kicking" macros="@vform.progr"/>
  <word form="kick" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="kick" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="kicks" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="kicked" macros="@finite @past @ind"/>
  <member-of family="tv"/>
  <member-of family="dtv"/>
  <member-of family="dtv.to-indobj"/>
</entry>
<entry pred="kick" stem="kick-NO-SUBJ__action-motion"  pos="V" class="action-motion">
  <word form="kick" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
  <member-of family="dtv.NO-SUBJ"/>
  <member-of family="dtv.to-indobj.NO-SUBJ"/>
</entry>

<entry pred="put" stem="put__action-motion"  pos="V" class="action-motion">
  <word form="put" macros="@nonfin"/>
  <word form="put" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="putting" macros="@vform.progr"/>
  <word form="put" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="put" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="puts" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="put" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="put" stem="put-NO-SUBJ__action-motion"  pos="V" class="action-motion">
  <word form="put" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="get" stem="get__action-motion"  pos="V" class="action-motion">
  <word form="get" macros="@nonfin"/>
  <word form="get" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="getting" macros="@vform.progr"/>
  <word form="get" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="get" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="gets" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="got" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="get" stem="get-NO-SUBJ__action-motion"  pos="V" class="action-motion">
  <word form="get" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="take" stem="take__action-motion"  pos="V" class="action-motion">
  <word form="take" macros="@nonfin"/>
  <word form="take" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="taking" macros="@vform.progr"/>
  <word form="take" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="take" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="takes" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="took" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="take" stem="take-NO-SUBJ__action-motion"  pos="V" class="action-motion">
  <word form="take" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="bring" stem="bring__action-motion"  pos="V" class="action-motion">
  <word form="bring" macros="@nonfin"/>
  <word form="bring" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="bringing" macros="@vform.progr"/>
  <word form="bring" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="bring" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="brings" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="brought" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="bring" stem="bring-NO-SUBJ__action-motion"  pos="V" class="action-motion">
  <word form="bring" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="myput" stem="myput__action-directed-motion"  pos="V" class="action-directed-motion">
  <word form="myput" macros="@nonfin"/>
  <word form="myput" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="myputing" macros="@vform.progr"/>
  <word form="myput" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="myput" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="myputs" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="myputed" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="myput" stem="myput-NO-SUBJ__action-directed-motion"  pos="V" class="action-directed-motion">
  <word form="myput" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="explore" stem="explore__action-non-motion"  pos="V" class="action-non-motion">
  <word form="explore" macros="@nonfin"/>
  <word form="explore" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="exploring" macros="@vform.progr"/>
  <word form="explore" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="explore" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="explores" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="explored" macros="@finite @past @ind"/>
  <word form="investigate" macros="@nonfin"/>
  <word form="investigate" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="investigating" macros="@vform.progr"/>
  <word form="investigate" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="investigate" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="investigates" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="investigated" macros="@finite @past @ind"/>
  <word form="survey" macros="@nonfin"/>
  <word form="survey" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="surveying" macros="@vform.progr"/>
  <word form="survey" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="survey" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="surveys" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="surveyed" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="explore" stem="explore-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="explore" macros="@finite @pres @imp-addressee"/>
  <word form="investigate" macros="@finite @pres @imp-addressee"/>
  <word form="survey" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="present" stem="present__action-non-motion"  pos="V" class="action-non-motion">
  <word form="present" macros="@nonfin"/>
  <word form="present" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="presenting" macros="@vform.progr"/>
  <word form="present" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="present" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="presents" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="presented" macros="@finite @past @ind"/>
  <member-of family="dtv"/>
  <member-of family="dtv.to-indobj"/>
</entry>
<entry pred="present" stem="present-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="present" macros="@finite @pres @imp-addressee"/>
  <member-of family="dtv.NO-SUBJ"/>
  <member-of family="dtv.to-indobj.NO-SUBJ"/>
</entry>

<entry pred="give" stem="give__action-non-motion"  pos="V" class="action-non-motion">
  <word form="give" macros="@nonfin"/>
  <word form="give" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="giving" macros="@vform.progr"/>
  <word form="give" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="give" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="gives" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="gave" macros="@finite @past @ind"/>
  <member-of family="dtv"/>
  <member-of family="dtv.to-indobj"/>
</entry>
<entry pred="give" stem="give-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="give" macros="@finite @pres @imp-addressee"/>
  <member-of family="dtv.NO-SUBJ"/>
  <member-of family="dtv.to-indobj.NO-SUBJ"/>
</entry>

<entry pred="pick" stem="pick__action-non-motion"  pos="V" class="action-non-motion">
  <word form="pick" macros="@nonfin"/>
  <word form="pick" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="picking" macros="@vform.progr"/>
  <word form="pick" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="pick" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="picks" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="picked" macros="@finite @past @ind"/>
  <member-of family="tv.prt"/>
</entry>
<entry pred="pick" stem="pick-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="pick" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.prt.NO-SUBJ"/>
</entry>

<entry pred="play" stem="play__action-non-motion"  pos="V" class="action-non-motion">
  <word form="play" macros="@nonfin"/>
  <word form="play" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="playing" macros="@vform.progr"/>
  <word form="play" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="play" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="plays" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="played" macros="@finite @past @ind"/>
  <member-of family="tv.with-obj"/>
</entry>
<entry pred="play" stem="play-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="play" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.with-obj.NO-SUBJ"/>
</entry>

<entry pred="win" stem="win__action-non-motion"  pos="V" class="action-non-motion">
  <word form="win" macros="@nonfin"/>
  <word form="win" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="winning" macros="@vform.progr"/>
  <word form="win" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="win" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="wins" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="won" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="win" stem="win-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="win" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="start" stem="start__action-non-motion"  pos="V" class="action-non-motion">
  <word form="start" macros="@nonfin"/>
  <word form="start" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="starting" macros="@vform.progr"/>
  <word form="start" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="start" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="starts" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="started" macros="@finite @past @ind"/>
  <member-of family="tv"/>
  <member-of family="tv.with-obj"/>
</entry>
<entry pred="start" stem="start-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="start" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
  <member-of family="tv.with-obj.NO-SUBJ"/>
</entry>

<entry pred="go" stem="go__action-non-motion"  pos="V" class="action-non-motion">
  <word form="go" macros="@nonfin"/>
  <word form="go" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="going" macros="@vform.progr"/>
  <word form="go" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="go" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="goes" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="went" macros="@finite @past @ind"/>
  <member-of family="iv"/>
</entry>
<entry pred="go" stem="go-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="go" macros="@finite @pres @imp-addressee"/>
  <member-of family="iv.NO-SUBJ"/>
</entry>

<entry pred="begin" stem="begin__action-non-motion"  pos="V" class="action-non-motion">
  <word form="begin" macros="@nonfin"/>
  <word form="begin" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="beginning" macros="@vform.progr"/>
  <word form="begin" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="begin" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="begins" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="began" macros="@finite @past @ind"/>
  <member-of family="tv"/>
  <member-of family="tv.with-obj"/>
</entry>
<entry pred="begin" stem="begin-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="begin" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
  <member-of family="tv.with-obj.NO-SUBJ"/>
</entry>

<entry pred="stop" stem="stop__action-non-motion"  pos="V" class="action-non-motion">
  <word form="stop" macros="@nonfin"/>
  <word form="stop" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="stopping" macros="@vform.progr"/>
  <word form="stop" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="stop" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="stops" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="stopped" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="stop" stem="stop-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="stop" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="bring" stem="bring__action-non-motion"  pos="V" class="action-non-motion">
  <word form="bring" macros="@nonfin"/>
  <word form="bring" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="bringing" macros="@vform.progr"/>
  <word form="bring" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="bring" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="brings" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="brung" macros="@finite @past @ind"/>
  <member-of family="dtv"/>
  <member-of family="dtv.to-indobj"/>
</entry>
<entry pred="bring" stem="bring-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="bring" macros="@finite @pres @imp-addressee"/>
  <member-of family="dtv.NO-SUBJ"/>
  <member-of family="dtv.to-indobj.NO-SUBJ"/>
</entry>

<entry pred="get" stem="get__action-non-motion"  pos="V" class="action-non-motion">
  <word form="get" macros="@nonfin"/>
  <word form="get" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="getting" macros="@vform.progr"/>
  <word form="get" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="get" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="gets" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="got" macros="@finite @past @ind"/>
  <member-of family="tv"/>
  <member-of family="dtv"/>
  <member-of family="dtv.to-indobj"/>
  <member-of family="v.adj-comp"/>
</entry>
<entry pred="get" stem="get-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="get" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
  <member-of family="dtv.NO-SUBJ"/>
  <member-of family="dtv.to-indobj.NO-SUBJ"/>
  <member-of family="v.adj-comp.NO-SUBJ"/>
</entry>

<entry pred="brush" stem="brush__action-non-motion"  pos="V" class="action-non-motion">
  <word form="brush" macros="@nonfin"/>
  <word form="brush" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="brushing" macros="@vform.progr"/>
  <word form="brush" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="brush" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="brushs" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="brushed" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="brush" stem="brush-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="brush" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="stand" stem="stand__action-non-motion"  pos="V" class="action-non-motion">
  <word form="stand" macros="@nonfin"/>
  <word form="stand" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="standing" macros="@vform.progr"/>
  <word form="stand" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="stand" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="stands" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="stood" macros="@finite @past @ind"/>
  <member-of family="iv"/>
</entry>
<entry pred="stand" stem="stand-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="stand" macros="@finite @pres @imp-addressee"/>
  <member-of family="iv.NO-SUBJ"/>
</entry>

<entry pred="show" stem="show__action-non-motion"  pos="V" class="action-non-motion">
  <word form="show" macros="@nonfin"/>
  <word form="show" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="showing" macros="@vform.progr"/>
  <word form="show" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="show" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="shows" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="showed" macros="@finite @past @ind"/>
  <member-of family="tv"/>
  <member-of family="dtv"/>
  <member-of family="dtv.to-indobj"/>
</entry>
<entry pred="show" stem="show-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="show" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
  <member-of family="dtv.NO-SUBJ"/>
  <member-of family="dtv.to-indobj.NO-SUBJ"/>
</entry>

<entry pred="have" stem="have__action-non-motion"  pos="V" class="action-non-motion">
  <word form="have" macros="@nonfin"/>
  <word form="have" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="having" macros="@vform.progr"/>
  <word form="have" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="have" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="has" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="had" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="have" stem="have-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="have" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="break" stem="break__action-non-motion"  pos="V" class="action-non-motion">
  <word form="break" macros="@nonfin"/>
  <word form="break" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="breaking" macros="@vform.progr"/>
  <word form="break" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="break" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="breaks" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="broke" macros="@finite @past @ind"/>
  <member-of family="iv"/>
  <member-of family="tv"/>
</entry>
<entry pred="break" stem="break-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="break" macros="@finite @pres @imp-addressee"/>
  <member-of family="iv.NO-SUBJ"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="find" stem="find__action-non-motion"  pos="V" class="action-non-motion">
  <word form="find" macros="@nonfin"/>
  <word form="find" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="finding" macros="@vform.progr"/>
  <word form="find" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="find" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="finds" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="found" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="find" stem="find-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="find" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="eat" stem="eat__action-non-motion"  pos="V" class="action-non-motion">
  <word form="eat" macros="@nonfin"/>
  <word form="eat" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="eating" macros="@vform.progr"/>
  <word form="eat" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="eat" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="eats" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="ate" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="eat" stem="eat-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="eat" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="drink" stem="drink__action-non-motion"  pos="V" class="action-non-motion">
  <word form="drink" macros="@nonfin"/>
  <word form="drink" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="drinking" macros="@vform.progr"/>
  <word form="drink" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="drink" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="drinks" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="drank" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="drink" stem="drink-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="drink" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="make" stem="make__action-non-motion"  pos="V" class="action-non-motion">
  <word form="make" macros="@nonfin"/>
  <word form="make" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="making" macros="@vform.progr"/>
  <word form="make" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="make" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="makes" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="made" macros="@finite @past @ind"/>
  <member-of family="tv"/>
  <member-of family="dtv"/>
  <member-of family="resultant-verb"/>
</entry>
<entry pred="make" stem="make-NO-SUBJ__action-non-motion"  pos="V" class="action-non-motion">
  <word form="make" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
  <member-of family="dtv.NO-SUBJ"/>
  <member-of family="resultant-verb.NO-SUBJ"/>
</entry>

<entry pred="want" stem="want__cognition"  pos="V" class="cognition">
  <word form="want" macros="@nonfin"/>
  <word form="want" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="wanting" macros="@vform.progr"/>
  <word form="want" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="want" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="wants" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="wanted" macros="@finite @past @ind"/>
  <member-of family="tv"/>
  <member-of family="v.subj-controlled-inf-verb-comp"/>
  <member-of family="v.obj-controlled-inf-verb-comp"/>
</entry>
<entry pred="want" stem="want-NO-SUBJ__cognition"  pos="V" class="cognition">
  <word form="want" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
  <member-of family="v.subj-controlled-inf-verb-comp.NO-SUBJ"/>
  <member-of family="v.obj-controlled-inf-verb-comp.NO-SUBJ"/>
</entry>

<entry pred="need" stem="need__cognition"  pos="V" class="cognition">
  <word form="need" macros="@nonfin"/>
  <word form="need" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="needing" macros="@vform.progr"/>
  <word form="need" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="need" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="needs" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="needed" macros="@finite @past @ind"/>
  <member-of family="tv"/>
  <member-of family="v.subj-controlled-inf-verb-comp"/>
  <member-of family="v.obj-controlled-inf-verb-comp"/>
</entry>
<entry pred="need" stem="need-NO-SUBJ__cognition"  pos="V" class="cognition">
  <word form="need" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
  <member-of family="v.subj-controlled-inf-verb-comp.NO-SUBJ"/>
  <member-of family="v.obj-controlled-inf-verb-comp.NO-SUBJ"/>
</entry>

<entry pred="like" stem="like__cognition"  pos="V" class="cognition">
  <word form="like" macros="@nonfin"/>
  <word form="like" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="liking" macros="@vform.progr"/>
  <word form="like" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="like" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="likes" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="liked" macros="@finite @past @ind"/>
  <member-of family="tv"/>
  <member-of family="v.subj-controlled-inf-verb-comp"/>
  <member-of family="v.obj-controlled-inf-verb-comp"/>
</entry>
<entry pred="like" stem="like-NO-SUBJ__cognition"  pos="V" class="cognition">
  <word form="like" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
  <member-of family="v.subj-controlled-inf-verb-comp.NO-SUBJ"/>
  <member-of family="v.obj-controlled-inf-verb-comp.NO-SUBJ"/>
</entry>

<entry pred="have" stem="have__cognition"  pos="V" class="cognition">
  <word form="have" macros="@nonfin"/>
  <word form="have" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="having" macros="@vform.progr"/>
  <word form="have" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="have" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="has" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="had" macros="@finite @past @ind"/>
  <member-of family="v.subj-controlled-inf-verb-comp"/>
</entry>
<entry pred="have" stem="have-NO-SUBJ__cognition"  pos="V" class="cognition">
  <word form="have" macros="@finite @pres @imp-addressee"/>
  <member-of family="v.subj-controlled-inf-verb-comp.NO-SUBJ"/>
</entry>

<entry pred="help" stem="help__cognition"  pos="V" class="cognition">
  <word form="help" macros="@nonfin"/>
  <word form="help" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="helping" macros="@vform.progr"/>
  <word form="help" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="help" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="helps" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="helped" macros="@finite @past @ind"/>
  <member-of family="tv"/>
  <member-of family="v.obj-controlled-non-fin-verb-comp"/>
</entry>
<entry pred="help" stem="help-NO-SUBJ__cognition"  pos="V" class="cognition">
  <word form="help" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
  <member-of family="v.obj-controlled-non-fin-verb-comp.NO-SUBJ"/>
</entry>

<entry pred="hate" stem="hate__cognition"  pos="V" class="cognition">
  <word form="hate" macros="@nonfin"/>
  <word form="hate" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="hating" macros="@vform.progr"/>
  <word form="hate" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="hate" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="hates" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="hated" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="hate" stem="hate-NO-SUBJ__cognition"  pos="V" class="cognition">
  <word form="hate" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="love" stem="love__cognition"  pos="V" class="cognition">
  <word form="love" macros="@nonfin"/>
  <word form="love" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="loving" macros="@vform.progr"/>
  <word form="love" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="love" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="loves" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="loved" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="love" stem="love-NO-SUBJ__cognition"  pos="V" class="cognition">
  <word form="love" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="suppose" stem="suppose__cognition"  pos="V" class="cognition">
  <word form="suppose" macros="@nonfin"/>
  <word form="suppose" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="supposing" macros="@vform.progr"/>
  <word form="suppose" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="suppose" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="supposes" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="supposed" macros="@finite @past @ind"/>
  <member-of family="v.sentence-comp"/>
</entry>
<entry pred="suppose" stem="suppose-NO-SUBJ__cognition"  pos="V" class="cognition">
  <word form="suppose" macros="@finite @pres @imp-addressee"/>
  <member-of family="v.sentence-comp.NO-SUBJ"/>
</entry>

<entry pred="think" stem="think__cognition"  pos="V" class="cognition">
  <word form="think" macros="@nonfin"/>
  <word form="think" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="thinking" macros="@vform.progr"/>
  <word form="think" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="think" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="thinks" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="thought" macros="@finite @past @ind"/>
  <member-of family="v.sentence-comp"/>
</entry>
<entry pred="think" stem="think-NO-SUBJ__cognition"  pos="V" class="cognition">
  <word form="think" macros="@finite @pres @imp-addressee"/>
  <member-of family="v.sentence-comp.NO-SUBJ"/>
</entry>

<entry pred="know" stem="know__cognition"  pos="V" class="cognition">
  <word form="know" macros="@nonfin"/>
  <word form="know" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="knowing" macros="@vform.progr"/>
  <word form="know" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="know" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="knows" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="knew" macros="@finite @past @ind"/>
  <member-of family="tv"/>
  <member-of family="v.sentence-comp"/>
</entry>
<entry pred="know" stem="know-NO-SUBJ__cognition"  pos="V" class="cognition">
  <word form="know" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
  <member-of family="v.sentence-comp.NO-SUBJ"/>
</entry>

<entry pred="feel" stem="feel__cognition"  pos="V" class="cognition">
  <word form="feel" macros="@nonfin"/>
  <word form="feel" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="feeling" macros="@vform.progr"/>
  <word form="feel" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="feel" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="feels" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="felt" macros="@finite @past @ind"/>
  <member-of family="v.adj-comp"/>
</entry>
<entry pred="feel" stem="feel-NO-SUBJ__cognition"  pos="V" class="cognition">
  <word form="feel" macros="@finite @pres @imp-addressee"/>
  <member-of family="v.adj-comp.NO-SUBJ"/>
</entry>

<entry pred="see" stem="see__perception"  pos="V" class="perception">
  <word form="see" macros="@nonfin"/>
  <word form="see" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="seeing" macros="@vform.progr"/>
  <word form="see" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="see" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="sees" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="saw" macros="@finite @past @ind"/>
  <member-of family="iv"/>
  <member-of family="tv"/>
</entry>
<entry pred="see" stem="see-NO-SUBJ__perception"  pos="V" class="perception">
  <word form="see" macros="@finite @pres @imp-addressee"/>
  <member-of family="iv.NO-SUBJ"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="watch" stem="watch__perception"  pos="V" class="perception">
  <word form="watch" macros="@nonfin"/>
  <word form="watch" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="watching" macros="@vform.progr"/>
  <word form="watch" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="watch" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="watches" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="watched" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="watch" stem="watch-NO-SUBJ__perception"  pos="V" class="perception">
  <word form="watch" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="look" stem="look__perception"  pos="V" class="perception">
  <word form="look" macros="@nonfin"/>
  <word form="look" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="looking" macros="@vform.progr"/>
  <word form="look" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="look" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="looks" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="looked" macros="@finite @past @ind"/>
  <member-of family="tv.at-obj"/>
</entry>
<entry pred="look" stem="look-NO-SUBJ__perception"  pos="V" class="perception">
  <word form="look" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.at-obj.NO-SUBJ"/>
</entry>

<entry pred="listen" stem="listen__perception"  pos="V" class="perception">
  <word form="listen" macros="@nonfin"/>
  <word form="listen" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="listening" macros="@vform.progr"/>
  <word form="listen" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="listen" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="listens" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="listened" macros="@finite @past @ind"/>
  <member-of family="tv.to-obj"/>
</entry>
<entry pred="listen" stem="listen-NO-SUBJ__perception"  pos="V" class="perception">
  <word form="listen" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.to-obj.NO-SUBJ"/>
</entry>

<entry pred="tell" stem="tell__communication"  pos="V" class="communication">
  <word form="tell" macros="@nonfin"/>
  <word form="tell" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="telling" macros="@vform.progr"/>
  <word form="tell" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="tell" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="tells" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="told" macros="@finite @past @ind"/>
  <member-of family="tv"/>
  <member-of family="v.obj-controlled-inf-verb-comp"/>
  <member-of family="v.obj.sentence-comp"/>
  <member-of family="dtv"/>
  <member-of family="dtv.to-indobj"/>
</entry>
<entry pred="tell" stem="tell-NO-SUBJ__communication"  pos="V" class="communication">
  <word form="tell" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
  <member-of family="v.obj-controlled-inf-verb-comp.NO-SUBJ"/>
  <member-of family="v.obj.sentence-comp.NO-SUBJ"/>
  <member-of family="dtv.NO-SUBJ"/>
  <member-of family="dtv.to-indobj.NO-SUBJ"/>
</entry>

<entry pred="say" stem="say__communication"  pos="V" class="communication">
  <word form="say" macros="@nonfin"/>
  <word form="say" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="saying" macros="@vform.progr"/>
  <word form="say" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="say" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="says" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="said" macros="@finite @past @ind"/>
  <member-of family="v.sentence-comp"/>
</entry>
<entry pred="say" stem="say-NO-SUBJ__communication"  pos="V" class="communication">
  <word form="say" macros="@finite @pres @imp-addressee"/>
  <member-of family="v.sentence-comp.NO-SUBJ"/>
</entry>

<entry pred="explain" stem="explain__communication"  pos="V" class="communication">
  <word form="explain" macros="@nonfin"/>
  <word form="explain" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="explaining" macros="@vform.progr"/>
  <word form="explain" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="explain" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="explains" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="explained" macros="@finite @past @ind"/>
  <member-of family="tv"/>
  <member-of family="dtv"/>
  <member-of family="dtv.to-indobj"/>
</entry>
<entry pred="explain" stem="explain-NO-SUBJ__communication"  pos="V" class="communication">
  <word form="explain" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
  <member-of family="dtv.NO-SUBJ"/>
  <member-of family="dtv.to-indobj.NO-SUBJ"/>
</entry>

<entry pred="talk" stem="talk__communication"  pos="V" class="communication">
  <word form="talk" macros="@nonfin"/>
  <word form="talk" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="talking" macros="@vform.progr"/>
  <word form="talk" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="talk" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="talks" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="talked" macros="@finite @past @ind"/>
  <member-of family="iv"/>
  <member-of family="tv.to-obj"/>
  <member-of family="tv.with-obj"/>
</entry>
<entry pred="talk" stem="talk-NO-SUBJ__communication"  pos="V" class="communication">
  <word form="talk" macros="@finite @pres @imp-addressee"/>
  <member-of family="iv.NO-SUBJ"/>
  <member-of family="tv.to-obj.NO-SUBJ"/>
  <member-of family="tv.with-obj.NO-SUBJ"/>
</entry>

<entry pred="speak" stem="speak__communication"  pos="V" class="communication">
  <word form="speak" macros="@nonfin"/>
  <word form="speak" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="speaking" macros="@vform.progr"/>
  <word form="speak" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="speak" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="speaks" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="spoke" macros="@finite @past @ind"/>
  <member-of family="iv"/>
  <member-of family="tv.to-obj"/>
  <member-of family="tv.with-obj"/>
</entry>
<entry pred="speak" stem="speak-NO-SUBJ__communication"  pos="V" class="communication">
  <word form="speak" macros="@finite @pres @imp-addressee"/>
  <member-of family="iv.NO-SUBJ"/>
  <member-of family="tv.to-obj.NO-SUBJ"/>
  <member-of family="tv.with-obj.NO-SUBJ"/>
</entry>

<entry pred="call" stem="call__symbolic"  pos="V" class="symbolic">
  <word form="call" macros="@nonfin"/>
  <word form="call" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="calling" macros="@vform.progr"/>
  <word form="call" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="call" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="calls" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="called" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="call" stem="call-NO-SUBJ__symbolic"  pos="V" class="symbolic">
  <word form="call" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<entry pred="name" stem="name__symbolic"  pos="V" class="symbolic">
  <word form="name" macros="@nonfin"/>
  <word form="name" macros="@finite @pres @imp @pers.3rd-agr"/>
  <word form="naming" macros="@vform.progr"/>
  <word form="name" macros="@finite @pres @ind @pers.non-3rd-agr @num.sg-agr "/>
  <word form="name" macros="@finite @pres @ind @num.pl-agr "/>
  <word form="names" macros="@finite @pres @ind @pers.3rd-agr @num.sg-agr"/>
  <word form="named" macros="@finite @past @ind"/>
  <member-of family="tv"/>
</entry>
<entry pred="name" stem="name-NO-SUBJ__symbolic"  pos="V" class="symbolic">
  <word form="name" macros="@finite @pres @imp-addressee"/>
  <member-of family="tv.NO-SUBJ"/>
</entry>

<!-- ++++++++++++++++++++++++++++++++++++++++++++++++ 
     +  PREP
     ++++++++++++++++++++++++++++++++++++++++++++++++ -->


<entry pred="at" stem="at__m-location"  pos="PREP" class="m-location">
  <word form="at"/>
  <member-of family="prep-mod-n.location"/>
  <member-of family="prep-mod-s.location"/>
</entry>

<entry pred="around" stem="around__m-location"  pos="PREP" class="m-location">
  <word form="around"/>
  <member-of family="prep-mod-n.location"/>
  <member-of family="prep-mod-s.location"/>
</entry>

<entry pred="beside" stem="beside__m-location"  pos="PREP" class="m-location">
  <word form="beside"/>
  <member-of family="prep-mod-n.location"/>
  <member-of family="prep-mod-s.location"/>
</entry>

<entry pred="near" stem="near__m-location"  pos="PREP" class="m-location">
  <word form="near"/>
  <member-of family="prep-mod-n.location"/>
  <member-of family="prep-mod-s.location"/>
</entry>

<entry pred="on" stem="on__m-location"  pos="PREP" class="m-location">
  <word form="on"/>
  <member-of family="prep-mod-n.location"/>
  <member-of family="prep-mod-s.location"/>
</entry>

<entry pred="in" stem="in__m-location"  pos="PREP" class="m-location">
  <word form="in"/>
  <word form="inside"/>
  <member-of family="prep-mod-n.location"/>
  <member-of family="prep-mod-s.location"/>
</entry>

<entry pred="out" stem="out__m-location"  pos="PREP" class="m-location">
  <word form="out"/>
  <word form="outside"/>
  <member-of family="prep-mod-n.location"/>
  <member-of family="prep-mod-s.location"/>
</entry>

<entry pred="over" stem="over__m-location"  pos="PREP" class="m-location">
  <word form="over"/>
  <word form="above"/>
  <member-of family="prep-mod-n.location"/>
  <member-of family="prep-mod-s.location"/>
</entry>

<entry pred="under" stem="under__m-location"  pos="PREP" class="m-location">
  <word form="under"/>
  <word form="underneath"/>
  <word form="beneath"/>
  <word form="below"/>
  <member-of family="prep-mod-n.location"/>
  <member-of family="prep-mod-s.location"/>
</entry>

<entry pred="by" stem="by__m-location"  pos="PREP" class="m-location">
  <word form="by"/>
  <member-of family="prep-mod-n.location"/>
  <member-of family="prep-mod-s.location"/>
</entry>

<entry pred="behind" stem="behind__m-location"  pos="PREP" class="m-location">
  <word form="behind"/>
  <member-of family="prep-mod-n.location"/>
  <member-of family="prep-mod-s.location"/>
</entry>

<entry pred="up" stem="up__m-location"  pos="PREP" class="m-location">
  <word form="up"/>
  <member-of family="prep-mod-n.location"/>
  <member-of family="prep-mod-s.location"/>
</entry>

<entry pred="down" stem="down__m-location"  pos="PREP" class="m-location">
  <word form="down"/>
  <member-of family="prep-mod-n.location"/>
  <member-of family="prep-mod-s.location"/>
</entry>

<entry pred="along" stem="along__m-location"  pos="PREP" class="m-location">
  <word form="along"/>
  <word form="alongside"/>
  <member-of family="prep-mod-n.location"/>
  <member-of family="prep-mod-s.location"/>
</entry>

<entry pred="opposite" stem="opposite__m-location"  pos="PREP" class="m-location">
  <word form="opposite"/>
  <member-of family="prep-mod-n.location"/>
  <member-of family="prep-mod-s.location"/>
</entry>

<entry pred="between" stem="between__m-location"  pos="PREP" class="m-location">
  <word form="between"/>
  <member-of family="prep-mod-n.location"/>
  <member-of family="prep-mod-s.location"/>
</entry>

<entry pred="right" stem="right-of__m-location"  pos="PREP" class="m-location">
  <word form="right"/>
  <member-of family="prep-mod-n.of-np.location"/>
  <member-of family="prep-mod-s.of-np.location"/>
</entry>

<entry pred="left" stem="left-of__m-location"  pos="PREP" class="m-location">
  <word form="left"/>
  <member-of family="prep-mod-n.of-np.location"/>
  <member-of family="prep-mod-s.of-np.location"/>
</entry>

<entry pred="from" stem="from__m-location"  pos="PREP" class="m-location">
  <word form="from"/>
  <member-of family="prep-mod-n.location"/>
</entry>

<entry pred="to" stem="to__m-whereto"  pos="PREP" class="m-whereto">
  <word form="to"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="into" stem="into__m-whereto"  pos="PREP" class="m-whereto">
  <word form="into"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="onto" stem="onto__m-whereto"  pos="PREP" class="m-whereto">
  <word form="onto"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="toward" stem="toward__m-whereto"  pos="PREP" class="m-whereto">
  <word form="toward"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="around" stem="around__m-whereto"  pos="PREP" class="m-whereto">
  <word form="around"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="beside" stem="beside__m-whereto"  pos="PREP" class="m-whereto">
  <word form="beside"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="near" stem="near__m-whereto"  pos="PREP" class="m-whereto">
  <word form="near"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="on" stem="on__m-whereto"  pos="PREP" class="m-whereto">
  <word form="on"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="in" stem="in__m-whereto"  pos="PREP" class="m-whereto">
  <word form="in"/>
  <word form="inside"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="out" stem="out__m-whereto"  pos="PREP" class="m-whereto">
  <word form="out"/>
  <word form="outside"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="over" stem="over__m-whereto"  pos="PREP" class="m-whereto">
  <word form="over"/>
  <word form="above"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="under" stem="under__m-whereto"  pos="PREP" class="m-whereto">
  <word form="under"/>
  <word form="underneath"/>
  <word form="beneath"/>
  <word form="below"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="by" stem="by__m-whereto"  pos="PREP" class="m-whereto">
  <word form="by"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="behind" stem="behind__m-whereto"  pos="PREP" class="m-whereto">
  <word form="behind"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="up" stem="up__m-whereto"  pos="PREP" class="m-whereto">
  <word form="up"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="down" stem="down__m-whereto"  pos="PREP" class="m-whereto">
  <word form="down"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="along" stem="along__m-whereto"  pos="PREP" class="m-whereto">
  <word form="along"/>
  <word form="alongside"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="opposite" stem="opposite__m-whereto"  pos="PREP" class="m-whereto">
  <word form="opposite"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="between" stem="between__m-whereto"  pos="PREP" class="m-whereto">
  <word form="between"/>
  <member-of family="prep-mod-s.whereto"/>
</entry>

<entry pred="right" stem="right-of__m-whereto"  pos="PREP" class="m-whereto">
  <word form="right"/>
  <member-of family="prep-mod-s.of-np.whereto"/>
</entry>

<entry pred="left" stem="left-of__m-whereto"  pos="PREP" class="m-whereto">
  <word form="left"/>
  <member-of family="prep-mod-s.of-np.whereto"/>
</entry>

<entry pred="from" stem="from__m-wherefrom"  pos="PREP" class="m-wherefrom">
  <word form="from"/>
  <member-of family="prep-mod-s.wherefrom"/>
</entry>

<entry pred="off" stem="off__m-wherefrom"  pos="PREP" class="m-wherefrom">
  <word form="off"/>
  <member-of family="prep-mod-s.wherefrom"/>
</entry>

<entry pred="across" stem="across__m-through"  pos="PREP" class="m-through">
  <word form="across"/>
  <member-of family="prep-mod-s.through"/>
</entry>

<entry pred="through" stem="through__m-through"  pos="PREP" class="m-through">
  <word form="through"/>
  <member-of family="prep-mod-s.through"/>
</entry>

<entry pred="with" stem="with__m-accompaniment"  pos="PREP" class="m-accompaniment">
  <word form="with"/>
  <member-of family="prep-mod-n.accompaniment"/>
  <member-of family="prep-mod-s.accompaniment"/>
</entry>

<entry pred="for" stem="for__m-benefactor"  pos="PREP" class="m-benefactor">
  <word form="for"/>
  <member-of family="prep-mod-n.benefactor"/>
  <member-of family="prep-mod-s.benefactor"/>
</entry>

<entry pred="like" stem="like__m-comparison"  pos="PREP" class="m-comparison">
  <word form="like"/>
  <member-of family="prep-mod-n.comparison"/>
  <member-of family="prep-mod-s.comparison"/>
</entry>

<!-- ++++++++++++++++++++++++++++++++++++++++++++++++ 
     +  DU
     ++++++++++++++++++++++++++++++++++++++++++++++++ -->
















<!-- ++++++++++++++++++++++++++++++++++++++++++++++++ 
     +  WORD
     ++++++++++++++++++++++++++++++++++++++++++++++++ -->


<entry pred="look" stem="look"  pos="WORD" class="blah">
  <word form="look"/>
</entry>
<macro name="@mwe1-look"> <fs id="51"><feat attr="wform" val="look"/> </fs> </macro>
<macro name="@mwe2-look"> <fs id="52"><feat attr="wform" val="look"/> </fs> </macro>
<macro name="@mwe3-look"> <fs id="53"><feat attr="wform" val="look"/> </fs> </macro>

<entry pred="a" stem="a"  pos="WORD" class="blah">
  <word form="a"/>
</entry>
<macro name="@mwe1-a"> <fs id="51"><feat attr="wform" val="a"/> </fs> </macro>
<macro name="@mwe2-a"> <fs id="52"><feat attr="wform" val="a"/> </fs> </macro>
<macro name="@mwe3-a"> <fs id="53"><feat attr="wform" val="a"/> </fs> </macro>

<entry pred="think" stem="think"  pos="WORD" class="blah">
  <word form="think"/>
</entry>
<macro name="@mwe1-think"> <fs id="51"><feat attr="wform" val="think"/> </fs> </macro>
<macro name="@mwe2-think"> <fs id="52"><feat attr="wform" val="think"/> </fs> </macro>
<macro name="@mwe3-think"> <fs id="53"><feat attr="wform" val="think"/> </fs> </macro>

<entry pred="hope" stem="hope"  pos="WORD" class="blah">
  <word form="hope"/>
</entry>
<macro name="@mwe1-hope"> <fs id="51"><feat attr="wform" val="hope"/> </fs> </macro>
<macro name="@mwe2-hope"> <fs id="52"><feat attr="wform" val="hope"/> </fs> </macro>
<macro name="@mwe3-hope"> <fs id="53"><feat attr="wform" val="hope"/> </fs> </macro>

<entry pred="guess" stem="guess"  pos="WORD" class="blah">
  <word form="guess"/>
</entry>
<macro name="@mwe1-guess"> <fs id="51"><feat attr="wform" val="guess"/> </fs> </macro>
<macro name="@mwe2-guess"> <fs id="52"><feat attr="wform" val="guess"/> </fs> </macro>
<macro name="@mwe3-guess"> <fs id="53"><feat attr="wform" val="guess"/> </fs> </macro>

<entry pred="know" stem="know"  pos="WORD" class="blah">
  <word form="know"/>
</entry>
<macro name="@mwe1-know"> <fs id="51"><feat attr="wform" val="know"/> </fs> </macro>
<macro name="@mwe2-know"> <fs id="52"><feat attr="wform" val="know"/> </fs> </macro>
<macro name="@mwe3-know"> <fs id="53"><feat attr="wform" val="know"/> </fs> </macro>

<entry pred="lot" stem="lot"  pos="WORD" class="blah">
  <word form="lot"/>
</entry>
<macro name="@mwe1-lot"> <fs id="51"><feat attr="wform" val="lot"/> </fs> </macro>
<macro name="@mwe2-lot"> <fs id="52"><feat attr="wform" val="lot"/> </fs> </macro>
<macro name="@mwe3-lot"> <fs id="53"><feat attr="wform" val="lot"/> </fs> </macro>

<entry pred="bunch" stem="bunch"  pos="WORD" class="blah">
  <word form="bunch"/>
</entry>
<macro name="@mwe1-bunch"> <fs id="51"><feat attr="wform" val="bunch"/> </fs> </macro>
<macro name="@mwe2-bunch"> <fs id="52"><feat attr="wform" val="bunch"/> </fs> </macro>
<macro name="@mwe3-bunch"> <fs id="53"><feat attr="wform" val="bunch"/> </fs> </macro>

<entry pred="load" stem="load"  pos="WORD" class="blah">
  <word form="load"/>
</entry>
<macro name="@mwe1-load"> <fs id="51"><feat attr="wform" val="load"/> </fs> </macro>
<macro name="@mwe2-load"> <fs id="52"><feat attr="wform" val="load"/> </fs> </macro>
<macro name="@mwe3-load"> <fs id="53"><feat attr="wform" val="load"/> </fs> </macro>

<entry pred="few" stem="few"  pos="WORD" class="blah">
  <word form="few"/>
</entry>
<macro name="@mwe1-few"> <fs id="51"><feat attr="wform" val="few"/> </fs> </macro>
<macro name="@mwe2-few"> <fs id="52"><feat attr="wform" val="few"/> </fs> </macro>
<macro name="@mwe3-few"> <fs id="53"><feat attr="wform" val="few"/> </fs> </macro>

<entry pred="couple" stem="couple"  pos="WORD" class="blah">
  <word form="couple"/>
</entry>
<macro name="@mwe1-couple"> <fs id="51"><feat attr="wform" val="couple"/> </fs> </macro>
<macro name="@mwe2-couple"> <fs id="52"><feat attr="wform" val="couple"/> </fs> </macro>
<macro name="@mwe3-couple"> <fs id="53"><feat attr="wform" val="couple"/> </fs> </macro>

<entry pred="see" stem="see"  pos="WORD" class="blah">
  <word form="see"/>
</entry>
<macro name="@mwe1-see"> <fs id="51"><feat attr="wform" val="see"/> </fs> </macro>
<macro name="@mwe2-see"> <fs id="52"><feat attr="wform" val="see"/> </fs> </macro>
<macro name="@mwe3-see"> <fs id="53"><feat attr="wform" val="see"/> </fs> </macro>

<entry pred="front" stem="front"  pos="WORD" class="blah">
  <word form="front"/>
</entry>
<macro name="@mwe1-front"> <fs id="51"><feat attr="wform" val="front"/> </fs> </macro>
<macro name="@mwe2-front"> <fs id="52"><feat attr="wform" val="front"/> </fs> </macro>
<macro name="@mwe3-front"> <fs id="53"><feat attr="wform" val="front"/> </fs> </macro>

<entry pred="back" stem="back"  pos="WORD" class="blah">
  <word form="back"/>
</entry>
<macro name="@mwe1-back"> <fs id="51"><feat attr="wform" val="back"/> </fs> </macro>
<macro name="@mwe2-back"> <fs id="52"><feat attr="wform" val="back"/> </fs> </macro>
<macro name="@mwe3-back"> <fs id="53"><feat attr="wform" val="back"/> </fs> </macro>

<entry pred="behind" stem="behind"  pos="WORD" class="blah">
  <word form="behind"/>
</entry>
<macro name="@mwe1-behind"> <fs id="51"><feat attr="wform" val="behind"/> </fs> </macro>
<macro name="@mwe2-behind"> <fs id="52"><feat attr="wform" val="behind"/> </fs> </macro>
<macro name="@mwe3-behind"> <fs id="53"><feat attr="wform" val="behind"/> </fs> </macro>

<entry pred="bottom" stem="bottom"  pos="WORD" class="blah">
  <word form="bottom"/>
</entry>
<macro name="@mwe1-bottom"> <fs id="51"><feat attr="wform" val="bottom"/> </fs> </macro>
<macro name="@mwe2-bottom"> <fs id="52"><feat attr="wform" val="bottom"/> </fs> </macro>
<macro name="@mwe3-bottom"> <fs id="53"><feat attr="wform" val="bottom"/> </fs> </macro>

<entry pred="from" stem="from"  pos="WORD" class="blah">
  <word form="from"/>
</entry>
<macro name="@mwe1-from"> <fs id="51"><feat attr="wform" val="from"/> </fs> </macro>
<macro name="@mwe2-from"> <fs id="52"><feat attr="wform" val="from"/> </fs> </macro>
<macro name="@mwe3-from"> <fs id="53"><feat attr="wform" val="from"/> </fs> </macro>

<entry pred="the" stem="the"  pos="WORD" class="blah">
  <word form="the"/>
</entry>
<macro name="@mwe1-the"> <fs id="51"><feat attr="wform" val="the"/> </fs> </macro>
<macro name="@mwe2-the"> <fs id="52"><feat attr="wform" val="the"/> </fs> </macro>
<macro name="@mwe3-the"> <fs id="53"><feat attr="wform" val="the"/> </fs> </macro>

<entry pred="right" stem="right"  pos="WORD" class="blah">
  <word form="right"/>
</entry>
<macro name="@mwe1-right"> <fs id="51"><feat attr="wform" val="right"/> </fs> </macro>
<macro name="@mwe2-right"> <fs id="52"><feat attr="wform" val="right"/> </fs> </macro>
<macro name="@mwe3-right"> <fs id="53"><feat attr="wform" val="right"/> </fs> </macro>

<entry pred="left" stem="left"  pos="WORD" class="blah">
  <word form="left"/>
</entry>
<macro name="@mwe1-left"> <fs id="51"><feat attr="wform" val="left"/> </fs> </macro>
<macro name="@mwe2-left"> <fs id="52"><feat attr="wform" val="left"/> </fs> </macro>
<macro name="@mwe3-left"> <fs id="53"><feat attr="wform" val="left"/> </fs> </macro>

<entry pred="job" stem="job"  pos="WORD" class="blah">
  <word form="job"/>
</entry>
<macro name="@mwe1-job"> <fs id="51"><feat attr="wform" val="job"/> </fs> </macro>
<macro name="@mwe2-job"> <fs id="52"><feat attr="wform" val="job"/> </fs> </macro>
<macro name="@mwe3-job"> <fs id="53"><feat attr="wform" val="job"/> </fs> </macro>

<entry pred="done" stem="done"  pos="WORD" class="blah">
  <word form="done"/>
</entry>
<macro name="@mwe1-done"> <fs id="51"><feat attr="wform" val="done"/> </fs> </macro>
<macro name="@mwe2-done"> <fs id="52"><feat attr="wform" val="done"/> </fs> </macro>
<macro name="@mwe3-done"> <fs id="53"><feat attr="wform" val="done"/> </fs> </macro>

<entry pred="and" stem="and"  pos="WORD" class="blah">
  <word form="and"/>
</entry>
<macro name="@mwe1-and"> <fs id="51"><feat attr="wform" val="and"/> </fs> </macro>
<macro name="@mwe2-and"> <fs id="52"><feat attr="wform" val="and"/> </fs> </macro>
<macro name="@mwe3-and"> <fs id="53"><feat attr="wform" val="and"/> </fs> </macro>

<entry pred="around" stem="around"  pos="WORD" class="blah">
  <word form="around"/>
</entry>
<macro name="@mwe1-around"> <fs id="51"><feat attr="wform" val="around"/> </fs> </macro>
<macro name="@mwe2-around"> <fs id="52"><feat attr="wform" val="around"/> </fs> </macro>
<macro name="@mwe3-around"> <fs id="53"><feat attr="wform" val="around"/> </fs> </macro>

<entry pred="here" stem="here"  pos="WORD" class="blah">
  <word form="here"/>
</entry>
<macro name="@mwe1-here"> <fs id="51"><feat attr="wform" val="here"/> </fs> </macro>
<macro name="@mwe2-here"> <fs id="52"><feat attr="wform" val="here"/> </fs> </macro>
<macro name="@mwe3-here"> <fs id="53"><feat attr="wform" val="here"/> </fs> </macro>

<entry pred="there" stem="there"  pos="WORD" class="blah">
  <word form="there"/>
</entry>
<macro name="@mwe1-there"> <fs id="51"><feat attr="wform" val="there"/> </fs> </macro>
<macro name="@mwe2-there"> <fs id="52"><feat attr="wform" val="there"/> </fs> </macro>
<macro name="@mwe3-there"> <fs id="53"><feat attr="wform" val="there"/> </fs> </macro>

<entry pred="phones" stem="phones"  pos="WORD" class="blah">
  <word form="phones"/>
</entry>
<macro name="@mwe1-phones"> <fs id="51"><feat attr="wform" val="phones"/> </fs> </macro>
<macro name="@mwe2-phones"> <fs id="52"><feat attr="wform" val="phones"/> </fs> </macro>
<macro name="@mwe3-phones"> <fs id="53"><feat attr="wform" val="phones"/> </fs> </macro>

<entry pred="phone" stem="phone"  pos="WORD" class="blah">
  <word form="phone"/>
</entry>
<macro name="@mwe1-phone"> <fs id="51"><feat attr="wform" val="phone"/> </fs> </macro>
<macro name="@mwe2-phone"> <fs id="52"><feat attr="wform" val="phone"/> </fs> </macro>
<macro name="@mwe3-phone"> <fs id="53"><feat attr="wform" val="phone"/> </fs> </macro>

<entry pred="to" stem="to"  pos="WORD" class="blah">
  <word form="to"/>
</entry>
<macro name="@mwe1-to"> <fs id="51"><feat attr="wform" val="to"/> </fs> </macro>
<macro name="@mwe2-to"> <fs id="52"><feat attr="wform" val="to"/> </fs> </macro>
<macro name="@mwe3-to"> <fs id="53"><feat attr="wform" val="to"/> </fs> </macro>

<entry pred="room" stem="room"  pos="WORD" class="blah">
  <word form="room"/>
</entry>
<macro name="@mwe1-room"> <fs id="51"><feat attr="wform" val="room"/> </fs> </macro>
<macro name="@mwe2-room"> <fs id="52"><feat attr="wform" val="room"/> </fs> </macro>
<macro name="@mwe3-room"> <fs id="53"><feat attr="wform" val="room"/> </fs> </macro>

<entry pred="rooms" stem="rooms"  pos="WORD" class="blah">
  <word form="rooms"/>
</entry>
<macro name="@mwe1-rooms"> <fs id="51"><feat attr="wform" val="rooms"/> </fs> </macro>
<macro name="@mwe2-rooms"> <fs id="52"><feat attr="wform" val="rooms"/> </fs> </macro>
<macro name="@mwe3-rooms"> <fs id="53"><feat attr="wform" val="rooms"/> </fs> </macro>

<entry pred="machine" stem="machine"  pos="WORD" class="blah">
  <word form="machine"/>
</entry>
<macro name="@mwe1-machine"> <fs id="51"><feat attr="wform" val="machine"/> </fs> </macro>
<macro name="@mwe2-machine"> <fs id="52"><feat attr="wform" val="machine"/> </fs> </macro>
<macro name="@mwe3-machine"> <fs id="53"><feat attr="wform" val="machine"/> </fs> </macro>

<entry pred="maker" stem="maker"  pos="WORD" class="blah">
  <word form="maker"/>
</entry>
<macro name="@mwe1-maker"> <fs id="51"><feat attr="wform" val="maker"/> </fs> </macro>
<macro name="@mwe2-maker"> <fs id="52"><feat attr="wform" val="maker"/> </fs> </macro>
<macro name="@mwe3-maker"> <fs id="53"><feat attr="wform" val="maker"/> </fs> </macro>

<entry pred="machines" stem="machines"  pos="WORD" class="blah">
  <word form="machines"/>
</entry>
<macro name="@mwe1-machines"> <fs id="51"><feat attr="wform" val="machines"/> </fs> </macro>
<macro name="@mwe2-machines"> <fs id="52"><feat attr="wform" val="machines"/> </fs> </macro>
<macro name="@mwe3-machines"> <fs id="53"><feat attr="wform" val="machines"/> </fs> </macro>

<entry pred="makers" stem="makers"  pos="WORD" class="blah">
  <word form="makers"/>
</entry>
<macro name="@mwe1-makers"> <fs id="51"><feat attr="wform" val="makers"/> </fs> </macro>
<macro name="@mwe2-makers"> <fs id="52"><feat attr="wform" val="makers"/> </fs> </macro>
<macro name="@mwe3-makers"> <fs id="53"><feat attr="wform" val="makers"/> </fs> </macro>

<entry pred="station" stem="station"  pos="WORD" class="blah">
  <word form="station"/>
</entry>
<macro name="@mwe1-station"> <fs id="51"><feat attr="wform" val="station"/> </fs> </macro>
<macro name="@mwe2-station"> <fs id="52"><feat attr="wform" val="station"/> </fs> </macro>
<macro name="@mwe3-station"> <fs id="53"><feat attr="wform" val="station"/> </fs> </macro>

<entry pred="stations" stem="stations"  pos="WORD" class="blah">
  <word form="stations"/>
</entry>
<macro name="@mwe1-stations"> <fs id="51"><feat attr="wform" val="stations"/> </fs> </macro>
<macro name="@mwe2-stations"> <fs id="52"><feat attr="wform" val="stations"/> </fs> </macro>
<macro name="@mwe3-stations"> <fs id="53"><feat attr="wform" val="stations"/> </fs> </macro>

<entry pred="point" stem="point"  pos="WORD" class="blah">
  <word form="point"/>
</entry>
<macro name="@mwe1-point"> <fs id="51"><feat attr="wform" val="point"/> </fs> </macro>
<macro name="@mwe2-point"> <fs id="52"><feat attr="wform" val="point"/> </fs> </macro>
<macro name="@mwe3-point"> <fs id="53"><feat attr="wform" val="point"/> </fs> </macro>

<entry pred="points" stem="points"  pos="WORD" class="blah">
  <word form="points"/>
</entry>
<macro name="@mwe1-points"> <fs id="51"><feat attr="wform" val="points"/> </fs> </macro>
<macro name="@mwe2-points"> <fs id="52"><feat attr="wform" val="points"/> </fs> </macro>
<macro name="@mwe3-points"> <fs id="53"><feat attr="wform" val="points"/> </fs> </macro>

<entry pred="kettle" stem="kettle"  pos="WORD" class="blah">
  <word form="kettle"/>
</entry>
<macro name="@mwe1-kettle"> <fs id="51"><feat attr="wform" val="kettle"/> </fs> </macro>
<macro name="@mwe2-kettle"> <fs id="52"><feat attr="wform" val="kettle"/> </fs> </macro>
<macro name="@mwe3-kettle"> <fs id="53"><feat attr="wform" val="kettle"/> </fs> </macro>

<entry pred="kettles" stem="kettles"  pos="WORD" class="blah">
  <word form="kettles"/>
</entry>
<macro name="@mwe1-kettles"> <fs id="51"><feat attr="wform" val="kettles"/> </fs> </macro>
<macro name="@mwe2-kettles"> <fs id="52"><feat attr="wform" val="kettles"/> </fs> </macro>
<macro name="@mwe3-kettles"> <fs id="53"><feat attr="wform" val="kettles"/> </fs> </macro>

<entry pred="by" stem="by"  pos="WORD" class="blah">
  <word form="by"/>
</entry>
<macro name="@mwe1-by"> <fs id="51"><feat attr="wform" val="by"/> </fs> </macro>
<macro name="@mwe2-by"> <fs id="52"><feat attr="wform" val="by"/> </fs> </macro>
<macro name="@mwe3-by"> <fs id="53"><feat attr="wform" val="by"/> </fs> </macro>


  

  <!-- ================================================ -->
  <!-- =====	  AND HERE						 ====== --> 
  <!-- ================================================ -->


  <xsl:call-template name="add-macros"/>
 

  </dictionary>
</xsl:template>

</xsl:transform>


