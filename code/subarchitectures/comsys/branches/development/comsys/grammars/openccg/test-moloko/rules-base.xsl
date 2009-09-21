<?xml version="1.0" encoding="UTF-8"?>


<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">

  <!-- ================================================ -->  
  <!-- OUTPUT OPTIONS                                   -->
  <!-- ================================================ -->  

  <xsl:import href="../test-core/unary-rules.xsl"/>
  <xsl:output indent="yes" xalan2:indent-amount="2"/>
  <xsl:strip-space elements="*"/> 

<xsl:template match="/">
<rules name="moloko"
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:noNamespaceSchemaLocation="../rules.xsd"
>

  <application dir="forward"/>
  <application dir="backward"/>
  <composition dir="forward" harmonic="true"/>
  <composition dir="forward" harmonic="false"/>
  <composition dir="backward" harmonic="true"/>
  <composition dir="backward" harmonic="false"/>

  <typeraising dir="forward" useDollar="true"/> 
  <typeraising dir="forward" useDollar="false"/> 
    
  <!-- 
  <typeraising dir="forward" useDollar="false">
    <arg>
      <atomcat type="s"/>
    </arg>
  </typeraising>
-->



<!-- <typeraising dir="backward" useDollar="false"/> -->

  <substitution dir="forward" harmonic="true"/>
  <substitution dir="forward" harmonic="false"/>
  <substitution dir="backward" harmonic="true"/>
  <substitution dir="backward" harmonic="false"/>


<!-- Add Unary Rules to handle (among other things)
	 complex cat adjs   (see unary-rules.xsl)  -->
	 
  <xsl:call-template name="add-unary-rules"/>


</rules>
</xsl:template>
</xsl:transform>
