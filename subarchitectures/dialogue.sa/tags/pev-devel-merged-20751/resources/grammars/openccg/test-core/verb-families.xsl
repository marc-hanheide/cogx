<?xml version="1.0"?>

<xsl:transform
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  version="1.0"
  xmlns:xalan="http://xml.apache.org/xalan"
  xmlns:xalan2="http://xml.apache.org/xslt"
  exclude-result-prefixes="xalan xalan2">


 <xsl:template name="add-verb-families">


 <family name="iv" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR"/>
      </xsl:call-template>
    </entry>
 </family>

  <family name="tv" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.np)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT"/>
      </xsl:call-template>
    </entry>
  </family>

 <family name="resultant" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.np.adj)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.RESULTANT"/>
      </xsl:call-template>
    </entry>
  </family>
  
  <family name="caused-motion.to" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.np.pp)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.WHERETO"/>
      </xsl:call-template>
    </entry>
  </family>

 <family name="caused-motion.from.to" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.np.pp.pp)/*"/>
        <xsl:with-param name="ext" select="$E.ACTOR.PATIENT.WHEREFROM.WHERETO"/>
      </xsl:call-template>
    </entry>
  </family>

 <family name="copular.np" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.np)/*"/>
        <xsl:with-param name="ext" select="$E.RESTR.SCOPE"/>
      </xsl:call-template>
    </entry>
  </family>

 <family name="copular.pp" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.pp)/*"/>
        <xsl:with-param name="ext" select="$E.RESTR.SCOPE"/>
      </xsl:call-template>
    </entry>
  </family>

 <family name="copular.adj" pos="V" closed="true">
    <entry name="Primary">
      <xsl:call-template name="extend">
        <xsl:with-param name="elt" select="xalan:nodeset($verb.adj)/*"/>
        <xsl:with-param name="ext" select="$E.RESTR.SCOPE"/>
      </xsl:call-template>
    </entry>
  </family>

</xsl:template>

</xsl:transform>