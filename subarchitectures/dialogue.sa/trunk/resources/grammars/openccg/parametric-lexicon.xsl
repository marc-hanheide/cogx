<?xml version="1.0"?>

<!-- Lexical rules for setting grammatical relations.
     These rules set up a skeletal CCG lexicon on which lexical
     development can be built.

- Cem Bozsahin 2003 (cem.bozsahin@ed.ac.uk / bozsahin@metu.edu.tr)
-->

<xsl:transform xmlns:xsl="http://www.w3.org/1999/XSL/Transform" 
version="1.0" xmlns:xalan="http://xml.apache.org/xslt" 
exclude-result-prefixes="xalan">

  <xsl:output method="xml" encoding="UTF-8" indent="yes" 
  xalan:indent-amount="3"/>

  <xsl:variable name="controlled.predicate.basic.type" 
  select="//iv//predicate/@syn-type"/>

  <xsl:variable name="controllee.type" 
  select="//controllee/@subject-type"/>

  <xsl:variable name="controlled.argument.basic.type" 
  select="//iv//s-argument/@syn-type"/>

  <xsl:variable name="pred.0.E">
    <fs id="100">        <!-- 0 is special; use 100 instead for result-->
      <feat attr="index">
        <lf>
          <nomvar name="E"/>
        </lf>
      </feat>
    </fs>
  </xsl:variable>

  <xsl:variable name="controlled-vp-inf-cat.1">
    <complexcat>
      <xsl:comment>NB: Word order and directionality of embedded clause's 
|(S|NP) type is an 'educated guess' from the syntactic type of TV. 
Change them accordingly.
  </xsl:comment>
      <atomcat>
        <xsl:attribute name="type">
          <xsl:value-of select="$controlled.predicate.basic.type"/>
        </xsl:attribute>
        <fs id="3">
          <feat attr="vform" val="non-fin"/>
          <feat attr="index">
            <lf>
              <nomvar name="E2"/>
            </lf>
          </feat>
        </fs>
      </atomcat>
      <slash dir="|"/>
      <atomcat>
        <xsl:attribute name="type">
          <xsl:value-of select="$controlled.argument.basic.type"/>
        </xsl:attribute>
        <fs id="4">
		<xsl:comment> NB: If controllee type is syntactic subject (which is the 
		  default, you
		  need to define in types.xml file a type called
		  "subject-controllee-cases". Include in this type all the subjects that
		  can be controlled e.g. nominative subjects only (as in German), 
		  dative and nominative subjects (as in Malayalam) etc.
		  
		  If controllee type is semantic subject, it is always 1. We suggest you
		  look at control of unaccusatives in the language as to whether their
		  surface subject can be controlled or not.
		</xsl:comment>
		<xsl:choose>
		<xsl:when test="$controllee.type = 'semantic'">
		  <xsl:call-template name="make-attr-val">
		     <xsl:with-param name="attr" select="'arg-index'"/>
		     <xsl:with-param name="val" select="'1'"/>
		   </xsl:call-template>
		</xsl:when>
		<xsl:otherwise>
		  <xsl:call-template name="make-attr-val">
		     <xsl:with-param name="attr" select="'case'"/>
		     <xsl:with-param name="val" select="'subject-controllee-cases'"/>
		   </xsl:call-template>
		</xsl:otherwise>
		</xsl:choose>
          <feat attr="index">
            <lf>
              <nomvar name="X1"/>
            </lf>
            <xsl:comment>NB: Index is same as Arg1's 
  </xsl:comment>
          </feat>
        </fs>
      </atomcat>
    </complexcat>
  </xsl:variable>

  <xsl:variable name="controlled-vp-inf-cat.2">
    <complexcat>
  <xsl:comment>NB: Word order and directionality of embedded clause's |(S|NP) 
type is an 'educated guess' from the syntactic type of TV. Change them
accordingly 
  </xsl:comment>
      <atomcat>
        <xsl:attribute name="type">
          <xsl:value-of select="$controlled.predicate.basic.type"/>
        </xsl:attribute>
        <fs id="3">
          <feat attr="vform" val="non-fin"/>
          <feat attr="index">
            <lf>
              <nomvar name="E2"/>
            </lf>
          </feat>
        </fs>
      </atomcat>
      <slash dir="|"/>
      <atomcat>
        <xsl:attribute name="type">
          <xsl:value-of select="$controlled.argument.basic.type"/>
        </xsl:attribute>
        <fs id="4">
<xsl:comment> NB: If controllee type is syntactic subject (which is the 
		  default, you
		  need to define in types.xml file a type called
		  "subject-controllee-cases". Include in this type all the cases of
		  subjects that can be controlled e.g. nominative subjects only 
		  (as in German), dative and nominative subjects (as in Malayalam) etc.
		  
		  If controllee type is semantic subject, it is always 1. We suggest you
		  look at control of unaccusatives in the language as to whether their
		  surface subject can be controlled or not.
		</xsl:comment>
		<xsl:choose>
		<xsl:when test="$controllee.type = 'semantic'">
		  <xsl:call-template name="make-attr-val">
		     <xsl:with-param name="attr" select="'arg-index'"/>
		     <xsl:with-param name="val" select="'1'"/>
		   </xsl:call-template>
		</xsl:when>
		<xsl:otherwise>
		  <xsl:call-template name="make-attr-val">
		     <xsl:with-param name="attr" select="'case'"/>
		     <xsl:with-param name="val" select="'subject-controllee-cases'"/>
		   </xsl:call-template>
		</xsl:otherwise>
		</xsl:choose>
          <feat attr="index">
            <lf>
              <nomvar name="X2"/>
            </lf>
            <xsl:comment>NB: Index is same as Arg2's 
  </xsl:comment>
          </feat>
        </fs>
      </atomcat>
    </complexcat>
  </xsl:variable>

  <xsl:template match="language">

   <!-- ** start output here ** -->

    <xsl:comment> 
      - This file is generated by parametric-lexicon.xsl to set up
          accusativity/ergativity parameter for IV and TV primary families
          and control primary families.

          NB: pre-CCG categories of lexicon-skeleton.xml are mapped to
              CCG categories in this file. From now on, it's all CCG

    Suggestions to start-up lexicon development:

      1) Copy this file to lexicon-base.xml to avoid losing your changes
         to it (remember, this file is auto-generated at the start)
      2) Edit lexicon-base.xml to modify the preset families and to add your 
          own families as needed (merging the entries of same family is
          left to you)
      3) Use the build facility of openCCG to build
          the lexicon.xml, morph.xml and rules.xml files needed by the system.
  </xsl:comment>

    <xsl:comment>

      *** Families derived from language parameters ***

   Includes primary entries for IV, TV, TV-control1, TV-control2, IV-control1
   
  </xsl:comment>

    <ccg-lexicon xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" 
	xsi:noNamespaceSchemaLocation="../lexicon.xsd">

      <xsl:attribute name="name">
        <xsl:value-of select="@name"/>
      </xsl:attribute>
    

      <xsl:apply-templates select="skeleton/iv">
        <xsl:with-param name="new.case" select="@type"/>
        <xsl:with-param name="iv.1.case" select="skeleton/iv//@case"/>
      </xsl:apply-templates>
      <xsl:apply-templates select="skeleton/tv" mode="normal">
        <xsl:with-param name="new.case" select="@type"/>
        <xsl:with-param name="iv.1.case" select="skeleton/iv//@case"/>
      </xsl:apply-templates>
      <xsl:apply-templates select="skeleton/tv" mode="control">
        <xsl:with-param name="new.case" select="@type"/>
        <xsl:with-param name="iv.1.case" select="skeleton/iv//@case"/>
      </xsl:apply-templates>

      <xsl:comment>

    *** End of derived families ***
 
        Add new families here, and merge the new entries for preset
         families as needed (e.g., you may add an entry to TV family
         for pro-dropping the subject etc.)
  </xsl:comment>
  

    </ccg-lexicon>

  </xsl:template>
  
  <xsl:template match="skeleton/iv">
    <xsl:param name="new.case"/>
    <xsl:param name="iv.1.case"/>

    <family name="IV" pos="V" closed="true">
      <entry name="primary">
        <complexcat>
          <atomcat>
            <xsl:attribute name="type">
              <xsl:value-of select="predicate/@syn-type"/>
            </xsl:attribute>
            <xsl:copy-of select="$pred.0.E"/>
          </atomcat>
          <xsl:apply-templates select="setarg|arg">
            <xsl:with-param name="mode" select="'normal'"/>
            <xsl:with-param name="new.case" select="$new.case"/>
            <xsl:with-param name="iv.1.case" select="$iv.1.case"/>
          </xsl:apply-templates>
          <lf>
            <satop nomvar="E">
              <prop name="[*DEFAULT*]"/>
              <diamond mode="Arg1">
                <nomvar name="X1"/>
              </diamond>
            </satop>
          </lf>
        </complexcat>
      </entry>
    </family>
  </xsl:template>

  <xsl:template match="skeleton/tv" mode="normal">
    <xsl:param name="new.case"/>
    <xsl:param name="iv.1.case"/>

    <family name="TV" pos="V" closed="true">
      <entry name="primary">
        <complexcat>
          <atomcat>
            <xsl:attribute name="type">
              <xsl:value-of select="predicate/@syn-type"/>
            </xsl:attribute>
            <xsl:copy-of select="$pred.0.E"/>
          </atomcat>
          <xsl:apply-templates select="setarg|arg">
            <xsl:with-param name="mode" select="'normal'"/>
            <xsl:with-param name="new.case" select="$new.case"/>
            <xsl:with-param name="iv.1.case" select="$iv.1.case"/>
          </xsl:apply-templates>
          <lf>
            <satop nomvar="E">
              <prop name="[*DEFAULT*]"/>
              <diamond mode="Arg1">
                <nomvar name="X1"/>
              </diamond>
              <diamond mode="Arg2">
                <nomvar name="X2"/>
              </diamond>
            </satop>
          </lf>
        </complexcat>
      </entry>
    </family>
  </xsl:template>

  <xsl:template match="skeleton/tv" mode="control">
    <xsl:param name="new.case"/>
    <xsl:param name="iv.1.case"/>

    <family name="TV-control2" pos="V" closed="true">
      <entry name="primary">
        <complexcat>
          <atomcat>
            <xsl:attribute name="type">
              <xsl:value-of select="predicate/@syn-type"/>
            </xsl:attribute>
            <xsl:copy-of select="$pred.0.E"/>
          </atomcat>
          <xsl:apply-templates select="setarg|arg">
            <xsl:with-param name="mode" select="'control2-tv'"/>
            <xsl:with-param name="new.case" select="$new.case"/>
            <xsl:with-param name="iv.1.case" select="$iv.1.case"/>
          </xsl:apply-templates>
          <lf>
            <satop nomvar="E">
              <prop name="[*DEFAULT*]"/>
              <diamond mode="Arg1">
                <nomvar name="X1"/>
              </diamond>
              <diamond mode="Arg2">
                <nomvar name="X2"/>
              </diamond>
              <diamond mode="Arg3">
                <nomvar name="E2"/>
     <xsl:comment>NB: Argument is the PAS of the embedded S|NP  (cf. E2 above)
</xsl:comment>
              </diamond>
            </satop>
          </lf>
        </complexcat>
      </entry>
    </family>
    <family name="TV-control1" pos="V" closed="true">
      <entry name="primary">
        <complexcat>
          <atomcat>
            <xsl:attribute name="type">
              <xsl:value-of select="predicate/@syn-type"/>
            </xsl:attribute>
            <xsl:copy-of select="$pred.0.E"/>
          </atomcat>
          <xsl:apply-templates select="setarg|arg">
            <xsl:with-param name="mode" select="'control1-tv'"/>
            <xsl:with-param name="new.case" select="$new.case"/>
            <xsl:with-param name="iv.1.case" select="$iv.1.case"/>
          </xsl:apply-templates>
          <lf>
            <satop nomvar="E">
              <prop name="[*DEFAULT*]"/>
              <diamond mode="Arg1">
                <nomvar name="X1"/>
              </diamond>
              <diamond mode="Arg2">
                <nomvar name="X2"/>
              </diamond>
              <diamond mode="Arg3">
                <nomvar name="E2"/>
 <xsl:comment>NB: Argument is the PAS of the embedded S|NP  (cf. E2 above)
</xsl:comment>
              </diamond>
            </satop>
          </lf>
        </complexcat>
      </entry>
    </family>
    <family name="IV-control1" pos="V" closed="true">
      <entry name="primary">
        <complexcat>
          <atomcat>
            <xsl:attribute name="type">
              <xsl:value-of select="predicate/@syn-type"/>
            </xsl:attribute>
            <xsl:copy-of select="$pred.0.E"/>
          </atomcat>
          <xsl:apply-templates select="setarg|arg">
            <xsl:with-param name="mode" select="'control1-iv'"/>
            <xsl:with-param name="new.case" select="$new.case"/>
            <xsl:with-param name="iv.1.case" select="$iv.1.case"/>
          </xsl:apply-templates>
          <lf>
            <satop nomvar="E">
              <prop name="[*DEFAULT*]"/>
              <diamond mode="Arg1">
                <nomvar name="X1"/>
              </diamond>
              <diamond mode="Arg2">
                <nomvar name="E2"/>
<xsl:comment>NB: Argument is the PAS of the embedded S|NP  (cf. E2 above)
</xsl:comment>
              </diamond>
            </satop>
          </lf>
        </complexcat>
      </entry>
    </family>
  </xsl:template>

  <xsl:template match="setarg">
    <xsl:param name="mode"/>
    <xsl:param name="new.case"/>
    <xsl:param name="iv.1.case"/>

    <setarg>
      <xsl:apply-templates select="s-argument|a-argument|p-argument">
        <xsl:with-param name="mode" select="$mode"/>
        <xsl:with-param name="new.case" select="$new.case"/>
        <xsl:with-param name="iv.1.case" select="$iv.1.case"/>
      </xsl:apply-templates>
    </setarg>
  </xsl:template>

  <xsl:template match="arg">
    <xsl:param name="mode"/>
    <xsl:param name="new.case"/>
    <xsl:param name="iv.1.case"/>

    <xsl:apply-templates select="s-argument|a-argument|p-argument">
      <xsl:with-param name="mode" select="$mode"/>
      <xsl:with-param name="new.case" select="$new.case"/>
      <xsl:with-param name="iv.1.case" select="$iv.1.case"/>
    </xsl:apply-templates>
  </xsl:template>

  <xsl:template match="a-argument">
    <xsl:param name="mode"/>
    <xsl:param name="new.case"/>
    <xsl:param name="iv.1.case"/>

    <slash>
      <xsl:attribute name="dir">
        <xsl:value-of select="@dir"/>
      </xsl:attribute>
    </slash>
    <atomcat>
      <xsl:attribute name="type">
        <xsl:value-of select="@syn-type"/>
      </xsl:attribute>
      <xsl:choose>
        <xsl:when test="$new.case = 'erg'">
          <fs id="1">
            <xsl:call-template name="make-attr-val">
              <xsl:with-param name="attr" select="'case'"/>
              <xsl:with-param name="val" select="$new.case"/>
            </xsl:call-template>
            <feat attr="arg-index" val="1"/>
            <feat attr="index">
              <lf>
                <nomvar name="X1"/>
              </lf>
            </feat>
          </fs>
        </xsl:when>
        <xsl:otherwise>   <!-- default language type is acc -->
          <fs id="1">
            <xsl:call-template name="make-attr-val">
              <xsl:with-param name="attr" select="'case'"/>
              <xsl:with-param name="val" select="$iv.1.case"/>
            </xsl:call-template>
            <feat attr="arg-index" val="1"/>
            <feat attr="index">
              <lf>
                <nomvar name="X1"/>
              </lf>
            </feat>
          </fs>
        </xsl:otherwise>
      </xsl:choose>
    </atomcat>
  </xsl:template>

  <xsl:template match="p-argument">
    <xsl:param name="mode"/>
    <xsl:param name="new.case"/>
    <xsl:param name="iv.1.case"/>

    <xsl:choose>
      <xsl:when test="$mode = 'normal'">
        <xsl:call-template name="set-p-argument-only">
          <xsl:with-param name="new.case" select="$new.case"/>
          <xsl:with-param name="iv.1.case" select="$iv.1.case"/>
        </xsl:call-template>
      </xsl:when>
      <xsl:when test="$mode = 'control1-tv' or 'control2-tv' or 'control1-iv'">
        <xsl:call-template name="set-p-argument-and-control">
          <xsl:with-param name="mode" select="$mode"/>
          <xsl:with-param name="new.case" select="$new.case"/>
          <xsl:with-param name="iv.1.case" select="$iv.1.case"/>
        </xsl:call-template>
      </xsl:when>
    </xsl:choose>
  </xsl:template>

  <xsl:template name="set-p-argument-only">
    <xsl:param name="new.case"/>
    <xsl:param name="iv.1.case"/>

    <slash>
      <xsl:attribute name="dir">
        <xsl:value-of select="@dir"/>
      </xsl:attribute>
    </slash>
    <atomcat>
      <xsl:attribute name="type">
        <xsl:value-of select="@syn-type"/>
      </xsl:attribute>
      <xsl:choose>
        <xsl:when test="$new.case = 'erg'">
          <fs id="2">
            <xsl:call-template name="make-attr-val">
              <xsl:with-param name="attr" select="'case'"/>
              <xsl:with-param name="val" select="$iv.1.case"/>
            </xsl:call-template>
            <feat attr="arg-index" val="2"/>
            <feat attr="index">
              <lf>
                <nomvar name="X2"/>
              </lf>
            </feat>
          </fs>
        </xsl:when>
        <xsl:otherwise>   <!-- default language type is acc -->
          <fs id="2">
            <xsl:call-template name="make-attr-val">
              <xsl:with-param name="attr" select="'case'"/>
              <xsl:with-param name="val" select="$new.case"/>
            </xsl:call-template>
            <feat attr="arg-index" val="2"/>
            <feat attr="index">
              <lf>
                <nomvar name="X2"/>
              </lf>
            </feat>
          </fs>
        </xsl:otherwise>
      </xsl:choose>
    </atomcat>
  </xsl:template>

  <xsl:template name="set-p-argument-and-control">
    <xsl:param name="mode"/>
    <xsl:param name="new.case"/>
    <xsl:param name="iv.1.case"/>
  
      <!-- S|NP comes after NP2 of matrix clause in SAME direction-->
    <slash>
      <xsl:attribute name="dir">
        <xsl:value-of select="@dir"/>
      </xsl:attribute>
    </slash>
    <xsl:choose>
      <xsl:when test="$mode = 'control1-tv'">
        <xsl:copy-of select="$controlled-vp-inf-cat.1"/>
        <slash>
          <xsl:attribute name="dir">
            <xsl:value-of select="@dir"/>
          </xsl:attribute>
        </slash>
        <atomcat>
          <xsl:attribute name="type">
            <xsl:value-of select="@syn-type"/>
          </xsl:attribute>
          <xsl:choose>
            <xsl:when test="$new.case = 'erg'">
              <fs id="2">
                <xsl:call-template name="make-attr-val">
                  <xsl:with-param name="attr" select="'case'"/>
                  <xsl:with-param name="val" select="$iv.1.case"/>
                </xsl:call-template>
                <feat attr="arg-index" val="2"/>
                <feat attr="index">
                  <lf>
                    <nomvar name="X2"/>
                  </lf>
                </feat>
              </fs>
            </xsl:when>
            <xsl:otherwise>   <!-- default language type is acc -->
              <fs id="2">
                <xsl:call-template name="make-attr-val">
                  <xsl:with-param name="attr" select="'case'"/>
                  <xsl:with-param name="val" select="$new.case"/>
                </xsl:call-template>
                <feat attr="arg-index" val="2"/>
                <feat attr="index">
                  <lf>
                    <nomvar name="X2"/>
                  </lf>
                </feat>
              </fs>
            </xsl:otherwise>
          </xsl:choose>
        </atomcat>
      </xsl:when>
      <xsl:when test="$mode = 'control1-iv'">
      <!-- just replace the NP2 of TV with S|NP with SAME direc.-->
        <xsl:copy-of select="$controlled-vp-inf-cat.1"/>
      </xsl:when>
      <xsl:when test="$mode = 'control2-tv'">
        <xsl:copy-of select="$controlled-vp-inf-cat.2"/>
        <slash>
          <xsl:attribute name="dir">
            <xsl:value-of select="@dir"/>
          </xsl:attribute>
        </slash>
        <atomcat>
          <xsl:attribute name="type">
            <xsl:value-of select="@syn-type"/>
          </xsl:attribute>
          <xsl:choose>
            <xsl:when test="$new.case = 'erg'">
              <fs id="2">
                <xsl:call-template name="make-attr-val">
                  <xsl:with-param name="attr" select="'case'"/>
                  <xsl:with-param name="val" select="$iv.1.case"/>
                </xsl:call-template>
                <feat attr="arg-index" val="2"/>
                <feat attr="index">
                  <lf>
                    <nomvar name="X2"/>
                  </lf>
                </feat>
              </fs>
            </xsl:when>
            <xsl:otherwise>   <!-- default language type is acc -->
              <fs id="2">
                <xsl:call-template name="make-attr-val">
                  <xsl:with-param name="attr" select="'case'"/>
                  <xsl:with-param name="val" select="$new.case"/>
                </xsl:call-template>
                <feat attr="arg-index" val="2"/>
                <feat attr="index">
                  <lf>
                    <nomvar name="X2"/>
                  </lf>
                </feat>
              </fs>
            </xsl:otherwise>
          </xsl:choose>
        </atomcat>
      </xsl:when>
    </xsl:choose>
  </xsl:template>

  <xsl:template match="s-argument">
    <xsl:param name="mode"/>
    <xsl:param name="new.case"/>
    <xsl:param name="iv.1.case"/>

    <slash>
      <xsl:attribute name="dir">
        <xsl:value-of select="@dir"/>
      </xsl:attribute>
    </slash>
    <atomcat>
      <xsl:attribute name="type">
        <xsl:value-of select="@syn-type"/>
      </xsl:attribute>
      <fs id="1">
        <xsl:call-template name="make-attr-val">
          <xsl:with-param name="attr" select="'case'"/>
          <xsl:with-param name="val" select="$iv.1.case"/>
        </xsl:call-template>
        <xsl:comment> NB: if the language is morphologically ergative but not
                        syntactically ergative, fix arg-index to 1 (cf.coord)
      </xsl:comment>
        <feat attr="arg-index">
          <featvar name="S_INDEX"/>
        </feat>
        <feat attr="index">
          <lf>
            <nomvar name="X1"/>
          </lf>
        </feat>
      </fs>
    </atomcat>
  </xsl:template>

  <xsl:template name="make-attr-val">
    <xsl:param name="attr"/>
    <xsl:param name="val"/>

    <feat>
      <xsl:attribute name="attr">
        <xsl:value-of select="$attr"/>
      </xsl:attribute>
      <xsl:attribute name="val">
        <xsl:value-of select="$val"/>
      </xsl:attribute>
    </feat>
  </xsl:template>

</xsl:transform>
