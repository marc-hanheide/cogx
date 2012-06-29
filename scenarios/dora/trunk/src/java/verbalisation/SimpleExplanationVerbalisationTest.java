package verbalisation;


import java.util.HashSet;

import junit.framework.TestCase;

import org.junit.Test;

import castutils.castextensions.IceXMLSerializer;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.HypotheticalBelief;

public class SimpleExplanationVerbalisationTest extends TestCase 
{
  private HashSet<HypotheticalBelief> initialBeliefs;
  private GroundedBelief proposedObjectLocation;

  @Override
  protected void setUp() throws Exception 
  {
    super.setUp();
    String in1 =
        "<eu.cogx.beliefs.slice.HypotheticalBelief>" +
        "  <frame class=\"de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame\">" +
        "    <place></place> " +
        "    <existProb>0.0</existProb>" +
        "  </frame>" +
        "  <estatus class=\"de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus\">" +
        "    <agent>robot</agent>" +
        "  </estatus>" +
        "  <id>7:t</id>" +
        "  <type>visualobject</type>" +
        "  <content class=\"de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs\">" +
        "    <distribs>" +
        "      <entry>" +
        "        <string>relation</string>" +
        "        <de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "          <key>relation</key>" +
        "          <values class=\"de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues\">" +    
        "            <values class=\"linked-list\">    " +
        "          <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula\">" +
        "                  <id>0</id>" +
        "                  <prop>in</prop>" +
        "                </val>" +
        "                <prob>1.0</prob>" +
        "              </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "            </values>" +
        "          </values>" +
        "        </de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "      </entry>" +
        "      <entry>" +
        "        <string>label</string>" +
        "        <de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "          <key>label</key>" +
        "          <values class=\"de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues\">" +
        "            <values class=\"linked-list\">" +
        "              <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula\">" +
        "                  <id>0</id>" +
        "                  <prop>container</prop>" +
        "                </val>" +
        "                <prob>1.0</prob>" +
        "              </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "            </values>" +
        "          </values>" +
        "        </de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "      </entry>" +
        "      <entry>" +
        "        <string>related-to</string>" +
        "        <de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "          <key>related-to</key>" +
        "          <values class=\"de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues\">" +
        "            <values class=\"linked-list\">" +
        "              <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula\">" +
        "                  <id>0</id>" +
        "                  <pointer>" +
        "                    <id>0:61</id>" +
        "                    <subarchitecture>coma</subarchitecture>" +
        "                  </pointer>" +
        "                  <type></type>" +
        "                </val>" +
        "                <prob>1.0</prob>" +
        "              </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "            </values>" +
        "          </values>" +
        "        </de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "      </entry>" +
        "    </distribs>" +
        "  </content>" +
        "  <hist class=\"de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory\">" +
        "    <ancestors/>" +
        "    <offspring/>" +
        "  </hist>" +
        "</eu.cogx.beliefs.slice.HypotheticalBelief>";
    
    String in2 =
        "<eu.cogx.beliefs.slice.HypotheticalBelief>" +
        "  <frame class=\"de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame\">" +
        "<place></place> " +
        "<existProb>0.0</existProb>" +
        "  </frame>" +
        "  <estatus class=\"de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus\">" +
        "    <agent>robot</agent>" +
        "  </estatus>" +
        "  <id>6:t</id>" +
        "  <type>visualobject</type>" +
        "  <content class=\"de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs\">" +
        "    <distribs>" +
        "      <entry>" +
        "        <string>relation</string>" +
        "        <de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "          <key>relation</key>" +
        "          <values class=\"de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues\">" +    
        "            <values class=\"linked-list\">    " +
        "          <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula\">" +
        "                  <id>0</id>" +
        "                  <prop>in</prop>" +
        "                </val>" +
        "                <prob>1.0</prob>" +
        "              </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "            </values>" +
        "          </values>" +
        "        </de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "      </entry>" +
        "      <entry>" +
        "        <string>label</string>" +
        "        <de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "          <key>label</key>" +
        "          <values class=\"de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues\">" +
        "            <values class=\"linked-list\">" +
        "              <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula\">" +
        "                  <id>0</id>" +
        "                  <prop>magazine</prop>" +
        "                </val>" +
        "                <prob>1.0</prob>" +
        "              </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "            </values>" +
        "          </values>" +
        "        </de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "      </entry>" +
        "      <entry>" +
        "        <string>related-to</string>" +
        "        <de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "          <key>related-to</key>" +
        "          <values class=\"de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues\">" +
        "            <values class=\"linked-list\">" +
        "              <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula\">" +
        "                  <id>0</id>" +
        "                  <pointer>" +
        "                    <id>7:t</id>" +
        "                    <subarchitecture>coma</subarchitecture>" +
        "                  </pointer>" +
        "                  <type></type>" +
        "                </val>" +
        "                <prob>1.0</prob>" +
        "              </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "            </values>" +
        "          </values>" +
        "        </de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "      </entry>" +
        "    </distribs>" +
        "  </content>" +
        "  <hist class=\"de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory\">" +
        "    <ancestors/>" +
        "    <offspring/>" +
        "  </hist>" +
        "</eu.cogx.beliefs.slice.HypotheticalBelief>";
    
    initialBeliefs = new HashSet<HypotheticalBelief>();
    initialBeliefs.add(IceXMLSerializer.fromXMLString(in1, HypotheticalBelief.class));
    initialBeliefs.add(IceXMLSerializer.fromXMLString(in2, HypotheticalBelief.class));
    
    String in3 = 
        "<eu.cogx.beliefs.slice.GroundedBelief>" +
        "    <frame class=\"de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame\">" +
        "       <place>here</place>" +
        "      <interval class=\"de.dfki.lt.tr.beliefs.slice.framing.CASTTemporalInterval\">" +
        "        <start>" +
        "          <s>55</s>" +
        "          <us>788505</us>" +
        "        </start>" +
        "        <end>" +
        "          <s>56</s>" +
        "          <us>8900</us>" +
        "        </end>" +
        "      </interval>" +
        "      <existProb>1.0</existProb>" +
        "    </frame>" +
        "    <estatus class=\"de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus\">" +
        "      <agent>self</agent>" +
        "    </estatus>" +
        "    <id>0:61</id>" +
        "    <type>comaroom</type>" +
        "    <content class=\"de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs\">" +
        "      <distribs>" +
        "        <entry>" +
        "          <string>category</string>" +
        "          <de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "            <key>category</key>" +
        "            <values class=\"de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues\">" +
        "              <values class=\"linked-list\">" +
        "                <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                  <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula\">" +
        "                    <id>-1</id>" +
        "                    <prop>corridor</prop>" +
        "                  </val>" +
        "                  <prob>0.0043241237</prob>" +
        "                </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                  <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula\">" +
        "                    <id>-1</id>" +
        "                    <prop>meetingroom</prop>" +
        "                  </val>" +
        "                  <prob>0.9898489</prob>" +
        "                </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                  <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula\">" +
        "                    <id>-1</id>" +
        "                    <prop>office</prop>" +
        "                  </val>" +
        "                  <prob>0.005826956</prob>" +
        "                </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "              </values>" +
        "            </values>" +
        "          </de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "        </entry>" +
        "        <entry>" +
        "          <string>RoomId</string>" +
        "          <de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "            <key>RoomId</key>" +
        "            <values class=\"de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues\">" +
        "              <values class=\"linked-list\">" +
        "                <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                  <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula\">" +
        "                    <id>-1</id>" +
        "                    <val>0</val>" +
        "                  </val>" +
        "                  <prob>1.0</prob>" +
        "               </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "              </values>" +
        "            </values>" +
        "          </de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "        </entry>" +
        "        <entry>" +
        "          <string>contains-a-person-prior</string>" +
        "          <de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "            <key>contains-a-person-prior</key>" +
        "            <values class=\"de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues\">" +
        "              <values class=\"linked-list\">" +
        "                <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                  <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula\">" +
        "                    <id>-1</id>" +
        "                    <val>true</val>" +
        "                 </val>" +
        "                  <prob>0.2</prob>" +
        "                </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "              </values>" +
        "            </values>" +
        "          </de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "        </entry>" +
        "      </distribs>" +
        "    </content>" +
        "    <hist class=\"de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory\">" +
        "      <ancestors>" +
        "        <cast.cdl.WorkingMemoryPointer>" +
        "          <address> " +
        "            <id>0:31</id>" +
        "            <subarchitecture>coma</subarchitecture>" +
        "          </address>" +
        "          <type>::comadata::ComaRoom</type>" +
        "        </cast.cdl.WorkingMemoryPointer>" +
        "      </ancestors>" +
        "      <offspring/>" +
        "    </hist>" +
        "  </eu.cogx.beliefs.slice.GroundedBelief>";
    proposedObjectLocation = IceXMLSerializer.fromXMLString(in3, GroundedBelief.class);
  }

  @Test
  public void testVerbaliseExplanation() 
  {
    SimpleExplanationVerbalisation sev = new SimpleExplanationVerbalisation(initialBeliefs);
    sev.findProposedObjectAndLocation();
    assertTrue(sev.generateText().startsWith("Perhaps the magazine is in a container which is in"));
  }

  @Test
  public void testGenerateRoomText() 
  {
    SimpleExplanationVerbalisation sev = new SimpleExplanationVerbalisation(initialBeliefs);
    assertEquals("room 0 which is most likely a meetingroom", 
        sev.generateRoomText(proposedObjectLocation));
  }

}
