/** 
 * @author Marc Hanheide (marc@hanheide.de)
 */
package motivation.components.generators;

import nu.xom.Document;
import nu.xom.Element;
import nu.xom.Node;
import nu.xom.Nodes;
import autogen.Planner.Goal;
import motivation.slice.GeneralGoalMotive;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.IceXMLSerializer;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;

/** a generator that takes an utterance like "find me the cornflakes box"
 * @author Marc Hanheide (marc@hanheide.de)
 *
 */
public class FindObjectFromIntentionGenerator extends AbstractIntentionMotiveGenerator<GeneralGoalMotive, Intention> {

	private static final String XPATH_SELECT_OBJ_LABEL = "//[op/text()='Color']/form/prop/text()";

	protected FindObjectFromIntentionGenerator() {
		super(GeneralGoalMotive.class, Intention.class);
	}

	@Override
	protected GeneralGoalMotive checkForAddition(WorkingMemoryAddress addr,
			Intention newEntry) {
		GeneralGoalMotive motive = new GeneralGoalMotive();
		fillDefault(motive);
		
		if (newEntry.estatus instanceof AttributedEpistemicStatus) {
			// only attributed beliefs contain intentions
			Document xmlDoc = IceXMLSerializer.toXomDom(newEntry);
			// get intention
			// TODO: something smart has to be done here...
			// I have to access from the struct
			// ... which object is referred to (the GroundedBelief)
			// ... which concept is given (Colour, Shape)
			// ... have to check whether this is new information or not

			Element objLabelNode = (Element) queryFirst(xmlDoc,
					XPATH_SELECT_OBJ_LABEL);
			// TODO: fill these values:
			String objLabel=objLabelNode.getValue();
			String robotBeliefId="";
			motive.goal= new Goal(-1, -1, "exists (?o - visualobject) (and (= (label ?o) "+objLabel+") (kval '"+robotBeliefId+"' (is-in ?o))))", false);
			return motive;
		} else {
			return null;
		}
		
	}
	private Node queryFirst(Node node, String xpath) {
		Nodes res = node.query(xpath);
		if (res.size() != 1)
			return null;
		else
			return res.get(0);
	}


	@Override
	protected GeneralGoalMotive checkForUpdate(Intention newEntry,
			GeneralGoalMotive motive) {

		return motive;
	}

}
