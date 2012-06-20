package castutils.tests;

import static org.junit.Assert.*;

import nu.xom.Document;
import nu.xom.Nodes;

import org.junit.Test;

import castutils.castextensions.IceXMLSerializer;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

public class BeliefXMLSerializerTest {
	final Ice.Object testobj=new dBelief(new SpatioTemporalFrame(), new PrivateEpistemicStatus("me"), "hurga", "test", null, null);
	
	
	@Test
	public void testToXMLString() {
		String res=IceXMLSerializer.toXMLString(testobj);
		System.out.println(res);
		assertTrue(true);
	}

	@Test
	public void testToXomDom() {
		Document doc = IceXMLSerializer.toXomDom(testobj);
		System.out.println(doc.toXML());
		Nodes n = doc.query("/*[estatus/agent/text()=\"me\"]/id/text()");
		assertTrue(n.get(0).toXML().equals("hurga"));
	}

}
