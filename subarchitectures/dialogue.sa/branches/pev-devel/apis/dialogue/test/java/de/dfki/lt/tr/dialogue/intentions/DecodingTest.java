package de.dfki.lt.tr.dialogue.intentions;

import cast.cdl.WorkingMemoryAddress;
import java.util.HashMap;
import org.junit.*;
import static org.junit.Assert.*;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.dialogue.intentions.inst.*;
import java.util.LinkedList;
import java.util.List;

public class DecodingTest {

	@Test
	public void featureAscription() {
		
		BaseIntention bint = new BaseIntention(new HashMap<String, String>(), new HashMap<String, WorkingMemoryAddress>());
		
		bint.stringContent.put("type", "assertion");
		bint.stringContent.put("subtype", "ascription");
		bint.stringContent.put("asserted-feature", "color");
		bint.stringContent.put("asserted-value", "red");
		bint.stringContent.put("asserted-polarity", "pos");
		bint.addressContent.put("about", new WorkingMemoryAddress("x", "y"));
	
		System.out.println("about to decode...");
		FeatureAscriptionIntention decoded = FeatureAscriptionIntention.Transcoder.INSTANCE.tryDecode(bint);
		System.out.println("decoded it.");
		
		assertNotNull(decoded);
		assertTrue(decoded.getFeatureName().equals("color"));
		assertTrue(decoded.isPositive());
	}

	@Test
	public void featureAnswer() {
		
		BaseIntention bint = new BaseIntention(new HashMap<String, String>(), new HashMap<String, WorkingMemoryAddress>());
		
		bint.stringContent.put("type", "assertion");
		bint.stringContent.put("subtype", "ascription");
		bint.stringContent.put("subsubtype", "answer");
		bint.stringContent.put("asserted-feature", "color");
		bint.stringContent.put("asserted-value", "red");
		bint.stringContent.put("asserted-polarity", "neg");
		bint.stringContent.put("certainty", "high");
		bint.addressContent.put("answer-to", new WorkingMemoryAddress("z", "w"));
		bint.addressContent.put("about", new WorkingMemoryAddress("x", "y"));
	
		FeatureAscriptionIntention ascr = FeatureAscriptionIntention.Transcoder.INSTANCE.tryDecode(bint);
		FeatureAnswerIntention answr = FeatureAnswerIntention.Transcoder.INSTANCE.tryDecode(bint);
		
		assertNotNull(ascr);
		assertNotNull(answr);

		System.out.println(ascr.getClass().getCanonicalName());
		System.out.println(answr.getClass().getCanonicalName());
	}

}
