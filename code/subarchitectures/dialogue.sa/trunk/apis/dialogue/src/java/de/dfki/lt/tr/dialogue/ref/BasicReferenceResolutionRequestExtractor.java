package de.dfki.lt.tr.dialogue.ref;

import de.dfki.lt.tr.dialogue.slice.lf.Feature;
import de.dfki.lt.tr.dialogue.slice.lf.LFNominal;
import de.dfki.lt.tr.dialogue.time.TimeInterval;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import org.apache.log4j.Logger;

public class BasicReferenceResolutionRequestExtractor
implements ReferenceResolutionRequestExtractor {

	private final Logger logger;

	public BasicReferenceResolutionRequestExtractor(Logger logger) {
		this.logger = logger;
	}

	@Override
	public ReferenceResolutionRequest extractReferenceResolutionRequest(LFNominal nom, TimeInterval ival) {
		List<Constraint> cs = new ArrayList<Constraint>();
		Iterator<Feature> iter = LFUtils.lfNominalGetFeatures(nom);
		logger.info("converting nominal to constraints");

		String sort = nom.sort;

		if (!nom.prop.prop.equals("context")) {
			String type = nom.prop.prop;
			cs.add(new Constraint("Type", type));
		}

		while (iter.hasNext()) {
			Feature feat = iter.next();
			Constraint c = new Constraint(feat.feat, feat.value);
			cs.add(c);
		}
		logger.info("conversion done");
		return new ReferenceResolutionRequest(nom.nomVar, sort, cs, ival.toIce());
	}

}
