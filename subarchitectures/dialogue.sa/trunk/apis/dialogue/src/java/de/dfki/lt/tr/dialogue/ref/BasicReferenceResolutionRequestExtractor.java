package de.dfki.lt.tr.dialogue.ref;

import de.dfki.lt.tr.dialogue.ref.util.ConstraintMap;
import de.dfki.lt.tr.dialogue.slice.lf.Feature;
import de.dfki.lt.tr.dialogue.slice.lf.LFNominal;
import de.dfki.lt.tr.dialogue.time.TimeInterval;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import java.util.Iterator;
import org.apache.log4j.Logger;

public class BasicReferenceResolutionRequestExtractor
implements ReferenceResolutionRequestExtractor {

	private final Logger logger;

	public BasicReferenceResolutionRequestExtractor(Logger logger) {
		this.logger = logger;
	}

	@Override
	public ReferenceResolutionRequest extractReferenceResolutionRequest(LFNominal nom, TimeInterval ival) {
		logger.info("converting nominal to constraints");
		ConstraintMap cmap = new ConstraintMap();

		String sort = getSort(nom.sort);

		if (!nom.prop.prop.equals("context")) {
			String type = nom.prop.prop;
			tryAddToConstraintMap(cmap, "type", type);
		}

		Iterator<Feature> iter = LFUtils.lfNominalGetFeatures(nom);
		while (iter.hasNext()) {
			Feature feat = iter.next();
			tryAddToConstraintMap(cmap, feat.feat, feat.value);
		}
		logger.info("conversion done");
		return new ReferenceResolutionRequest(nom.nomVar, sort, cmap.toConstraintList(), ival.toIce());
	}

	private void tryAddToConstraintMap(ConstraintMap cmap, String feature, String value) {
		cmap.contentMap().put(feature.toLowerCase(), value);
	}

	private String getSort(String sort) {
		if (sort.equals("thing")) {
			return "object";
		}
		return sort;
	}

}
