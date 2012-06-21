package de.dfki.lt.tr.dialogue.ref;

import de.dfki.lt.tr.dialogue.ref.util.ConstraintMap;
import de.dfki.lt.tr.dialogue.slice.lf.LFNominal;
import de.dfki.lt.tr.dialogue.slice.lf.LFRelation;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.time.TimeInterval;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import org.apache.log4j.Logger;

public class BasicReferenceResolutionRequestExtractor
implements ReferenceResolutionRequestExtractor {

	private final Logger logger;

	public BasicReferenceResolutionRequestExtractor(Logger logger) {
		this.logger = logger;
	}

	@Override
	public ReferenceResolutionRequest extractReferenceResolutionRequest(LogicalForm lf, String nomvar, TimeInterval ival) {
		logger.info("converting nominal to constraints");
		ConstraintMap cmap = new ConstraintMap();

		String sort = getSort(lf, nomvar);
		traverseNominal(cmap, lf, nomvar);

		logger.info("conversion done");
		return new ReferenceResolutionRequest(nomvar, sort, cmap.toConstraintList(), ival.toIce());
	}

	private static void tryAddToConstraintMap(ConstraintMap cmap, String feature, String value) {
		cmap.contentMap().put(feature.toLowerCase(), value);
	}

	// (box1_1:thing ^ box ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(red1_1:q-color ^ red)
	// (object1_2:thing ^ object ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(red1_2:q-color ^ red))

	private static String getSort(LogicalForm lf, String nomvar) {
		LFNominal nom = LFUtils.lfGetNominal(lf, nomvar);
		String sort = nom.sort;
		String prop = nom.prop.prop;
		if (prop.equals("it")) {
			return ReferenceResolver.SORT_DISCOURSE;
		}
		if (prop.equals("context")) {
			return ReferenceResolver.SORT_INDETERMINATE;
		}

		if (sort.equals("thing")) {
			return ReferenceResolver.SORT_OBJECT;
		}
		if (sort.equals("e-place")) {
			return ReferenceResolver.SORT_PLACE;
		}

		return sort;
	}

	public static void traverseNominal(ConstraintMap cmap, LogicalForm lf, String nomvar) {
		LFNominal nom = LFUtils.lfGetNominal(lf, nomvar);
		if (nom != null) {

			String propName = getPropertyName(nom);
			if (propName != null) {
				// it's a property
				tryAddToConstraintMap(cmap, propName, nom.prop.prop);
			}
			else {
				// it's an object
				String prop = nom.prop.prop;
				if (!prop.equals("object") && !prop.equals("thing") && !prop.equals("context") && !prop.equals("it")) {
					tryAddToConstraintMap(cmap, "type", nom.prop.prop);
				}

				// try for saliency
				if (!LFUtils.lfNominalGetFeature(nom, "Proximity").equals("") || prop.equals("it")) {
					tryAddToConstraintMap(cmap, "salience", "high");
				}
			}

			// do the traversal
			LFRelation rel = LFUtils.lfNominalGetRelation(nom, "Modifier");
			if (rel != null) {
				traverseNominal(cmap, lf, rel.dep);
			}
		}
	}

	public static String getPropertyName(LFNominal nom) {
		if (nom.sort.equals("q-color")) {
			return "color";
		}
		if (nom.sort.equals("q-shape")) {
			return "shape";
		}
		return null;
	}

}
