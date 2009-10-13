/**
 * 
 */
package spatial.motivation;


import java.util.HashMap;
import java.util.Map;

import motivation.components.generators.AbstractMotiveGenerator;
import motivation.factories.MotiveFactory;
import motivation.slice.ExploreMotive;
import motivation.slice.Motive;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import SpatialProperties.AssociatedBorderPlaceholderProperty;
import SpatialProperties.AssociatedSpacePlaceholderProperty;
import SpatialProperties.DiscreteProbabilityDistribution;
import SpatialProperties.FloatValue;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

/**
 * @author marc
 * 
 */
public class ExplorePlaceGenerator extends AbstractMotiveGenerator {


	private HashMap<Long, Double> m_placeIDtoBorderProperty;
	private HashMap<Long, Double> m_placeIDtoSpaceProperty;
	private HashMap<Long, WorkingMemoryAddress> m_placeIDtoMotiveWMA;
	private double m_spaceMeasureConstant;
	private double m_borderMeasureConstant;

	public ExplorePlaceGenerator() {
		super();
		m_placeIDtoBorderProperty = new HashMap<Long, Double>();
		m_placeIDtoSpaceProperty = new HashMap<Long, Double>();
		m_placeIDtoMotiveWMA = new HashMap<Long, WorkingMemoryAddress>();
	}

	/*
	 * (non-Javadoc) 
	 * 
	 * @see
	 * motivation.generators.Generator#updateMotive(cast.cdl.WorkingMemoryAddress
	 * , cast.cdl.WorkingMemoryAddress)
	 */
	@Override
	protected boolean checkMotive(Motive motive) {
		try {
			Place source = getMemoryEntry(motive.referenceEntry, Place.class);

			// if it is a yet unexplored one...
			log("there is a place to be checked, created "
					+ Long.toString(motive.created.s - getCASTTime().s)
					+ " seconds ago");

			if (source.status == PlaceStatus.PLACEHOLDER) {
				log("  it's a placeholder, so it should be considered as a motive");

				addHypothesisFeatures((ExploreMotive) motive);

				log("writing ExploreMotive to WM for place_id " + ((ExploreMotive) motive).placeID);
				WorkingMemoryAddress motiveAddress = write(motive);
				// need to store mapping back to place struct to establish
				// relations between places and their properties
				if (!m_placeIDtoMotiveWMA.containsKey(source.id)) {
					m_placeIDtoMotiveWMA.put(source.id, motiveAddress);
				}

				return true;
			} else {
				log("  turn out this place is not a placeholder, so, it should be no motive then");
				log("  getting rid of ExploreMotive for place_id " + ((ExploreMotive) motive).placeID);				
				remove(motive);
			}
		} catch (CASTException e) {
			e.printStackTrace();
		}
		return false;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		super.start();

		addChangeFilter(
				ChangeFilterFactory
						.createGlobalTypeFilter(AssociatedSpacePlaceholderProperty.class),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _arg0)
							throws CASTException {
						associatedSpaceChange(_arg0);
					}
				});

		addChangeFilter(
				ChangeFilterFactory
						.createGlobalTypeFilter(AssociatedBorderPlaceholderProperty.class),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _arg0)
							throws CASTException {
						associatedBorderChange(_arg0);
					}
				});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				debug(CASTUtils.toString(_wmc));
				// create a new motive from this node...
				ExploreMotive newMotive = MotiveFactory
						.createExploreMotive(_wmc.address);

				Place p;
				try {
					p = getMemoryEntry(_wmc.address, Place.class);
					newMotive.placeID = p.id;

				} catch (CASTException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				checkMotive(newMotive);
			}
		});
	}

	protected void associatedBorderChange(WorkingMemoryChange _wmc)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {

		if (_wmc.operation != WorkingMemoryOperation.DELETE) {
			AssociatedBorderPlaceholderProperty borderProperty = getMemoryEntry(
					_wmc.address, AssociatedBorderPlaceholderProperty.class);

			// first time through, add the place to the source list for the
			// place's motive
			if (_wmc.operation == WorkingMemoryOperation.ADD) {
				assert m_placeIDtoMotiveWMA.containsKey(borderProperty.placeId) : "place should be written before property";
				addReceivers(m_placeIDtoMotiveWMA.get(borderProperty.placeId),
						_wmc.address);
			}

			m_placeIDtoBorderProperty
					.put(
							borderProperty.placeId,
							getFirstPropertyValue((DiscreteProbabilityDistribution) borderProperty.distribution));
		}
	}

	protected void associatedSpaceChange(WorkingMemoryChange _wmc)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		if (_wmc.operation != WorkingMemoryOperation.DELETE) {
			AssociatedSpacePlaceholderProperty spaceProperty = getMemoryEntry(
					_wmc.address, AssociatedSpacePlaceholderProperty.class);

			// first time through, add the place to the source list for the
			// place's motive
			if (_wmc.operation == WorkingMemoryOperation.ADD) {
				assert m_placeIDtoMotiveWMA.containsKey(spaceProperty.placeId) : "place should be written before property";
				addReceivers(m_placeIDtoMotiveWMA.get(spaceProperty.placeId),
						_wmc.address);
			}

			m_placeIDtoSpaceProperty
					.put(
							spaceProperty.placeId,
							getFirstPropertyValue((DiscreteProbabilityDistribution) spaceProperty.distribution));
		}
	}

	private double getFirstPropertyValue(
			DiscreteProbabilityDistribution _probabilityDistribution) {
		return ((FloatValue) _probabilityDistribution.data[0].value).value;
	}

	/**
	 * 
	 * Assign an information gain value to the motive by using the two frontier
	 * measures.
	 * 
	 * @param _em
	 */
	private void addHypothesisFeatures(ExploreMotive _em) {
		long placeID = _em.placeID;

		// A measure of how many new nav nodes could be placed in the free space
		// beyond the frontoer

		Double spaceMeasureBoxed = m_placeIDtoSpaceProperty.get(placeID);
		double spaceMeasure = spaceMeasureBoxed == null ? 0 : spaceMeasureBoxed;

		// A measure of how much of the frontier borders on unknown space
		Double borderMeasureBoxed = m_placeIDtoBorderProperty.get(placeID);
		double borderMeasure = borderMeasureBoxed == null ? 0
				: borderMeasureBoxed;
		_em.informationGain = (spaceMeasure * m_spaceMeasureConstant)
				+ (borderMeasure * m_borderMeasureConstant);

		log(CASTUtils.concatenate(_em.informationGain, " = space(",
				spaceMeasure, " * ", m_spaceMeasureConstant, ") + border(",
				borderMeasure, " * ", m_borderMeasureConstant, ")"));
	}

	@Override
	protected void configure(Map<String, String> _config) {
		m_spaceMeasureConstant = 1;
		m_borderMeasureConstant = 1;

	}


}
