/**
 * 
 */
package vision.components.mediators.abstr;

import java.util.NoSuchElementException;
import java.util.Set;

import SpatialProperties.IntegerValue;
import SpatialProperties.PlaceContainmentAgentProperty;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEntrySynchronizer;
import castutils.castextensions.WMView;
import castutils.castextensions.WMEntrySynchronizer.TransferFunction;

/**
 * @author marc
 * 
 */
public abstract class AbstractLocalizedPerceptionMediator<From extends Ice.ObjectImpl, To extends Ice.ObjectImpl> extends WMEntrySynchronizer<From, To> implements TransferFunction<From, To>{
	protected WMView<PlaceContainmentAgentProperty> agentPlace;
	
	public AbstractLocalizedPerceptionMediator(ManagedComponent c,
			Class<From> fromType, Class<To> toType,
			Set<WorkingMemoryOperation> ops) {
		super(c, fromType, toType, null, ops);
		this.transferFunction=this;
		agentPlace = WMView.create(component, PlaceContainmentAgentProperty.class);
	}

	@Override
	public boolean transform(WorkingMemoryChange wmc, From from, To to) {
		long place = getPlace();
		
		return convert(wmc, from, to, place);
	}
	
	protected long getPlace() {
		try {
			PlaceContainmentAgentProperty pcap = agentPlace.values().iterator()
					.next();
			return ((IntegerValue) pcap.mapValue).value;
		} catch (NoSuchElementException e) {
			component.logException(e);
			return -1;
		}

	}

	protected abstract boolean convert(WorkingMemoryChange wmc, From from,
	To to, long placeId);
	
	
}
