/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions.abstr;

import java.util.ArrayList;
import java.util.StringTokenizer;

import org.apache.log4j.Logger;

import cast.architecture.ManagedComponent;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import cast.interfaces.TimeServerPrx;
import castutils.castextensions.CASTHelper;
import castutils.castextensions.WMEntrySynchronizer.TransferFunction;
import de.dfki.lt.tr.beliefs.data.CASTSuperBelief;
import de.dfki.lt.tr.beliefs.data.genericproxies.Distribution;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

/**
 * This is an abstract class for the most simple {@link TransferFunction} to
 * establish a mapping between input percepts (of generic type From) and
 * {@link dBelief}.
 * 
 * @author nah (copied from {@link SimpleDiscreteTransferFunction}
 * 
 * @param <From>
 *            type we generate beliefs from
 */
public abstract class AbstractDiscreteTransferFunction<From extends Ice.ObjectImpl, To extends dBelief>
		extends CASTHelper implements TransferFunction<From, To> {

	public static String getBeliefTypeFromCastType(
			Class<? extends Ice.Object> class1) {
		return getBeliefTypeFromCastType(CASTUtils.typeName(class1));
	}

	public static String getBeliefTypeFromCastType(String casttype) {
		StringTokenizer st = new StringTokenizer(casttype, ":");
		String type = casttype.toLowerCase();
		while (st.hasMoreTokens())
			type = st.nextToken();
		return type;
	}

	private static TimeServerPrx timeServer = null;
	final protected Class<To> beliefClass;

	/**
	 * constructor
	 * 
	 * @param component
	 * @param beliefType
	 *            TODO
	 */
	public AbstractDiscreteTransferFunction(ManagedComponent component,
			Logger logger, Class<To> beliefType) {
		super(component);
		this.beliefClass = beliefType;
	}

	protected void addAncestors(WorkingMemoryChange _wmc, From _from,
			CASTSuperBelief<To, ? extends Distribution<?>> _p) {
		CASTBeliefHistory hist = new CASTBeliefHistory(
				new ArrayList<WorkingMemoryPointer>(1),
				new ArrayList<WorkingMemoryPointer>(0));
		hist.ancestors.add(new WorkingMemoryPointer(_wmc.address, _wmc.type));
		_p.get().hist = hist;
	}

	public static CASTTime now() {
		if (timeServer == null)
			timeServer = CASTUtils.getTimeServer();
		return timeServer.getCASTTime();
	}
}
