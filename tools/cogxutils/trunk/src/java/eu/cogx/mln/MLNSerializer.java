package eu.cogx.mln;

import java.util.Collections;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.Vector;
import java.util.Map.Entry;

import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEventQueue;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import de.dfki.lt.tr.beliefs.util.ProbFormula;
import eu.cogx.mln.slice.MLNState;

public class MLNSerializer extends ManagedComponent {

	WMEventQueue queue = new WMEventQueue();
	Set<WorkingMemoryAddress> allBeliefWMA = Collections
			.synchronizedSet(new HashSet<WorkingMemoryAddress>());
	private String stateId = null;
	private static final int TIME_TO_WAIT_TO_SETTLE = 100;

	static void addToMLN(dBelief bel, Vector<String> types,
			Vector<String> ids, Vector<String> facts,
			Vector<Double> probs) throws BeliefException {

		IndependentFormulaDistributionsBelief<dBelief> b = IndependentFormulaDistributionsBelief
				.create(dBelief.class, bel);
		for (Entry<String, FormulaDistribution> d : b.getContent().entrySet()) {
			for (ProbFormula v : d.getValue()) {
				if (b.getType().equals("relation")) {
					// not looking at relations yet
				} else {
					double logProb = weight(v.getProbability());
					probs.add(logProb);
					
					types.add(bel.type);
					ids.add(b.getId());
					
					String formula = v.getFormula().toString();
					if (v.getFormula().get() instanceof PointerFormula) {
						PointerFormula pf = (PointerFormula) v.getFormula()
								.get();
						formula = pf.pointer.id;
					} else if (v.getFormula().get() instanceof ElementaryFormula) {
						ElementaryFormula pf = (ElementaryFormula) v
								.getFormula().get();
						formula = pf.prop;
					}
					facts.add("   " + d.getKey() + "(" + b.getType() + "_" + b.getId() + ", "
							+ formula + ")");
				}
			}
		}

	}

	private static double weight(float probability) {

		return probability;
	}

	@Override
	protected void configure(Map<String, String> config) {
		// TODO Auto-generated method stub
		super.configure(config);
	}

	@Override
	protected void runComponent() {
		
		while (isRunning()) {
			Vector<String> types = new Vector<String>();
			Vector<String> ids = new Vector<String>();
			Vector<String> facts = new Vector<String>();
			Vector<Double> probs = new Vector<Double>();
			
			try {
				queue.take();
				log("got an belief update event, will wait now for "
						+ TIME_TO_WAIT_TO_SETTLE
						+ "ms to let all changes come in.");
				sleepComponent(TIME_TO_WAIT_TO_SETTLE);
				queue.clear();

				lockComponent();
				for (WorkingMemoryAddress adr : allBeliefWMA) {
					dBelief b;
					try {
						b = getMemoryEntry(adr, dBelief.class);
						addToMLN(b, types, ids, facts, probs);
					} catch (DoesNotExistOnWMException e) {
						// ignore this, it could happen
					} catch (Exception e) {
						logException(e);
					}
				}
				
				MLNState mlnState = new MLNState(types.toArray(new String[0]),
					ids.toArray(new String[0]), facts.toArray(new String[0]),
					toDoubleArray(probs));
				
				for (int i = 0; i < probs.size(); i++)
					log(mlnState.types[i] + " | " + mlnState.ids[i] + " | " +
						mlnState.probs[i] + " | " + mlnState.facts[i]);
				if (stateId == null) {
					stateId = newDataID();
					addToWorkingMemory(stateId, mlnState);
				} else {
					overwriteWorkingMemory(stateId, mlnState);
				}
			} catch (InterruptedException e) {
				logException(e);
			} catch (CASTException e) {
				logException(e);
			} finally {
				unlockComponent();
			}
		}
	}

	private static double[] toDoubleArray(Vector<Double> probs) {
		double[] p = new double[probs.size()];
		for (int i = 0; i < probs.size(); i++)
			p[i] = probs.get(i);
		return p;
	}

	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory
				.createGlobalTypeFilter(dBelief.class),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange arg0)
							throws CASTException {
						if (arg0.operation == WorkingMemoryOperation.ADD)
							allBeliefWMA.add(arg0.address);
						if (arg0.operation == WorkingMemoryOperation.DELETE)
							allBeliefWMA.remove(arg0.address);
						queue.add(arg0);
					}
				});
	}

	public static void main(String[] argv) {
		IndependentFormulaDistributionsBelief<dBelief> b = IndependentFormulaDistributionsBelief
				.create(dBelief.class);
		b.setId("theBeliefId");
		FormulaDistribution fd = FormulaDistribution.create();
		WMPointer ptr = WMPointer.create(new WorkingMemoryAddress("sdds",
				"sdsdf"), "place");
		fd.add(ptr.get(), 0.3);
		ptr = WMPointer.create(new WorkingMemoryAddress("erter", "sfghfgdsdf"),
				"place");
		fd.add(ptr.get(), 0.4);
		fd.add("unknown", 0.3);

		b.getContent().put("is-in", fd);

		fd = FormulaDistribution.create();
		fd.add("VRed", 0.4);
		fd.add("VGreen", 0.5);
		fd.add("VBlue", 0.1);

		b.getContent().put("color", fd);
		Vector<String> types = new Vector<String>();
		Vector<String> ids = new Vector<String>();
		Vector<String> facts = new Vector<String>();
		Vector<Double> probs = new Vector<Double>();
		addToMLN(b.get(), types, ids,facts, probs);
		MLNState mlnState = new MLNState(types.toArray(new String[0]),
				ids.toArray(new String[0]), facts.toArray(new String[0]),
				toDoubleArray(probs));
		

		for (int i = 0; i < probs.size(); i++)
			System.out.println(mlnState.probs[i] +" "+mlnState.facts[i]);
	}
}
