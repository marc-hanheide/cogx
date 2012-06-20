package eu.cogx.mln;

import java.util.Collections;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.Vector;

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
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import de.dfki.lt.tr.beliefs.util.ProbFormula;
import eu.cogx.beliefs.slice.MergedBelief;
import eu.cogx.mln.slice.MLNFact;
import eu.cogx.mln.slice.MLNState;

public class MLNSerializer extends ManagedComponent {

	WMEventQueue queue = new WMEventQueue();
	
	Set<WorkingMemoryAddress> allBeliefWMA = Collections
			.synchronizedSet(new HashSet<WorkingMemoryAddress>());
	
	private String stateId = null;
	
	private static final int TIME_TO_WAIT_TO_SETTLE = 100;

	static void addToMLN(dBelief bel, Vector<MLNFact> facts)
			throws BeliefException {
	
		IndependentFormulaDistributionsBelief<dBelief> b = IndependentFormulaDistributionsBelief
				.create(dBelief.class, bel);
		// Do not add anything if not visible
		for (Entry<String, FormulaDistribution> d : b.getContent().entrySet()) {
			if (d.getKey().equals("presence")) {
				for (ProbFormula v : d.getValue()) {
					ElementaryFormula pf = (ElementaryFormula) v.getFormula()
							.get();
					if (pf.prop.equals("removed") && v.getProbability() > 0.8)
						return;
				}
			}
			// HACK: Filtering out phantom shared beliefs
			if (d.getKey().equals("color")) {
				if (d.getValue().size() == 0)
					return;
			}
		}

		MLNFact bfact = new MLNFact();
		MLNFact sfact = new MLNFact();

		bfact.prob = sfact.prob = 1;
		bfact.type = sfact.type = bel.type;
		bfact.id = sfact.id = bel.id;
		bfact.key = "belief";
		sfact.key = "epstatus";

		if (bel.estatus instanceof PrivateEpistemicStatus) {
			bfact.estatus = sfact.estatus = "private";
			sfact.atom = sfact.key + "(" + sfact.id + ", " + "Private)";
		} else if (bel.estatus instanceof AttributedEpistemicStatus) {
			bfact.estatus = sfact.estatus = "attributed";
			sfact.atom = sfact.key + "(" + sfact.id + ", " + "Attributed)";
		} else if (bel.estatus instanceof SharedEpistemicStatus) {
			bfact.estatus = sfact.estatus = "shared";
			sfact.atom = sfact.key + "(" + sfact.id + ", " + "Shared)";
		}
		bfact.atom = bfact.key + "(" + bfact.id + ")";

		facts.add(bfact);
		facts.add(sfact);

		for (Entry<String, FormulaDistribution> d : b.getContent().entrySet()) {
			for (ProbFormula v : d.getValue()) {
				if (b.getType().equals("relation")) {
					// not looking at relations yet
				} else {
					MLNFact fact = new MLNFact();
					fact.prob = weight(v.getProbability());
					fact.type = bel.type;
					// fact.key = d.getKey();
					fact.id = b.getId();
					fact.estatus = bfact.estatus;

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
					if (bel.estatus instanceof PrivateEpistemicStatus) {
						fact.key = "private_" + d.getKey();
						fact.atom = fact.key + "(" + b.getId() + ", P_"
								+ formula + ")";
					} else if (bel.estatus instanceof AttributedEpistemicStatus) {
						fact.key = "attributed_" + d.getKey();
						fact.atom = fact.key + "(" + b.getId() + ", A_"
								+ formula + ")";
					} else if (bel.estatus instanceof SharedEpistemicStatus) {
						fact.key = d.getKey();
						fact.atom = fact.key + "(" + b.getId() + ", P_"
								+ formula + ")";
					}

					// fact.atom = "   " + d.getKey() + "(" + b.getType() + "_"
					// + b.getId() + ", V_"
					// + formula + ")";
					facts.add(fact);
				}
			}
		}

	}

	private static double weight(float probability) {

		return probability;
	}


	@Override
	protected void runComponent() {

		while (isRunning()) {
			Vector<MLNFact> facts = new Vector<MLNFact>();

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
						addToMLN(b, facts);
					} catch (DoesNotExistOnWMException e) {
						// ignore this, it could happen
					} catch (Exception e) {
						logException(e);
					}
				}

				MLNState mlnState = new MLNState(facts.toArray(new MLNFact[0]));

				for (int i = 0; i < facts.size(); i++)
					log(mlnState.facts[i].type + " | " + mlnState.facts[i].id
							+ " | " + mlnState.facts[i].prob + " | "
							+ mlnState.facts[i].atom);
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
		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(MergedBelief.class),
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

		Vector<MLNFact> facts = new Vector<MLNFact>();
		addToMLN(b.get(), facts);

		MLNState mlnState = new MLNState(facts.toArray(new MLNFact[0]));

		for (int i = 0; i < facts.size(); i++)
			System.out.println(mlnState.facts[i].prob + " "
					+ mlnState.facts[i].atom);
	}
}
