package comsys.processing.cca;

import java.util.*;

import Abducer.*;
import comsys.datastructs.comsysEssentials.*;
import beliefmodels.adl.*;
import beliefmodels.domainmodel.cogx.*;
import binder.utils.BeliefModelUtils;

import comsys.processing.cca.abduction.ProofUtils;

public class ProofStack {

	public ProofBlock[] blocks = null;
	
	public ProofStack() {
		blocks = new ProofBlock[0];
	}

	public static String[] beliefIds(Belief[] bs) {
		String[] ids = new String[bs.length];
		for (int i = 0; i < ids.length; i++) {
			ids[i] = bs[i].id;
		}
		return ids;
	}
	
	/**
	 * Construct a proof block based on an abductive proof.
	 * 
	 * @param proof
	 * @param proofId
	 * @return
	 */
	public static ProofBlock construct(ContextUpdate cu, Map<String, Belief> localBeliefs) {
		System.err.println("stack.construct()");
		ProofBlock pb = new ProofBlock();
		pb.intention = cu.intention;
		List<String> ids = new ArrayList<String>();
		for (int i = 0; i < cu.beliefs.length; i++) {
			Belief b = cu.beliefs[i];
			localBeliefs.put(b.id, b);
			ids.add(b.id);
		}
		pb.assertedBeliefIds = ids.toArray(new String[] {});
		return pb;
	}
 
	public static Belief[] blockToBeliefs(ProofBlock block, Map<String, Belief> localBeliefs) {
		List<Belief> bs = new ArrayList<Belief>();
		for (int i = 0; i < block.assertedBeliefIds.length; i++) {
			bs.add(localBeliefs.get(block.assertedBeliefIds[i]));
		}
		return bs.toArray(new Belief[]{});
	}
	
	/**
	 * Push the proof block onto the stack.
	 * 
	 * @param block
	*/
	public void push(ProofBlock block) {
		System.err.println("stack.push()");
		ProofBlock[] newStack = new ProofBlock[blocks.length+1];
		newStack[0] = block;
		for (int i = 0; i < blocks.length; i++) {
			newStack[i+1] = blocks[i];
		}
		//System.err.println(proofBlockToString(block));
		blocks = newStack;
	}

	/**
	 * Pop the top proof block from the stack.
	 * 
	 * @return the proof block, null if the stack is empty
	 */
	public ProofBlock pop() {
		System.err.println("stack.pop()");
		if (isEmpty()) {
			return null;
		}

		ProofBlock[] newBlocks = new ProofBlock[blocks.length-1];
		ProofBlock result = blocks[0];
		for (int i = 1; i < blocks.length; i++) {
			newBlocks[i-1] = blocks[i];
		}
		blocks = newBlocks;
		return result;
	}

	public int findTopmostBlockByIntention(String intentionPredSym) {
		boolean found = false;
		for (int i = 0; i < blocks.length; i++) {
			if (blocks[i].intention.predSym.equals(intentionPredSym)) {
				return i;
			}
		}
		return 999;  // ugly
	}
	
	public ProofBlock retrieveBlockByIntention(String intentionPredSym) {
		boolean found = false;
		ProofBlock result = null;
		Vector<ProofBlock> newBlocks = new Vector<ProofBlock>();
		for (int i = 0; i < blocks.length; i++) {
			if (!found && blocks[i].intention.predSym.equals(intentionPredSym)) {
				result = blocks[i];
				found = true;
			}
			else {
				newBlocks.add(blocks[i]);
			}
		}
		blocks = newBlocks.toArray(new ProofBlock[]{});
		return result;
	}
	
	public void removeBeliefFromBlocks(String beliefId) {
		// TODO
		Vector<ProofBlock> newBlocks = new Vector<ProofBlock>();
		for (int i = 0; i < blocks.length; i++) {
			ProofBlock block = blockWithoutBelief(blocks[i], beliefId);
			if (block != null) {
				newBlocks.add(block);
			}
		}
		blocks = newBlocks.toArray(new ProofBlock[]{});
	}
	
	private ProofBlock blockWithoutBelief(ProofBlock block, String beliefId) {
		Vector<String> ids = new Vector<String>();
		for (int i = 0; i < block.assertedBeliefIds.length; i++) {
			if (!block.assertedBeliefIds[i].equals(beliefId)) {
				ids.add(block.assertedBeliefIds[i]);
			}
		}
		if (ids.size() > 0) {
			ProofBlock newBlock = new ProofBlock();
			newBlock.intention = block.intention;
			newBlock.assertedBeliefIds = ids.toArray(new String[]{});
			return newBlock;
		}
		else {
			return null;
		}
	}

	public boolean beliefPresent(String beliefId) {
		for (int i = 0; i < blocks.length; i++) {
			for (int j = 0; j < blocks[i].assertedBeliefIds.length; j++) {
				if (blocks[i].assertedBeliefIds[j].equals(beliefId)) {
					return true;
				}
			}
		}
		return false;
	}
	
	/**
	 * Inspect the top proof block on the stack (which remains there).
	 * 
	 * @return the proof block, null if the stack is empty
	 */
	public ProofBlock atTop() {
		if (isEmpty()) {
			return null;
		}
		else {
			return blocks[0];
		}
	}
	
	/**
	 * Boolean indicating whether the stack is empty or not.
	 * 
	 * @return true iff the stack is empty
	 */
	public boolean isEmpty() {
		return (blocks.length == 0);
	}
	
}
