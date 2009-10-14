package comsys.processing.cca;

import Abducer.*;
import comsys.datastructs.comsysEssentials.*;
import beliefmodels.adl.*;
import beliefmodels.domainmodel.cogx.*;
import binder.utils.BeliefModelUtils;

import comsys.processing.cca.ProofUtils;

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
	public static ProofBlock construct(ContextUpdate cu) {
		System.err.println("stack.construct()");
		ProofBlock pb = new ProofBlock();
		pb.intention = cu.intention;
		pb.assertedBeliefIds = beliefIds(cu.beliefs);
		return pb;
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
