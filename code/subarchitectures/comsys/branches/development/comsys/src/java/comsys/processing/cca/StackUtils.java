package comsys.processing.cca;

import comsys.datastructs.comsysEssentials.*;
import Abducer.*;

public class StackUtils {

	/** Construct a proof block based on an abductive proof.
	 * 
	 * @param proof
	 * @param proofId
	 * @return
	 */
	public static ProofBlock construct(MarkedQuery[] proof, String proofId) {
		// TODO
		return null;
	}
	
	/** Push the proof block onto the stack.
	 * 
	 * @param stack
	 * @param block
	*/
	public static void push(ProofBlock[] stack, ProofBlock block) {
		ProofBlock[] newStack = new ProofBlock[stack.length+1];
		newStack[0] = block;
		for (int i = 0; i < stack.length; i++) {
			newStack[i+1] = stack[i];
		}
		stack = newStack;
	}
	
	/** Pop the top proof block from the stack.
	 * 
	 * @param stack
	 * @return the proof block, null if the stack is empty
	 */
	public static ProofBlock pop(ProofBlock[] stack) {
		if (isEmpty(stack)) {
			return null;
		}

		ProofBlock[] newStack = new ProofBlock[stack.length-1];
		ProofBlock result = stack[0];
		for (int i = 1; i < stack.length; i++) {
			newStack[i-1] = stack[i];
		}
		stack = newStack;
		return result;
	}
	
	/** Inspect the top proof block on the stack (which remains there).
	 * 
	 * @param stack
	 * @return the proof block, null if the stack is empty
	 */
	public static ProofBlock atTop(ProofBlock[] stack) {
		if (isEmpty(stack)) {
			return null;
		}
		else {
			return stack[0];
		}
	}
	
	/** Boolean indicating whether the stack is empty or not.
	 * 
	 * @param stack
	 * @return true iff the stack is empty
	 */
	public static boolean isEmpty(ProofBlock[] stack) {
		return (stack.length == 0);
	}
	
	/** Retrieve the proof block with the given proofID; the proof block remains on stack.
	 * 
	 * @param stack
	 * @param proofId
	 * @return the proof block, null if such a proof block is not present
	 */
	public static ProofBlock inspect(ProofBlock[] stack, String proofId) {
		for (int i = 0; i < stack.length; i++) {
			if (stack[i].proofId == proofId)
				return stack[i];
		}
		return null;
	}
	
	/** Retrieve the top-most proof block which contains an asserted belief (DeltaSet) with the given
	 *  beliefID; the proof block remains on stack.
	 *
	 * @param stack
	 * @param beliefId
	 * @return the proof block, null if such a proof block is not present
	 */
	public static ProofBlock inspectAsserted(ProofBlock[] stack, String beliefId) {
		for (int i = 0; i < stack.length; i++) {
			for (int j = 0; j < stack[i].assertions.length; j++) {
				if (stack[i].assertions[j].id == beliefId) {
					return stack[i];
				}
			}
		}
		return null;
	}

	/** Retrieve the top-most proof block which contains an assumed belief (DeltaSet) with the given
	 *  beliefID; the proof block remains on stack.
	 * @param stack
	 * @param beliefId
	 * @return
	 */
	public static ProofBlock inspectAssumed(ProofBlock[] stack, String beliefId) {
		for (int i = 0; i < stack.length; i++) {
			for (int j = 0; j < stack[i].assumptions.length; j++) {
				if (stack[i].assumptions[j].id == beliefId) {
					return stack[i];
				}
			}
		}
		return null;
	}
	
	/** Retrieve the proof block with the given proofID; the proof block is removed from the stack.
	 * 
	 * @param stack
	 * @param proofId
	 * @return the proof block, null if such a proof block is not present
	 */
	public static ProofBlock retrieve(ProofBlock[] stack, String proofId) {
		ProofBlock result = null;
		ProofBlock[] newStack = new ProofBlock[stack.length-1];
		int j = 0;
		for (int i = 0; i < stack.length; i++) {
			if (stack[i].proofId == proofId) {
				assert result == null; // not null -> duplicit ids
				result = stack[i];
			}
			else {
				newStack[j] = stack[i];
				j++;
			}
		}
		stack = newStack;
		return result;
	}
	
	/** Retrieve the top-most proof block which contains an asserted belief (DeltaSet) with the given
	 *  beliefID; the proof block is removed from the stack.
	 *  
	 *  FIXME: will fail array bounds check when no such proof block exists!
	 * 
	 * @param stack the stack
	 * @param beliefId id of the belief
	 * @return the proof block, TODO null if no such block exists
	 */
	public static ProofBlock retrieveAsserted(ProofBlock[] stack, String beliefId) {
		ProofBlock[] newStack = new ProofBlock[stack.length-1];
		ProofBlock result = null;
		int k = 0;
		for (int i = 0; i < stack.length; i++) {
			if (result == null) {
				for (int j = 0; j < stack[i].assertions.length; j++) {
					if (stack[i].assertions[j].id == beliefId) {
						assert result == null; // not null -> duplicit ids in the delta set
						result = stack[i];
						i++;
					}
				}
			}
			newStack[k++] = stack[i];
		}
		stack = newStack;
		return result;
	}

	/** Retrieve the top-most proof block which contains an assumed belief (DeltaSet) with the given
	 *  beliefID; the proof block is removed from the stack.
	 *  
	 *  FIXME: will fail array bounds check when no such proof block exists!
	 * 
	 * @param stack
	 * @param beliefId
	 * @return the proof block, TODO null if no such block exists
	 */
	public static ProofBlock retrieveAssumed(ProofBlock[] stack, String beliefId) {
		ProofBlock[] newStack = new ProofBlock[stack.length-1];
		ProofBlock result = null;
		int k = 0;
		for (int i = 0; i < stack.length; i++) {
			if (result == null) {
				for (int j = 0; j < stack[i].assumptions.length; j++) {
					if (stack[i].assumptions[j].id == beliefId) {
						assert result == null; // not null -> duplicit ids in the delta set
						result = stack[i];
						i++;
					}
				}
			}
			newStack[k++] = stack[i];
		}
		stack = newStack;
		return result;
	}
	
}
