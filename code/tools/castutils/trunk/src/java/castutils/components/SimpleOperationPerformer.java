/**
 * 
 */
package castutils.components;

import java.util.Map;

import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

/**
 * 
 * Configured with an input working memory operation (default add), an output
 * operation (default delete) and a class. When it sees an opertion of the input
 * type on the class it perform the output type.
 * 
 * @author nah
 * 
 */
public class SimpleOperationPerformer extends ManagedComponent {

	private final static String INPUT_OPERATION_KEY = "--input-operation";
	private final static String OUTPUT_OPERATION_KEY = "--output-operation";
	private final static String CLASS_KEY = "--class";

	private final static WorkingMemoryOperation DEFAULT_INPUT_OPERATION = WorkingMemoryOperation.ADD;
	private final static WorkingMemoryOperation DEFAULT_OUTPUT_OPERATION = WorkingMemoryOperation.DELETE;

	private WorkingMemoryOperation m_inputOperation = DEFAULT_INPUT_OPERATION;
	private WorkingMemoryOperation m_outputOperation = DEFAULT_OUTPUT_OPERATION;

	private Class<? extends Ice.Object> m_class;

	@SuppressWarnings("unchecked")
	@Override
	protected void configure(Map<String, String> _config) {

		String classString = _config.get(CLASS_KEY);
		if (classString == null) {
			throw new RuntimeException("Missing required config parameter: "
					+ CLASS_KEY);
		}
		try {
			m_class = (Class<? extends Ice.Object>) Class.forName(classString);
		} catch (ClassNotFoundException e) {
			throw new RuntimeException(e);
		}

		String inputOperationString = _config.get(INPUT_OPERATION_KEY);
		if (inputOperationString != null) {
			m_inputOperation = WorkingMemoryOperation
					.valueOf(inputOperationString);
		}

		String outputOperationString = _config.get(OUTPUT_OPERATION_KEY);
		if (outputOperationString != null) {
			m_outputOperation = WorkingMemoryOperation
					.valueOf(outputOperationString);
		}

		// try to avoid dumb behaviour please...
		assert (m_inputOperation == WorkingMemoryOperation.ADD || m_inputOperation == WorkingMemoryOperation.OVERWRITE);
		assert (m_outputOperation == WorkingMemoryOperation.DELETE || m_outputOperation == WorkingMemoryOperation.OVERWRITE);
		assert (m_inputOperation != m_outputOperation);

	}

	@Override
	protected void start() {

		if (m_outputOperation == WorkingMemoryOperation.OVERWRITE) {
			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(m_class,
					m_inputOperation), new WorkingMemoryChangeReceiver() {

				@Override
				public void workingMemoryChanged(WorkingMemoryChange _arg0)
						throws CASTException {
					log("performing overwrite on " + m_class + " at "
							+ CASTUtils.toString(_arg0.address) + " after " + m_inputOperation);
					overwriteWorkingMemory(_arg0.address,
							getMemoryEntry(_arg0.address, m_class));
				}
			});
		} else if (m_outputOperation == WorkingMemoryOperation.DELETE) {
			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(m_class,
					m_inputOperation), new WorkingMemoryChangeReceiver() {

				@Override
				public void workingMemoryChanged(WorkingMemoryChange _arg0)
						throws CASTException {
					log("performing delete on " + m_class + " at "
							+ CASTUtils.toString(_arg0.address) + " after " + m_inputOperation);
					deleteFromWorkingMemory(_arg0.address);
				}
			});
		}
	}

}
