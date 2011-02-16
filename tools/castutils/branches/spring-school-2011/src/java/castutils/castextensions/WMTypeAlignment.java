package castutils.castextensions;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import Ice.Object;
import cast.architecture.WorkingMemoryWriterComponent;
import cast.cdl.WorkingMemoryAddress;
import castutils.slice.WMAdd;
import castutils.slice.WMDelete;
import castutils.slice.WMOperation;
import castutils.slice.WMOverwrite;

/**
 * Given 2 types, this class will output operations to WM that keeps the output
 * type present on working memory to reflect the presence of absence of the
 * input type in the input data. It uses the hashCode() method of the input
 * objects to determine identity.
 * 
 * @author nah
 * 
 * @param <InputT>
 * @param <OutputT>
 */
public class WMTypeAlignment<InputT, OutputT extends Object> {

	private final Converter<InputT, OutputT> m_converter;
	private final HashCoder<InputT> m_hasher;
	private HashMap<Integer, WorkingMemoryAddress> m_previousInput;
	private final WorkingMemoryWriterComponent m_component;
	private final Condition<InputT> m_filter;

	/**
	 * 
	 * @param _component
	 *            Component. Only used for newDataID() and getSubarchitecture().
	 * @param _converter
	 *            Converts InputT to OutputT.
	 * @param _filter
	 *            Alignment is only performed when this evaluates to true, or is
	 *            null.
	 * @param _hasher
	 *            Generates unique ids for InputT.
	 */
	public WMTypeAlignment(WorkingMemoryWriterComponent _component,
			Converter<InputT, OutputT> _converter, Condition<InputT> _filter,
			HashCoder<InputT> _hasher) {
		m_component = _component;
		m_converter = _converter;
		m_filter = _filter;
		m_hasher = _hasher;
		m_previousInput = new HashMap<Integer, WorkingMemoryAddress>();
	}

	/**
	 * Creates a new alignment object with no filter and using the hashCode of
	 * the input type.
	 * 
	 * @param _component
	 *            Component. Only used for newDataID() and getSubarchitecture().
	 * @param _converter
	 *            Converts InputT to OutputT.
	 */
	public WMTypeAlignment(WorkingMemoryWriterComponent _component,
			Converter<InputT, OutputT> _converter) {
		this(_component, _converter, null, new DefaultHashCoder<InputT>());
	}

	public List<WMOperation> sync(List<InputT> _input) {

		ArrayList<WMOperation> operations = new ArrayList<WMOperation>(_input
				.size());

		HashMap<Integer, WorkingMemoryAddress> newInput = new HashMap<Integer, WorkingMemoryAddress>(
				_input.size());

		for (InputT inputObject : _input) {

			// if we're not filtering, or if we're filtering and the input tests
			// true
			if (m_filter == null || m_filter.test(inputObject)) {

				Integer hash = m_hasher.hashCode(inputObject);
				WorkingMemoryAddress address = null;

				// if it's a new object
				if (!m_previousInput.containsKey(hash)) {
//					System.out.println("new object");
					address = new WorkingMemoryAddress(m_component.newDataID(),
							m_component.getSubarchitectureID());
					operations.add(new WMAdd(address, m_converter
							.convert(inputObject)));
				}
				// else we've seen it before
				else {
//					System.out.println("updated object");
					address = m_previousInput.get(hash);
					operations.add(new WMOverwrite(address, m_converter
							.convert(inputObject)));
				}
				newInput.put(hash, address);
			}
		}

		for (Integer hash : m_previousInput.keySet()) {
			// if the new input no longer has this item, delete it
			if (!newInput.containsKey(hash)) {
				operations.add(new WMDelete(m_previousInput.get(hash)));
			}
		}

		m_previousInput = newInput;

		return operations;
	}
}
