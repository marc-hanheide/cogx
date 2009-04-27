/**
 * 
 */
package cast.architecture;

import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChangeFilter;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

/**
 * 
 * Helper functions to create change filters for passing to
 * {@link WorkingMemoryReaderProcess}.addChangerFilter. If you primarily care
 * about the type of the data look at the *TypeFilter* methods. If you primarily
 * care about the operation being performed look at the createOperationFilter
 * methods. For the id or address of the data, look at the createID and
 * createAddress methods. And for the component that changed the data, look at
 * createSourceFilter.
 * 
 * @author nah
 * 
 */
public class ChangeFilterFactory {

	public static WorkingMemoryChangeFilter createAddressFilter(String _id,
			String _subarch) {
		return createAddressFilter(_id, _subarch,
				WorkingMemoryOperation.WILDCARD);
	}

	public static WorkingMemoryChangeFilter createAddressFilter(String _id,
			String _subarch, WorkingMemoryOperation _operation) {
		return createChangeFilter("", _operation, "", _id, _subarch,
		// this can be refined
				// later in the actual
				// component actually
				// subarch local
				FilterRestriction.ALLSA);
	}

	/**
	 * Create a filter that listens to changes to the given id in the local
	 * subarchitecture.
	 */
	public static WorkingMemoryChangeFilter createAddressFilter(
			WorkingMemoryAddress _wma) {
		return createAddressFilter(_wma, WorkingMemoryOperation.WILDCARD);
	}

	/**
	 * Create a filter that listens to changes to the given id in the local
	 * subarchitecture. Optional parameter provides the operation to listen for.
	 */
	public static WorkingMemoryChangeFilter createAddressFilter(
			WorkingMemoryAddress _wma, WorkingMemoryOperation _operation) {
		return createAddressFilter(_wma.id, _wma.subarchitecture,
				_operation);
	}

	/**
	 * Create an arbitrary change filter.
	 */
	public static <Type extends Ice.Object> WorkingMemoryChangeFilter createChangeFilter(
			Class<Type> _cls, WorkingMemoryOperation _op, String _src,
			String _changeID, String _changeSA, FilterRestriction _restriction) {

		String type = CASTUtils.typeName(_cls);
		return createChangeFilter(type, _op, _src, _changeID, _changeSA,
				_restriction);

	}

	/**
	 * Create an arbitrary change filter.
	 */

	public static WorkingMemoryChangeFilter createChangeFilter(String _type,
			WorkingMemoryOperation _op, String _src, String _changeID,
			String _changeSA, FilterRestriction _restriction) {
		return new WorkingMemoryChangeFilter(_op, _src,
				new WorkingMemoryAddress(_changeID, _changeSA), _type,
				_restriction, "");

	}

	/**
	 * Create a filter that matches all operations on the given type across the
	 * architecture
	 */
	public static <Type extends Ice.Object> WorkingMemoryChangeFilter createGlobalTypeFilter(
			Class<Type> _cls) {
		return createGlobalTypeFilter(_cls, WorkingMemoryOperation.WILDCARD);
	}

	/**
	 * Create a filter that matches all operations on the given type across the
	 * architecture
	 */
	public static <Type extends Ice.Object> WorkingMemoryChangeFilter createGlobalTypeFilter(
			Class<Type> _cls, WorkingMemoryOperation _operation) {
		return createTypeFilter(_cls, _operation, FilterRestriction.ALLSA);
	}

	/**
	 * Create a filter that listens to changes to the given id in the local
	 * subarchitecture.
	 */
	public static WorkingMemoryChangeFilter createIDFilter(String _id) {
		return createAddressFilter(_id, "", WorkingMemoryOperation.WILDCARD);
	}

	/**
	 * Create a filter that listens to changes to the given id in the local
	 * subarchitecture. Optional parameter provides the operation to listen for.
	 */
	public static WorkingMemoryChangeFilter createIDFilter(String _id,
			WorkingMemoryOperation _operation) {
		return createAddressFilter(_id, "", _operation);
	}

	/**
	 * Create a filter that matches all operations on the given type in the
	 * component's subarchitecture.
	 */
	public static <Type extends Ice.Object> WorkingMemoryChangeFilter createLocalTypeFilter(
			Class<Type> _cls) {
		return createLocalTypeFilter(_cls, WorkingMemoryOperation.WILDCARD);
	}

	/**
	 * Create a filter that matches all operations on the given type in the
	 * component's subarchitecture.
	 */
	public static <Type extends Ice.Object> WorkingMemoryChangeFilter createLocalTypeFilter(
			Class<Type> _cls, WorkingMemoryOperation _operation) {
		return createTypeFilter(_cls, _operation, FilterRestriction.LOCALSA);
	}

	/**
	 * Create a filter that listens to all untyped operations in the local sa
	 */
	public static WorkingMemoryChangeFilter createOperationFilter(
			WorkingMemoryOperation _operation) {
		return createChangeFilter("", _operation, "", "", "",
				FilterRestriction.LOCALSA);
	}

	/**
	 * Create a filter that listens to all untyped operations.
	 */
	public static WorkingMemoryChangeFilter createOperationFilter(
			WorkingMemoryOperation _operation, FilterRestriction _restriction) {
		return createChangeFilter("", _operation, "", "", "", _restriction);
	}

	/**
	 * Create a filter that listens to changes of a type made by a particular
	 * component.
	 */

	public static <Type extends Ice.Object> WorkingMemoryChangeFilter createSourceFilter(
			Class<Type> _cls, String _component) {
		return createChangeFilter(_cls, WorkingMemoryOperation.WILDCARD,
				_component, "", "", FilterRestriction.ALLSA);
	}

	/**
	 * Create a filter that listens to changes of a type made by a particular
	 * component. Optional parameter provides the operation to listen for.
	 */

	public static <Type extends Ice.Object> WorkingMemoryChangeFilter createSourceFilter(
			Class<Type> _cls, String _component,
			WorkingMemoryOperation _operation) {
		return createChangeFilter(_cls, _operation, _component, "", "",
				FilterRestriction.ALLSA);
	}

	/**
	 * Create a filter that listens to changes of made by a particular
	 * component.
	 */
	public static WorkingMemoryChangeFilter createSourceFilter(String _component) {
		return createChangeFilter("", WorkingMemoryOperation.WILDCARD,
				_component, "", "", FilterRestriction.ALLSA);
	}

	/**
	 * Create a filter that listens to changes of made by a particular
	 * component. Optional parameter provides the operation to listen for.
	 */
	public static WorkingMemoryChangeFilter createSourceFilter(
			String _component, WorkingMemoryOperation _operation) {
		return createChangeFilter("", _operation, _component, "", "",
				FilterRestriction.ALLSA);
	}

	/**
	 * Create a filter that matches all operations on the given type using the
	 * given restriction.
	 */
	public static <Type extends Ice.Object> WorkingMemoryChangeFilter createTypeFilter(
			Class<Type> _cls, FilterRestriction _restriction) {
		return createTypeFilter(_cls, WorkingMemoryOperation.WILDCARD,
				_restriction);
	}

	/**
	 * Create a filter that matches the given type using the given operation.
	 */
	public static <Type extends Ice.Object> WorkingMemoryChangeFilter createTypeFilter(
			Class<Type> _cls, WorkingMemoryOperation _operation) {
		return createTypeFilter(_cls, _operation, FilterRestriction.ALLSA);
	}

	/**
	 * Create a filter that matches the given type using the given operation and
	 * restriction
	 */
	public static <Type extends Ice.Object> WorkingMemoryChangeFilter createTypeFilter(
			Class<Type> _cls, WorkingMemoryOperation _operation,
			FilterRestriction _restriction) {
		return createChangeFilter(_cls, _operation, "", "", "", _restriction);
	}

}
