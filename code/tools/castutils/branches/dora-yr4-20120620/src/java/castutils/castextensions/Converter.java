package castutils.castextensions;

/**
 * Converts one object into another.
 * 
 * @author nah
 * 
 * @param <InputT>
 * @param <OutputT>
 */
public interface Converter<InputT, OutputT> {
	OutputT convert(InputT _o);
}
