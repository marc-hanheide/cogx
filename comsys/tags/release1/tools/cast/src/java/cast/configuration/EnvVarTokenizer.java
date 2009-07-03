/**
 * 
 */
package cast.configuration;

import java.util.StringTokenizer;

/**
 * 
 * Subclass of {@link StringTokenizer} that replaces all tokens starting with $
 * with the matching environment variable.
 * 
 * @author nah
 * 
 */
public class EnvVarTokenizer extends StringTokenizer {

	/**
	 * 
	 * @param _str
	 */
	public EnvVarTokenizer(String _str) {
		super(_str);
	}

	/**
	 * @param _str
	 * @param _delim
	 */
	public EnvVarTokenizer(String _str, String _delim) {
		super(_str, _delim);
	}

	/**
	 * @param _str
	 * @param _delim
	 * @param _returnDelims
	 */
	public EnvVarTokenizer(String _str, String _delim, boolean _returnDelims) {
		super(_str, _delim, _returnDelims);
	}

	
	
	@Override
	public String nextToken() {
		return envVar(super.nextToken());
	}

	private String envVar(String _token) {

		if(_token.charAt(0) == '$') {
			String var = System.getenv(_token.substring(1));
			if(var == null) {
				throw new RuntimeException("Undefined environment variable: " + _token);
			}
			return var;
		}
		else {
			return _token;
		}
	}

	@Override
	public String nextToken(String _delim) {
		return envVar(super.nextToken(_delim));
	}

}
