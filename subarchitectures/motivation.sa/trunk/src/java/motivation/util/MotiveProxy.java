/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package motivation.util;

import motivation.slice.Motive;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 *
 */
public interface MotiveProxy<T extends Motive>  {
	public String getGoal();
	public T getMotive();
}
