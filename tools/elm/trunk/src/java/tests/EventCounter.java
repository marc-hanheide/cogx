/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.tests;

public interface EventCounter {

    public long getEventCnt();
    public void resetEventCnt();
    
    public long getLastEventID();
    public void resetLastEventID();
}
