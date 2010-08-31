/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.event;

public class PhysicalEntityID extends StringID {

    public static final String physicalEntityPrefix = "physicalEntity-";

    public PhysicalEntityID(PhysicalEntityID eid) {
	super(eid);
    }

    public PhysicalEntityID(String id) {
	super(id);
    }

    public PhysicalEntityID(long id) {
	super(physicalEntityPrefix + id);
    }

}
