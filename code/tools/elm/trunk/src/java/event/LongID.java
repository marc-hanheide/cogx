/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.event;

public class LongID {

    private long id = -1;

    public LongID(LongID lid) {
	id = lid.id;
    }

    public LongID(long id) {
	this.id = id;
    }

    public boolean equals(LongID lid) {
	return (lid.id == id);
    }

    public String toString() {
	return Long.toString(id);
    }

    public long getLong() {
	return id;
    }
}
