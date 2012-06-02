package cdsr.objects;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.List;

public class AnnotatedCDSR extends CDSR{

	private final String m_room;
	private final String m_filename;
	private final String m_subject;
	private final Point2D.Double m_sweetspot;
	
	public AnnotatedCDSR(String _subject, String _room, String _filename, 
			List<Line2D.Double> _lines, String _type, Point2D.Double _sweetspot)
	{
		super(_lines, _type);
		m_room = _room;
		m_filename = _filename;
		m_sweetspot = _sweetspot;
		m_subject = _subject;
	}
	
	public String getSubject() {
		return m_subject;
	}

   public Point2D.Double getSweetspot() {
	   return m_sweetspot;
   }

	public String getRoom() {
		return m_room;
	}

	public String getFilename() {
		return m_filename;
	}
	
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append(m_subject).append(", ");
		sb.append(m_room).append(", ");
		sb.append(getType()).append("\n");
		sb.append(super.toString());
		sb.append("Sweetspot = ").append(m_sweetspot);
		return sb.toString();
	}
	

}
