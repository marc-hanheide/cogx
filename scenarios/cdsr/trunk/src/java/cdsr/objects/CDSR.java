package cdsr.objects;

import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class CDSR implements Iterable<Line2D.Double> {

	private final List<Line2D.Double> m_lines;
	private final String m_type;
	
	public CDSR(List<Line2D.Double> _lines, String _type) {
		m_lines = _lines;
		m_type = _type;
	}

//	public CDSR() {
//		this(new ArrayList<Line2D.Double>());
//	}

	@Override
	public Iterator<Line2D.Double> iterator() {
		return m_lines.iterator();
	}

	public void addLine(Line2D.Double _line) {
		m_lines.add(_line);
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		for (Line2D.Double line : this) {
			sb.append(line.x1);
			sb.append(",");
			sb.append(line.y1);
			sb.append(",");
			sb.append(line.x2);
			sb.append(",");
			sb.append(line.y2);
			sb.append("\n");
		}
		return sb.toString();
	}
}
