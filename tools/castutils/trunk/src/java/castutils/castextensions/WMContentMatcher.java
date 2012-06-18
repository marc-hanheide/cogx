package castutils.castextensions;

import java.util.ArrayList;
import java.util.List;
import java.util.StringTokenizer;

import org.apache.log4j.Logger;

import com.thoughtworks.xstream.XStream;

import nu.xom.Document;
import nu.xom.Nodes;
import cast.CASTException;
import cast.architecture.ManagedComponent;

/**
 * This class allows to check the working memory content of an subarchitecture
 * against a given XPATH statement. So it allows to check not only if some entry
 * of a certain does exist, but also if the <i>content</i> of the entry fulfils
 * a certain given condition. Therefore the content is serialised into the
 * standard XML representation of {@link XStream}, which is then checks to a
 * given XPATH using the {@link nu.xom.Document} implementation.
 * 
 * @author marc
 * 
 * @param <T>
 *            the ICE type we check for
 */
public class WMContentMatcher<T extends Ice.Object> extends CASTHelper {

	String sa;
	Class<T> type;
	String xpath;

	enum Qualification {
		ALL, EXISTS
	};

	Qualification qualifier = Qualification.ALL;

	public static <T2 extends Ice.Object> WMContentMatcher<T2> create(
			ManagedComponent c, Class<T2> type, String sa, String xpath,
			Qualification qual) {
		return new WMContentMatcher<T2>(c, type, sa, xpath, qual);
	}

	/**
	 * Factory method for the {@link WMContentMatcher}.
	 * 
	 * @param component
	 *            a reference to the CAST {@link ManagedComponent} through which
	 *            all WM operation are delegated.
	 * @param specs
	 *            a 'key=value' specification of the parameters for the
	 *            {@link WMContentMatcher} to be created. An example looks like
	 *            this:
	 *            <code>sa=testsa;xpath=//status[node()='TRUEPLACE'];type=eu.cogx.beliefs.slice.PerceptBelief;qualifier=EXISTS</code>
	 * @return the created instance of {@link WMContentMatcher}
	 * @throws ClassNotFoundException
	 */
	@SuppressWarnings("unchecked")
	public static <T2 extends Ice.Object> WMContentMatcher<T2> create(
			ManagedComponent component, String specs) throws ClassNotFoundException {
		StringTokenizer st = new StringTokenizer(specs, ";");
		String xpath = null;
		String sa = null;
		Class<T2> type = null;
		Qualification qual = Qualification.ALL;

		while (st.hasMoreTokens()) {
			String elem = st.nextToken();
			int eqInd = elem.indexOf("=");
			String key = elem.substring(0, eqInd);
			Logger.getLogger(WMContentMatcher.class).debug("key=" + key);
			String value = elem.substring(eqInd + 1);
			Logger.getLogger(WMContentMatcher.class).debug("value=" + value);
			if (key.equals("xpath"))
				xpath = value;
			if (key.equals("sa"))
				sa = value;
			if (key.equals("qualifier"))
				qual = Qualification.valueOf(value.toUpperCase());
			if (key.equals("type"))
				type = (Class<T2>) Class.forName(value);
		}
		return new WMContentMatcher<T2>(component, type, sa, xpath, qual);
	}

	public static void main(String[] argv) {
		try {
			WMContentMatcher<?> matcher = WMContentMatcher
					.create(null,
							"sa=testsa;xpath=//*[@prop='test'];type=eu.cogx.beliefs.slice.PerceptBelief;qualifier=EXISTS");
			System.out.println(matcher.toString());
		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@Override
	public String toString() {
		return getClass().getName() + ": sa=" + sa + ";xpath=" + xpath
				+ ";type=" + type.getName() + ";qualifier=" + qualifier;
	}

	protected WMContentMatcher(ManagedComponent c, Class<T> type, String sa,
			String xpath, Qualification qual) {
		super(c);
		this.sa = sa;
		this.type = type;
		this.xpath = xpath;
		this.qualifier = qual;
	}

	public boolean check() {
		try {
			log("query the WM of SA " + sa + " for type " + type);
			List<T> wmes = new ArrayList<T>();
			component.getMemoryEntries(type, wmes, sa);
			log("  => got " + wmes.size()
					+ " elements... now checking them all:");
			boolean matchedOne = false;
			// assumption is they all match until we prove the opposite.
			boolean matchedAll = (wmes.size() > 0);
			for (T wme : wmes) {
				Document d = IceXMLSerializer.toXomDom(wme);
				log("  check document " + d.getValue());
				Nodes res = d.query(xpath);
				if (res.size() > 0) {
					matchedOne = true;
					log("    matched xpath " + xpath);
					// break;
				} else {
					matchedAll = false;
					log("    did not match xpath " + xpath);
				}
			}
			switch (qualifier) {
			case ALL:
				return matchedAll;
			default:
			case EXISTS:
				return matchedOne;
			}
		} catch (CASTException e) {
			component.logException(e);
			return false;
		}
	}

}
