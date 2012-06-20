/**
 * 
 */
package castutils.castextensions.wmeditor.serializer;

import java.io.Reader;

import org.yaml.snakeyaml.DumperOptions;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.DumperOptions.FlowStyle;

import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import eu.cogx.beliefs.slice.GroundedBelief;

/**
 * @author cogx
 * 
 */
public class YAMLSerializer implements Serializer {

	final private Yaml yaml;

	public YAMLSerializer() {
		DumperOptions dumperOptions = new DumperOptions();
		dumperOptions.setDefaultFlowStyle(FlowStyle.BLOCK);
		dumperOptions.setIndent(4);
		dumperOptions.setWidth(1000);
		yaml = new Yaml(dumperOptions);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see castutils.castextensions.wmeditor.Serializer#dump(Ice.Object)
	 */
	@Override
	public String dump(Object obj) {
		return yaml.dump(obj);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see castutils.castextensions.wmeditor.Serializer#load(java.lang.String)
	 */
	@Override
	public Object load(String str) {
		return (Object) yaml.load(str);
	}

	public static void main(String[] args) {
		Yaml yaml = new Yaml();

		IndependentFormulaDistributionsBelief<GroundedBelief> gb = IndependentFormulaDistributionsBelief
				.create(GroundedBelief.class);
		gb.getContent().put("test", FormulaDistribution.create());
		gb.getContent().get("test").add("stupid value", 0.9);
		gb.getContent().get("test").add(true, 0.1);
		String voDump = yaml.dump(gb.get());
		Object reread = yaml.load(voDump);

		System.out.println(voDump);
		System.out.println(reread.getClass().getName());

		// XStream xstream = new XStream();
		// String xml=xstream.toXML(gb.get());
		// System.out.println(xml);
	}

	@Override
	public Object load(Reader reader) {
		return (Object) yaml.load(reader);
	}

}
