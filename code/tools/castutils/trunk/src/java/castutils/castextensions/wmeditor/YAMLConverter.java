package castutils.castextensions.wmeditor;

import org.yaml.snakeyaml.Yaml;

import com.thoughtworks.xstream.XStream;

import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;

import eu.cogx.beliefs.slice.GroundedBelief;

import VisionData.VisualObject;

public class YAMLConverter {
	public static void main(String[] args) {
		Yaml yaml=new Yaml();
		
		IndependentFormulaDistributionsBelief<GroundedBelief> gb = IndependentFormulaDistributionsBelief.create(GroundedBelief.class);
		gb.getContent().put("test", FormulaDistribution.create());
		gb.getContent().get("test").add("stupid value", 0.9);
		gb.getContent().get("test").add(true, 0.1);
		String voDump=yaml.dump(gb.get());
		Object reread = yaml.load(voDump);
		
		System.out.println(voDump);
		System.out.println(reread.getClass().getName());
		
//		XStream xstream = new XStream();
//		String xml=xstream.toXML(gb.get());
//		System.out.println(xml);
	}
}
