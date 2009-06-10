package binding.monitors.fakecomsys;

import java.util.TreeSet;
import BindingFeaturesCommon.TruthValue;


public class DemoDiscRef {
    public TreeSet<String> m_colours;
    public String m_size;
    public String m_shape;
    public String m_noun;
    public String m_relation;
    public TruthValue m_colours_truth_value;
    public TruthValue m_size_truth_value;
    public TruthValue m_shape_truth_value;
    public TruthValue m_noun_truth_value;
    public boolean m_relationForward;
    public boolean m_hypothetical;

    public DemoDiscRef() {
	m_colours = new TreeSet<String>();
	m_size = "";
	m_shape = "";
	m_noun = "";
	m_relation = "";
	m_colours_truth_value = TruthValue.POSITIVE;
	m_size_truth_value = TruthValue.POSITIVE;
	m_shape_truth_value = TruthValue.POSITIVE;
	m_noun_truth_value = TruthValue.POSITIVE;
	m_relationForward = true;
	m_hypothetical = false;
    }
}
