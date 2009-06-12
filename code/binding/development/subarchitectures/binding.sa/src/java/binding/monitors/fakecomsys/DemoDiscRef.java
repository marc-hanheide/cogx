package binding.monitors.fakecomsys;

import java.util.TreeSet;

public class DemoDiscRef {
    public TreeSet<String> colours;
    public String size;
    public String shape;
    public String noun;
    public String relation;
    public boolean colours_truth_value;
    public boolean size_truth_value;
    public boolean shape_truth_value;
    public boolean noun_truth_value;
    public boolean relationForward;
    public boolean hypothetical;

    public DemoDiscRef() {
	colours = new TreeSet<String>();
	size = "";
	shape = "";
	noun = "";
	relation = "";
	colours_truth_value = true;
	size_truth_value = true;
	shape_truth_value = true;
	noun_truth_value = true;
	relationForward = true;
	hypothetical = false;
    }
}
