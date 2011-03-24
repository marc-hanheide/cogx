package castutils.castextensions;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Arrays;
import java.util.LinkedList;

import cast.architecture.ChangeFilterFactory;
import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryChangeFilter;
import cast.cdl.WorkingMemoryOperation;

public class FilterConfiguration extends LinkedList<WorkingMemoryChangeFilter>{
	private static final long serialVersionUID = -1375060739519685081L;

	public FilterConfiguration() {}
	
	public FilterConfiguration(String filename) {
		try {
			BufferedReader in = new BufferedReader(new FileReader(filename));
			String line = in.readLine();
			while (line != null) {
				String[] split = line.split("#");
				line = split[0].trim();
				if (!line.isEmpty()) {
					String[] elems = line.split(" ");
					add(createFilter(elems));
				}
				line = in.readLine();
			}
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		 
	}
	 
	private WorkingMemoryChangeFilter createFilter(String[] elems) {
		String[] fields = new String[6];
		Arrays.fill(fields, "*");
		for (int i=0; i< elems.length; i++) {
			fields[i] = elems[i].trim();
			System.out.println(elems[i]);
		}
		String cls = "";
		WorkingMemoryOperation op = WorkingMemoryOperation.WILDCARD;
		String src = "";
		String id = "";
		String sa = "";
		FilterRestriction restriction = FilterRestriction.LOCALSA;
		if (!fields[0].equals("*")) {
			cls = fields[0];
		}
		if (!fields[1].equals("*")) {
			op = WorkingMemoryOperation.valueOf(fields[1]);
		}
		if (!fields[2].equals("*")) {
			src = fields[2];
		}
		if (!fields[3].equals("*")) {
			id = fields[3];
		}
		if (!fields[4].equals("*")) {
			sa = fields[4];
		}
		if (!fields[5].equals("*")) {
			restriction = FilterRestriction.valueOf(fields[5]);
		}
		return ChangeFilterFactory.createChangeFilter(cls, op, src, id, sa, restriction);
		
	}
}
