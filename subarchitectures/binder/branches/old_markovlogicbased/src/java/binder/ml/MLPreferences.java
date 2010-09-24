package binder.ml;

import java.util.LinkedList;
import java.util.List;

public class MLPreferences {
	
	private String path_to_alchemy;
	private String path_to_input;
	private String path_to_output;
	private String path_to_correlations;
	private String path_to_db;
	
	public MLPreferences() {
		path_to_alchemy = "/Users/djan/DFKI/spring-school-2010/tools/alchemy/bin/./infer";
		path_to_input = "/Users/djan/DFKI/spring-school-2010/test.mln";
		path_to_output = "/Users/djan/DFKI/spring-school-2010/result.txt";
		path_to_correlations = "/Users/djan/DFKI/spring-school-2010/correlations.txt";
		path_to_db = "/Users/djan/DFKI/spring-school-2010/empty.db";
	}
	
	public MLPreferences(String pathToAlchemy, String pathToInput,
			String pathToOutput, String pathToCorrelations, String pathToDb) {
		path_to_alchemy = pathToAlchemy;
		path_to_input = pathToInput;
		path_to_output = pathToOutput;
		path_to_correlations = pathToCorrelations;
		path_to_db = pathToDb;
	}

	public String getPath_to_alchemy() {
		return path_to_alchemy;
	}

	public void setPath_to_alchemy(String pathToAlchemy) {
		path_to_alchemy = pathToAlchemy;
	}

	public String getPath_to_input() {
		return path_to_input;
	}

	public void setPath_to_input(String pathToInput) {
		path_to_input = pathToInput;
	}

	public String getPath_to_output() {
		return path_to_output;
	}

	public void setPath_to_output(String pathToOutput) {
		path_to_output = pathToOutput;
	}

	public String getPath_to_correlations() {
		return path_to_correlations;
	}

	public void setPath_to_correlations(String pathToCorrelations) {
		path_to_correlations = pathToCorrelations;
	}

	public List<String> getParameters() {
		// TODO: add more functionality here
		// currently we can only do belief propagation
		List<String> params = new LinkedList<String>();
		params.add("");
		return params;
	}

	public String getPath_to_db() {
		return path_to_db;
	}
}
