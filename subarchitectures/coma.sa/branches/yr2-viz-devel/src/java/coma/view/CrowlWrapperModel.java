package coma.view;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Properties;
import java.util.Vector;

import com.hp.hpl.jena.query.QuerySolution;
import com.hp.hpl.jena.query.ResultSet;

import coma.model.ComaModel;
import coma.reasoning.CrowlWrapper;

public class CrowlWrapperModel implements ComaModel {

	private CrowlWrapper cw;
	private Properties config;

	public CrowlWrapperModel(CrowlWrapper cw, Properties config) {
		this.config = config;
		this.cw = cw;
	}

	@Override
	public List<String> getEntity(String concept) {
		String query = "select $x {$x rdf:type " + concept + "}";
		ResultSet results = cw.executeSPARQLQuery(query);
		if (results == null)
			return null;
		LinkedList<String> entities = new LinkedList<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			entities.add(handelURI(_qs.getResource("x").toString()));
		}
		if (entities.isEmpty())
			return null;
		return entities;
	}

	@Override
	public Vector<String> getObjectProperties() {
		Vector<String> entities = new Vector<String>();
		String query = "select $x {$x rdf:type owl:ObjectProperty}";
		ResultSet results = cw.executeSPARQLQuery(query);
		if (results == null)
			return entities;
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			entities.add(handelURI(_qs.getResource("x").toString()));
		}
		return entities;
	}

	@Override
	public List<String> getObjects(String subject, String property) {

		String query = "select $x { " + subject + " " + property + " $x}";
		ResultSet results = cw.executeSPARQLQuery(query);
		if (results == null)
			return null;
		LinkedList<String> entities = new LinkedList<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			entities.add(handelURI(_qs.getResource("x").toString()));
		}
		if (entities.isEmpty())
			return null;
		return entities;
	}

	@Override
	public List<String> getSubClass(String concept) {

		String query = "select $x {$x rdfs:subClassOf " + concept + "}";
		ResultSet results = cw.executeSPARQLQuery(query);
		if (results == null)
			return null;
		LinkedList<String> entities = new LinkedList<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			String con = handelURI(_qs.getResource("x").toString());
			if (con != null)
				entities.add(con);
		}
		if (entities.isEmpty())
			return null;

		HashSet<String> toRemoved = new HashSet<String>();
		for (String subCon : entities) {

			String subquery = "select $x { " + subCon
					+ " rdfs:subClassOf $x.\n" + "$x rdfs:subClassOf "
					+ concept + " }";
			ResultSet subresults = cw.executeSPARQLQuery(subquery);
			if (subresults == null)
				continue;
			while (subresults.hasNext()) {
				QuerySolution sub_qs = (QuerySolution) subresults.next();
				String transCon = handelURI(sub_qs.getResource("x").toString());
				if (transCon == null)
					continue;
				if (!toRemoved.contains(subCon) && !transCon.equals(concept)
						&& !transCon.equals(subCon))
					toRemoved.add(subCon);
			}
		}
		for (String remove : toRemoved)
			entities.remove(remove);
		return entities;
	}

	@Override
	public List<String> getSubjects(String object, String property) {

		String query = "select $x {$x " + property + " " + object + " }";
		ResultSet results = cw.executeSPARQLQuery(query);
		if (results == null)
			return null;
		LinkedList<String> entities = new LinkedList<String>();
		while (results.hasNext()) {
			QuerySolution _qs = (QuerySolution) results.next();
			entities.add(handelURI(_qs.getResource("x").toString()));
		}
		if (entities.isEmpty())
			return null;
		return entities;
	}
	
	public String handelURI(String uri) {
		String[] ret = uri.split("#");
		if (ret.length == 2) {
			return config.getProperty(ret[0]) + ":" + ret[1];
		} else {
			return ret[0];
		}

	}
}

