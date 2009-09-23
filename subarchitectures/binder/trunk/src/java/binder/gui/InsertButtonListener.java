// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
//                                                                                                                          
// This library is free software; you can redistribute it and/or                                                            
// modify it under the terms of the GNU Lesser General Public License                                                       
// as published by the Free Software Foundation; either version 2.1 of                                                      
// the License, or (at your option) any later version.                                                                      
//                                                                                                                          
// This library is distributed in the hope that it will be useful, but                                                      
// WITHOUT ANY WARRANTY; without even the implied warranty of                                                               
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU                                                         
// Lesser General Public License for more details.                                                                          
//                                                                                                                          
// You should have received a copy of the GNU Lesser General Public                                                         
// License along with this program; if not, write to the Free Software                                                      
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA                                                                
// 02111-1307, USA.                                                                                                         
// =================================================================                                                        

package binder.gui;

import java.awt.Component;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Enumeration;
import java.util.Vector;

import javax.swing.JPanel;
import javax.swing.JTextField;

import binder.abstr.BindingWorkingMemoryWriter;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.Proxy;
import binder.autogen.distributions.FeatureValuePair;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.autogen.featvalues.StringValue;
import binder.components.BinderMonitor;
import binder.utils.DistributionGeneration;
import binder.utils.ProbabilityUtils;

public class InsertButtonListener extends BindingWorkingMemoryWriter implements ActionListener {
	
	JPanel proxyPanel;
	BinderMonitor bm;
	
	public InsertButtonListener(JPanel proxyPanel, BinderMonitor bm) {
		this.proxyPanel = proxyPanel;
		this.bm = bm;
	}

	public void actionPerformed(ActionEvent event) {
		String proxyID = "";
		String subarch = "";
		String exists = "";
		Vector<Feature> fds = new Vector<Feature>();

		Component[] components = proxyPanel.getComponents();
		
		Vector<DiscreteProbabilityAssignment> assignments = new Vector<DiscreteProbabilityAssignment>();
		
		for (int i = 0 ; i < components.length ; i++) {
			if (components[i].getClass().equals(JPanel.class)) {
				JPanel compo = (JPanel) components[i];
				Component[] subcomponents = compo.getComponents();
				for (int j = 0 ; j < subcomponents.length ; j++) {
				
					Component subcompo = subcomponents[j];
					if (subcompo.getName() != null && subcompo.getName().equals("proxyID")) {
						proxyID = ((JTextField)subcompo).getText();
						log("Proxy ID: " + proxyID);
						((JTextField)subcompo).setText("proxyID" + ControlPanel.proxyCount);
						ControlPanel.proxyCount++;
					}
					if (subcompo.getName() != null && subcompo.getName().equals("subarch")) {
						subarch = ((JTextField)subcompo).getText();
						log("subarch: " + subarch);
					}
					if (subcompo.getName() != null && subcompo.getName().equals("exists")) {
						exists = ((JTextField)subcompo).getText();
						log("Exists: " + exists);
					}

				}
				
				if (compo.getName() != null && compo.getName().contains("feature")) {
					log("OK Found feature panel");
					Feature feat = new Feature();
					
					Vector<FeatureValue> vals = new Vector<FeatureValue>();
					
					for (int k = 0 ; k < subcomponents.length ; k++) {
						if (subcomponents[k].getClass().equals(JPanel.class)) {
							JPanel subcompo = (JPanel) subcomponents[k];
							Component[] subsubcomponents = subcompo.getComponents();

							for (int l = 0 ; l < subsubcomponents.length ; l++) {
								
								Component subsubcompo = subsubcomponents[l];
								if (subsubcompo.getName() != null && 
										subsubcompo.getName().equals("featLabel")) {
									feat.featlabel = ((JTextField)subsubcompo).getText();
									log("Feature label: " + feat.featlabel);
								}
							}
							
							if (subcompo.getName() != null && 
									subcompo.getName().contains("featvalueprob")) {
								
								FeatureValue value = new StringValue();
								
								for (int m = 0 ; m < subsubcomponents.length ; m++) {
									if (subsubcomponents[m].getClass().equals(JPanel.class)) {
										JPanel subsubcompo = (JPanel) subsubcomponents[m];
										Component[] subsubsubcomponents = subsubcompo.getComponents();

										for (int n = 0 ; n < subsubsubcomponents.length ; n++) {
											
											Component subsubsubcompo = subsubsubcomponents[n];
											if (subsubsubcompo.getName() != null && 
													subsubsubcompo.getName().equals("featvalue")) {
												StringValue stringval = 
													new StringValue(0, getCASTTime(), ((JTextField)subsubsubcompo).getText());
												value = stringval;
												log("Feature value: " +  stringval.val);
											}
											
											if (subsubsubcompo.getName() != null && 
													subsubsubcompo.getName().equals("featprob")) {
												float prob = 
													Float.parseFloat(((JTextField)subsubsubcompo).getText());								
												value.independentProb = prob;									
												DiscreteProbabilityAssignment assignment = 
													new DiscreteProbabilityAssignment();
												assignment.featurepairs = new FeatureValuePair[1];
												assignment.featurepairs[0] = new FeatureValuePair();
												assignment.featurepairs[0].featlabel = feat.featlabel;
												assignment.featurepairs[0].featvalue = value;
												assignment.prob = prob;
												assignments.add(assignment);
												log("Feature prob: " +  prob);
											} 
										}
									}
								}
								
								if (value != null)
								{
									vals.add(value);
								}
							
							}
						}	
						feat.alternativeValues = new FeatureValue[vals.size()];
						int z = 0;
						for (Enumeration<FeatureValue> e = vals.elements() ; e.hasMoreElements(); ){
							feat.alternativeValues[z] = e.nextElement();
							z++;
						}
						log("number of feat values in feature: " + vals.size());
					}
					
					fds.add(feat);
				}
			}
		}

		Proxy newProxy = new Proxy();
		newProxy.entityID = proxyID;
		newProxy.origin = createWorkingMemoryPointer(subarch, "", "");
		newProxy.probExists = Float.parseFloat(exists);
		
		newProxy.features = new Feature[fds.size()];
		int z = 0;
		for (Enumeration<Feature> e = fds.elements(); e.hasMoreElements();) {
			newProxy.features[z] = e.nextElement();
			z++;
		}
		log("number of features in proxy: " + fds.size());
		
		newProxy.distribution = DistributionGeneration.generateProbabilityDistribution(newProxy);
		
		DiscreteProbabilityDistribution distrib = new DiscreteProbabilityDistribution();
		distrib.assignments = new DiscreteProbabilityAssignment[assignments.size()];
		
		for (int i = 0; i < assignments.size(); i++) {
			distrib.assignments[i] = assignments.elementAt(i);
		}
		
		try {
			bm.addToWorkingMemory(newProxy.entityID, newProxy);
			log("new Proxy succesfully added to the binding working memory");
			
			}
			catch (Exception e) {
				e.printStackTrace();
			}

	}


	private void log(String s) {
		System.out.println("[InsertButtonListener]" + s);
	}
}
