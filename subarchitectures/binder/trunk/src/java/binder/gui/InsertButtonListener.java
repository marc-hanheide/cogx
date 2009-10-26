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

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JTextField;

import org.apache.log4j.Logger;

import cast.cdl.WorkingMemoryPointer;
import cast.core.logging.ComponentLogger;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.Proxy;
import binder.autogen.featvalues.StringValue;
import binder.components.Binder;
import binder.components.BinderMonitor;
import binder.constructors.ProxyConstructor;

public class InsertButtonListener implements ActionListener {
	
	InsertProxyWindow frame;
	BinderMonitor bm;
	
	public boolean LOGGING = false;
	
	private static Logger logger = ComponentLogger.getLogger(InsertButtonListener.class);
	
	
	public InsertButtonListener(InsertProxyWindow frame, BinderMonitor bm) {
		this.frame = frame;
		this.bm = bm;
	}

	public void actionPerformed(ActionEvent event) {
		String proxyID = "";
		String subarch = "";
		String exists = "";
		Vector<Feature> fds = new Vector<Feature>();

		Component[] components = frame.proxyPanel.getComponents();

		
		for (int i = 0 ; i < components.length ; i++) {
			if (components[i].getClass().equals(JPanel.class)) {
				JPanel compo = (JPanel) components[i];
				Component[] subcomponents = compo.getComponents();
				for (int j = 0 ; j < subcomponents.length ; j++) {
				
					Component subcompo = subcomponents[j];
					if (subcompo.getName() != null && subcompo.getName().equals("proxyID")) {
						proxyID = ((JTextField)subcompo).getText();
						log("Proxy ID: " + proxyID);
						((JTextField)subcompo).setText("proxyID" + InsertProxyWindow.proxyCount);
						InsertProxyWindow.proxyCount++;
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
													ProxyConstructor.createStringValue(((JTextField)subsubsubcompo).getText(), 0);
												ProxyConstructor.setTimeStamp(stringval, bm.getCASTTimeInMonitor());
												value = stringval;
												log("Feature value: " +  stringval.val);
											}
											
											if (subsubsubcompo.getName() != null && 
													subsubsubcompo.getName().equals("featprob")) {
												float prob = 
													Float.parseFloat(((JTextField)subsubsubcompo).getText());								
												value.independentProb = prob;									
												log("Feature prob: " +  prob);
											} 
										}
									}
								}
								
								if (value != null)
								{
									ProxyConstructor.addFeatureValueToFeature(feat, value);
								}
							
							}
						}	
					}
					
					fds.add(feat);
				}
			}
		}
		
		
		WorkingMemoryPointer origin = ProxyConstructor.createWorkingMemoryPointer(subarch, "", "");
		Proxy newProxy = ProxyConstructor.createNewProxy(origin, proxyID, Float.parseFloat(exists));
	
		int z = 0;
		for (Enumeration<Feature> e = fds.elements(); e.hasMoreElements();) {
			Feature feat = e.nextElement();
			if (feat.alternativeValues != null && feat.alternativeValues.length > 0) {
				ProxyConstructor.addFeatureToProxy(newProxy, feat);
				z++;
			}
		}
		log("number of features in proxy: " + fds.size());
		
       	frame.dispose();

		
		try {
			bm.addToWorkingMemory(newProxy.entityID, Binder.BINDER_SA, newProxy);
			log("new Proxy succesfully added to the binding working memory");
			
			}
			catch (Exception e) {
				e.printStackTrace();
			}

	}


	private void log(String s) {
		if (LOGGING)
			logger.debug("[InsertButtonListener]" + s);
	}
}
