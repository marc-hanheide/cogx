package org.cognitivesystems.comsys.general.testers;

import java.util.Properties;
import java.util.Vector;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs;
import org.cognitivesystems.comsys.data.testData.TestData;
import org.cognitivesystems.comsys.data.testData.VisualGroundingTestData;
import org.cognitivesystems.comsys.processing.ActiveIncrCCGParser;
import org.cognitivesystems.comsys.util.XMLTestReader;

import BindingData.BindingProxy;
import BindingData.BindingUnion;
import BindingFeatures.Concept;
import BindingData.FeaturePointer;
import BindingFeatures.SourceID;

import cast.core.CASTException;
import cast.core.data.CASTData;

public class VisualGroundingTester extends AbstractComsysTester {


	/**
	 * @param _id
	 */
	public VisualGroundingTester(String _id) {
		super(_id);
	}

	public class VisualGroundingTest extends AbstractComsysTest  {


		VisualGroundingTestData testData ;

		String reference;
		String referent;

		public VisualGroundingTest(VisualGroundingTestData testData) {
			this.testData = testData;
		}

		protected boolean shouldBeGrounded = true;


		@Override
		public void checkPLF(PackedLFs plf) {

			if (plf.finalized == ActiveIncrCCGParser.FINAL_PARSE) {

				// HENRIK + GJ -- HACK!!! 
				// need to check here whether binding is stable -- if to check for binding ... 
				sleepProcess(8000); 

				testComplete(testUnion()==shouldBeGrounded);

			}
		}

		public boolean testUnion() {

			try {

				CASTData<?>[] sourceIds = getWorkingMemoryEntries("binding.sa", SourceID.class, 0);

				CASTData<?>[] sourceIds2 = getWorkingMemoryEntries("binding.sa", BindingUnion.class, 0);

				log("UNION: length: " + sourceIds2.length);

				BindingProxy proxyDataComsys = new BindingProxy();
				boolean foundComsysProxy = false;

				for (int i = 0 ; i < sourceIds.length ; i++) {
					SourceID sourceId1 = (SourceID)sourceIds[i].getData();	

					if (sourceId1.m_sourceID.equals("comsys.sa")) {
						String ProxyID1 = sourceId1.m_parent.m_immediateProxyID;
						try {
							CASTData<?> proxy1 =  getWorkingMemoryEntry(ProxyID1, "binding.sa"); 
							proxyDataComsys = (BindingProxy)proxy1.getData();
							FeaturePointer[] pointers = proxyDataComsys.m_proxyFeatures;

							for (int p = 0 ; p < pointers.length ; p++) {
								FeaturePointer pointer = pointers[p];
								try {
									CASTData<?> proxyF =  getWorkingMemoryEntry(pointer.m_address, "binding.sa");
									if (pointer.m_type.equals("BindingFeatures::Concept")) {
										Concept c = (Concept)proxyF.getData();
										log("Concept: " + c.m_concept);
										if (reference.contains(c.m_concept)) {
											foundComsysProxy = true;
											log("foundComsysProxy");
										}
									}
								}
								catch (RuntimeException e) { }
							}
						}
						catch (RuntimeException e) { }

					}
				}	

				BindingProxy proxyDataVision = new BindingProxy();
				boolean foundVisionProxy = false;

				for (int i = 0 ; i < sourceIds.length ; i++) {
					SourceID sourceId1 = (SourceID)sourceIds[i].getData();	

					if (sourceId1.m_sourceID.equals("vision.sa")) {
						String ProxyID1 = sourceId1.m_parent.m_immediateProxyID;
						try {
							CASTData<?> proxy1 =  getWorkingMemoryEntry(ProxyID1, "binding.sa"); 
							proxyDataVision = (BindingProxy)proxy1.getData();
							FeaturePointer[] pointers = proxyDataVision.m_proxyFeatures;
							for (int p = 0 ; p < pointers.length ; p++) {
								FeaturePointer pointer = pointers[p];
								log(pointer.m_address);
								try {
									CASTData<?> proxyF =  getWorkingMemoryEntry(pointer.m_address, "binding.sa");
									if (pointer.m_type.equals("BindingFeatures::Concept")) {
										Concept c = (Concept)proxyF.getData();
										log("Concept: " + c.m_concept);
										log("REFERENT: " + referent);
										if (referent.contains(c.m_concept)) {
											foundVisionProxy = true;
											log("foundVisionProxy");
										}
									}
								}
								catch (RuntimeException e) { }
							}
						}
						catch (RuntimeException e) { }

					}
				}


				if (foundComsysProxy & foundVisionProxy) {

					log("unionID-2: " + proxyDataComsys.m_unionID);
					log("unionID-2: " + proxyDataVision.m_unionID);
					return (proxyDataComsys.m_unionID.equals(proxyDataVision.m_unionID)) ;
				}
				// check grounding
				//			return (proxyData1.m_unionID.equals(proxyData2.m_unionID));


			}
			catch (Exception e) {
				e.printStackTrace();
			}
			return false;
		}

		@Override
		protected void startTest() {
			if (testData != null && testData.getString() != "") {
				shouldBeGrounded = testData.shouldBeGrounded();				
				reference = testData.getString();
				referent = testData.getReferent();
				log("REFERENT: " + testData.getReferent());
				initParseTest(testData.getString(), new Vector<String>());
			}
		}


	}


	String VGTestsFile = "";

	Vector<TestData> VGTests;


	@Override
	public void configure(Properties _config) {

		if (_config.containsKey("--testfile")) {
			VGTestsFile = _config.getProperty("--testfile");
			XMLTestReader reader = new XMLTestReader();
			VGTests = reader.readTest(VGTestsFile);	
		}

		try {
			for (int i = 0 ; VGTests != null && i < VGTests.size(); i++) {
				registerTest(""+(i+1), new VisualGroundingTest((VisualGroundingTestData)VGTests.elementAt(i)));
				log("Test parsing"+(i+1)+" successfully registered");
			}
		} catch (CASTException e) {
			e.printStackTrace();
		}

		super.configure(_config);

	}


}