package de.dfki.lt.tr.cast.dialogue;

import cast.AlreadyExistsOnWMException;
import cast.core.CASTData;
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.dialogue.asr.LoquendoClient;
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import de.dfki.lt.tr.dialogue.slice.asr.RecognitionResult;
import de.dfki.lt.tr.dialogue.slice.asr.loquendo.Hypothesis;
import de.dfki.lt.tr.dialogue.slice.asr.loquendo.NBestList;
import de.dfki.lt.tr.meta.TRResultListener;
import de.dfki.lt.tr.dialogue.util.DialogueException;
import java.util.Iterator;
import java.util.Map;

/**
 * A CAST wrapper for the Loquendo ASR client.
 *
 * @author Miroslav Janicek
 */
public class LoquendoASR
extends AbstractDialogueComponent
implements TRResultListener {

	public static final String defaultServerName = "LoquendoRecogniserServer";
	public static final String defaultServerEndpoint = "tcp -p 9021";

	private LoquendoClient client = null;

	private String serverName = defaultServerName;
	private String serverEndpoint = defaultServerEndpoint;

	/**
	 * Starts up the component. The engine is configured and started
	 * already in the configure method (which is called before start).
	 *
	 * @see #configure(Map)
	 */

	@Override
	public void start() {
		super.start();
	}


    /**
     * Uses the component configuration arguments to configure the ASR engine,
     * and starts the engine (<tt>run</tt>) once it has been configured.
     */
	@Override
	public void configure (Map<String, String> _config) {
		client = new LoquendoClient(serverName, serverEndpoint);
		client.registerNotification(this);
//		client.configure(_config);
		client.start();
	}

	@Override
	public void executeTask(ProcessingData data)
	throws DialogueException {
		try {
			Iterator<CASTData> dit = data.getData();
			while (dit.hasNext()) {
				CASTData d = dit.next();
				if (d.getData() instanceof RecognitionResult) {
					RecognitionResult rr = (RecognitionResult)d.getData();
					addToWorkingMemory(newDataID(), rr);
				}
				if (d.getData() instanceof PhonString) {
					PhonString pstr = (PhonString)d.getData();
					addToWorkingMemory(pstr.id, pstr);
				}
			}
		}
		catch (AlreadyExistsOnWMException ex) {
			ex.printStackTrace();
		}
	}

	/**
	 * Called whenever the ASR engine produces a RecognitionResult.
	 *
	 * @param object the recognition result
	 */
	@Override
	public void notify(Object rr) {
		if (rr instanceof RecognitionResult) {
			log("notify: got a RecognitionResult");

			if (rr instanceof NBestList) {
				PhonString pstr = nBestListToPhonString((NBestList)rr);
				if (pstr != null) {
					log("recognised phonstring: \"" + pstr.wordSequence + "\"");
					CASTData data = new CASTData ("emptyid", nBestListToPhonString((NBestList)rr));
					String taskID = newTaskID();

					ProcessingData pd = new ProcessingData(newProcessingDataId());
					pd.add(data);
					m_proposedProcessing.put(taskID, pd);

					String taskGoal = DialogueGoals.ASR_TASK;
					proposeInformationProcessingTask(taskID, taskGoal);
				}
				else {
					log("got a NULL in phonstring extraction");
				}
			}
			else {
				log("don't know how to treat a " + rr.getClass().getCanonicalName());
			}
		}
		else {
			log("notify: not a RecognitionResult: " + rr.getClass().getCanonicalName());
		}
	}

	public PhonString nBestListToPhonString(NBestList nbl) {
		PhonString pstr = null;
		if (nbl.hypos.length > 0) {
			pstr = new PhonString();
			pstr.id = newDataID();

			Hypothesis hypo = nbl.hypos[0];
			pstr.rank = 1;
			pstr.confidenceValue = hypo.confidence;
			pstr.NLconfidenceValue = hypo.confidence;
			pstr.length = hypo.words.length;
			pstr.wordSequence = hypo.str;
		}
 		return pstr;
	}

}
