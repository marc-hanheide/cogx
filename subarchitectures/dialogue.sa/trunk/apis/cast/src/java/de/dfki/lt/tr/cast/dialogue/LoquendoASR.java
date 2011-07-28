package de.dfki.lt.tr.cast.dialogue;

import cast.AlreadyExistsOnWMException;
import cast.core.CASTData;
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.dialogue.asr.LoquendoClient;
import de.dfki.lt.tr.dialogue.asr.loquendo.result.RecognitionResult;
import de.dfki.lt.tr.dialogue.asr.loquendo.result.Hypothesis;
import de.dfki.lt.tr.dialogue.asr.loquendo.result.NBestList;
import de.dfki.lt.tr.dialogue.asr.loquendo.result.NoRecognitionResult;
import de.dfki.lt.tr.dialogue.asr.loquendo.result.RejectionFlag;
import de.dfki.lt.tr.dialogue.asr.loquendo.time.TimeVal;
import de.dfki.lt.tr.dialogue.slice.asr.InitialNoise;
import de.dfki.lt.tr.dialogue.slice.asr.InitialPhonString;
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import de.dfki.lt.tr.dialogue.slice.asr.Noise;
import de.dfki.lt.tr.dialogue.slice.time.Interval;
import de.dfki.lt.tr.dialogue.slice.time.TimePoint;
import de.dfki.lt.tr.dialogue.util.DialogueException;
import de.dfki.lt.tr.meta.TRResultListener;
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

	public static final String defaultServerName = "LoquendoASRServer";
	public static final String defaultServerEndpoint = "tcp -p 9021";

	private LoquendoClient client = null;

	private String serverName = defaultServerName;
	private String serverEndpoint = defaultServerEndpoint;
	
	private float threshold = 0.2f;

	private final int FRAME_RATE = 10;  // length of a frame in ms
	private final int VOICEDETECT_CORRECTION = 100;  // in ms

	/**
	 * Starts up the component. The engine is configured and started
	 * already in the configure method (which is called before start).
	 *
	 * @see #configure(Map)
	 */

	@Override
	public void start() {
		super.start();
		if (client != null) {
			client.start();
		}
	}


    /**
     * Uses the component configuration arguments to configure the ASR engine,
     * and starts the engine (<tt>run</tt>) once it has been configured.
     */
	@Override
	public void configure (Map<String, String> _config) {
		if (_config.containsKey("--serverName")) {
			serverName = _config.get("--serverName");
		}
		if (_config.containsKey("--threshold")) {
			try {
				threshold = Float.parseFloat(_config.get("--threshold"));
			}
			catch (NumberFormatException ex) {
				getLogger().error("wrong format for threshold", ex);
			}
		}
		if (_config.containsKey("--serverEndpoint")) {
			serverEndpoint = _config.get("--serverEndpoint");
		}
		try {
			client = new LoquendoClient(serverName, serverEndpoint, this.getLogger(".client"));
			client.registerNotification(this);
		}
		catch (Ice.LocalException ex) {
			getLogger().error("failed to connect to the ASR server", ex);
		}
	}

	@Override
	public void executeTask(ProcessingData data)
	throws DialogueException {
		try {
			Iterator<CASTData> dit = data.getData();
			while (dit.hasNext()) {
				CASTData d = dit.next();
				if (d.getData() instanceof Noise) {
					Noise noise = (Noise)d.getData();
					addToWorkingMemory(newDataID(), new InitialNoise(noise));
				}
				if (d.getData() instanceof PhonString) {
					PhonString pstr = (PhonString)d.getData();
					addToWorkingMemory(newDataID(), new InitialPhonString(pstr));
				}
			}
		}
		catch (AlreadyExistsOnWMException ex) {
			getLogger().error("already exists on WM", ex);
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
			getLogger().info("notify: got a RecognitionResult: " + rr.getClass().getCanonicalName());

			if (rr instanceof NoRecognitionResult) {
				getLogger().debug("ignoring a NoRecognitionResult");
			}
			else if (rr instanceof NBestList) {
				PhonString pstr = nBestListToPhonString((NBestList)rr);
				if (pstr != null) {
					
					getLogger().info("Recognised phonological string: " + pstr.wordSequence + " [" + pstr.confidenceValue + "], rejectionAdvice=" + (pstr.maybeOOV ? "true" : "false"));

					String taskID = newTaskID();
					ProcessingData pd = new ProcessingData(newProcessingDataId());
					if (pstr.confidenceValue > threshold) {

						pd.add(new CASTData<PhonString> ("emptyid", pstr));
					}
					else {
						getLogger().debug("phonological string below minimal threshold, will forward a Noise object");
						pd.add(new CASTData<Noise> ("emptyid", nBestListToNoise((NBestList) rr)));
					}
					addProposedTask(taskID, pd);
					String taskGoal = DialogueGoals.ASR_TASK;
					proposeInformationProcessingTask(taskID, taskGoal);
				}
				else {
					getLogger().warn("got a NULL in phonstring extraction");
				}
			}
			else {
				getLogger().warn("don't know how to treat a " + rr.getClass().getCanonicalName());
			}
		}
		else {
			getLogger().error("notify: not a RecognitionResult: " + rr.getClass().getCanonicalName());
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
			if (pstr.wordSequence.equals("no")) {
				pstr.wordSequence = "No";
			}
			pstr.maybeOOV = (nbl.rejectionAdvice == RejectionFlag.RecognitionNomatch);

			TimePoint begin = timeValToTimePoint(nbl.timeAnchor);
			begin.msec -= VOICEDETECT_CORRECTION;
			TimePoint end = timeValToTimePoint(nbl.timeAnchor);

			// compute the duration
			long duration_msec = 0;
			if (hypo.words.length > 0) {
				duration_msec = (hypo.words[hypo.words.length-1].endFrame) * FRAME_RATE;
				getLogger().debug("phonstring duration is " + duration_msec + " ms (" + (nbl.speechEndFrame - nbl.speechStartFrame) * FRAME_RATE + " ms)");
			}

			// extend the temporal interval by the duration
			end.msec += duration_msec;

			pstr.ival = new Interval(begin, end);
		}
 		return pstr;
	}

	public Noise nBestListToNoise(NBestList nbl) {
		TimePoint begin = timeValToTimePoint(nbl.timeAnchor);
		begin.msec -= VOICEDETECT_CORRECTION;
		TimePoint end = timeValToTimePoint(nbl.timeAnchor);

		// compute the duration
		long duration_msec = (nbl.speechEndFrame - nbl.speechStartFrame) * FRAME_RATE;
		getLogger().debug("noise duration is " + duration_msec + " ms");

		// extend the temporal interval by the duration
		end.msec += duration_msec;

		return new Noise(new Interval(begin, end));
	}

	public static TimePoint timeValToTimePoint(TimeVal ta) {
		return new TimePoint(ta.sec * 1000 + ta.usec / 1000);
	}

}
