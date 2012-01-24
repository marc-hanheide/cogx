package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
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
import de.dfki.lt.tr.meta.TRResultListener;
import java.util.Map;

/**
 * A CAST wrapper for the Loquendo ASR client.
 *
 * @author Miroslav Janicek
 */
public class LoquendoASR
extends AbstractDialogueComponent
implements TRResultListener<RecognitionResult> {

	public static final String DEFAULT_LOQSERVER_NAME = "LoquendoASRServer";
	public static final String DEFAULT_LOQSERVER_ENDPOINT = "tcp -p 9021";

	private LoquendoClient client = null;

	private String serverName = DEFAULT_LOQSERVER_NAME;
	private String serverEndpoint = DEFAULT_LOQSERVER_ENDPOINT;
	
	private float threshold = 0.2f;

	private final int FRAME_RATE = 10;  // length of a frame in ms
	private final int VOICEDETECT_CORRECTION = 100;  // in ms

	public LoquendoASR(String serverName, String serverEndpoint) {
		this.serverName = serverName;
		this.serverEndpoint = serverEndpoint;
		client = null;
	}

	public LoquendoASR() {
		this(DEFAULT_LOQSERVER_NAME, DEFAULT_LOQSERVER_ENDPOINT);
	}

	@Override
	public void onConfigure(Map<String, String> _config) {
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
	public void onStart() {
		super.onStart();
		if (client != null) {
			client.start();
		}
		else {
			getLogger().info("no client found, going to die");
			scheduleOwnDeath();
		}
	}

	@Override
	public void notify(RecognitionResult rr) {
		addTask(new ProcessingTaskWithData<RecognitionResult>(rr) {

			@Override
			public void execute(RecognitionResult rr) {
				getLogger().info("notify: got a RecognitionResult: " + rr.getClass().getCanonicalName());

				if (rr instanceof NoRecognitionResult) {
					getLogger().debug("ignoring a NoRecognitionResult");
				}
				else if (rr instanceof NBestList) {
					NBestList nbestlist = (NBestList) rr;
		
					PhonString pstr = nBestListToPhonString(nbestlist);
					if (pstr != null) {

						try {
							getLogger().info("recognised phonological string: " + pstr.wordSequence + " [" + pstr.confidenceValue + "], rejectionAdvice=" + (pstr.maybeOOV ? "true" : "false"));

							String taskID = newTaskID();
							if (pstr.confidenceValue > threshold) {
								addToWorkingMemory(newDataID(), new InitialPhonString(pstr));
							}
							else {
								getLogger().debug("phonological string below minimal threshold, will forward a Noise object");
								Noise noise = nBestListToNoise(nbestlist);
								addToWorkingMemory(newDataID(), new InitialNoise(noise));
							}
						}
						catch (SubarchitectureComponentException ex) {
							getLogger().error("subarchitecture component exception", ex);
						}
					}
					else {
						getLogger().warn("got a NULL in phonstring extraction, ignoring");
					}
				}
				else {
					getLogger().warn("don't know how to treat a " + rr.getClass().getCanonicalName());
				}
			}
		});
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
