package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.cast.dialogue.DiscourseReferringExpressionGeneration.DiscursiveGenerator;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationRequest;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationResult;
import de.dfki.lt.tr.dialogue.production.ReferringExpressionGenerator;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

public class DiscourseReferringExpressionGeneration
extends AbstractReferringExpressionGenerationComponent<DiscursiveGenerator> {

	private WorkingMemoryAddress lastMentioned = null;

	@Override
	public void onStart() {
		super.onStart();

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				BaseIntention.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

							@Override
							public void execute(WorkingMemoryAddress addr) {
								try {
									BaseIntention bint = getMemoryEntry(addr, BaseIntention.class);

									WorkingMemoryAddress aboutAddr = bint.addressContent.get("about");
									if (aboutAddr != null) {
										setLastMentioned(aboutAddr);
									}

								}
								catch (SubarchitectureComponentException ex) {
									logException(ex);
								}
							}

						});
					}
				});

	}

	@Override
	protected DiscursiveGenerator initGenerator() {
		return new DiscursiveGenerator();
	}

	private void setLastMentioned(WorkingMemoryAddress wma) {
		lastMentioned = wma;
	}

	private WorkingMemoryAddress getLastMentioned() {
		return lastMentioned;
	}


	public class DiscursiveGenerator implements ReferringExpressionGenerator {

		public DiscursiveGenerator() {
		}

		@Override
		public ReferenceGenerationResult generate(ReferenceGenerationRequest request, WorkingMemoryAddress requestAddr) {
			List<String> phrases = new LinkedList<String>();
			if (request.shortNP) {
				phrases.add(getShortNP(request.obj, request.disabledProperties));
			}
			if (request.spatialRelation) {
				phrases.add("on the table");
			}

			return new ReferenceGenerationResult(requestAddr, join(" ", phrases));
		}

		private String getShortNP(WorkingMemoryAddress obj, List<String> disabledProps) {
			List<String> words = new LinkedList<String>();

			WorkingMemoryAddress inFocus = getLastMentioned();

			if (inFocus != null && obj != null && obj.equals(inFocus)) {
				words.add("it");
			}
			else {
				words.add("this");
			}
			return join(" ", words);
		}

	}

	public static String join(String separator, List<String> args) {
		StringBuilder sb = new StringBuilder();
		if (args != null) {
			Iterator<String> iter = args.iterator();
			while (iter.hasNext()) {
				sb.append(iter.next());
				if (iter.hasNext()) {
					sb.append(separator);
				}
			}
		}
		return sb.toString();
	}

}
