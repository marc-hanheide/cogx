package de.dfki.lt.tr.cast.dialogue;

import java.awt.BorderLayout;
import java.awt.Font;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.SwingUtilities;

import cast.UnknownSubarchitectureException;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.cast.dialogue.VisualSalienceReferenceResolution.Resolver;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.ref.Constraint;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolver;
import de.dfki.lt.tr.dialogue.ref.util.ReferenceUtils;
import de.dfki.lt.tr.dialogue.slice.time.Interval;
import de.dfki.lt.tr.dialogue.slice.time.TimePoint;
import de.dfki.lt.tr.dialogue.util.EpistemicStatusFactory;
import eu.cogx.beliefs.slice.MergedBelief;


public class VisualSalienceReferenceResolution
        extends AbstractReferenceResolutionComponent<Resolver> {

    private boolean visualMode = false;
    private WMView<MergedBelief> view = null;
    private DebugWindow win = null;
    
    public VisualSalienceReferenceResolution() {
        super();
    }
    
    @Override
    public void onConfigure(Map<String, String> props) {
        
        if (props.containsKey("--visual")) {
            visualMode = true;
        }
        
    }
    
    @Override
    public void onStart() {
        super.onStart();
        
        if (visualMode) {
            win = new DebugWindow();
            win.pack();
            win.setVisible(true);
            
            Thread th = new Thread() {
                @Override
                public void run() {
                    log("starting the monitor thread");
                    try {
                        while (isRunning()) {
                            StringBuilder sb = new StringBuilder();

                            CASTTime t = getCASTTime();

                            long now = System.currentTimeMillis();
                            
                            String tstr = String.format("%d.%03d", t.s, t.us / 1000);
                            sb.append("Visual salience at t = ").append(tstr).append("\n");
                            sb.append("\n");

                            if (view != null) {
                                Iterator<Map.Entry<WorkingMemoryAddress, MergedBelief>> it = view.entrySet().iterator();
                                
                                while (it.hasNext()) {
                                    Map.Entry<WorkingMemoryAddress, MergedBelief> entry = it.next();

                                    double sal = getSalienceForBelief(entry.getValue());
                                    if (!Double.isNaN(sal)) {
                                        sb.append(wmaToString(entry.getKey())).append(" .. ");
                                        sb.append(sal);
                                        sb.append("\n");
                                    }
                                }
                                
                            }

                            // the request
                            final WorkingMemoryAddress addr = new WorkingMemoryAddress("DUMMY", "dialogue");
                            final ReferenceResolutionRequest dummyThis = new ReferenceResolutionRequest();
                            dummyThis.nom = "context_0";
                            dummyThis.sort = "context";
                            dummyThis.ival = new Interval(new TimePoint(now - 100), new TimePoint(now));
                            dummyThis.constraints = new LinkedList<Constraint>();
                            dummyThis.constraints.add(new Constraint("salience", "high"));
                            
                            sb.append("\n");
                            sb.append("Request:").append("\n");
                            sb.append(ReferenceUtils.resolutionRequestToString(dummyThis));

                            sb.append("\n");
                            sb.append("Result:").append("\n");

                            // reply
                            Resolver rslvr = getResolver();
                            if (rslvr != null) {
                                ReferenceResolutionResult result = rslvr.resolve(dummyThis, addr);
                                sb.append(ReferenceUtils.resolutionResultToString(result));
                            }
                            else {
                                sb.append("RESOLVER is NULL!");
                            }
                            
                            final String s = sb.toString();
                            SwingUtilities.invokeLater(new Runnable() {
                                @Override
                                public void run() {
                                    win.entries.setText(s);
                                }
                            });
                            Thread.sleep(250);
                        }
                    }
                    catch (InterruptedException ex) {
                        log("was interrupted, exiting the monitor thread");
                    }
                    log("monitor thread done");
                }
            };
            
            th.start();
        }
        
        view = WMView.<MergedBelief>create(this, MergedBelief.class);
        try {
            view.start();
        }
        catch (UnknownSubarchitectureException ex) {
            logException(ex);
        }
        
    }

    @Override
    protected Resolver initResolver() {
        return new Resolver();
    }

    public static boolean isVisible(dBelief bel) {
        if ("visualobject".equals(bel.type)) {
            if (bel.content instanceof CondIndependentDistribs) {
                CondIndependentDistribs d = (CondIndependentDistribs) bel.content;
                ProbDistribution pd = d.distribs.get("presence");
                if (pd != null && pd instanceof BasicProbDistribution) {
                    BasicProbDistribution presd = (BasicProbDistribution) pd;
                    if (presd.values instanceof FormulaValues) {
                        FormulaValues fvs = (FormulaValues) presd.values;
                        if (fvs.values.size() == 1) {
                            FormulaProbPair fpp = fvs.values.get(0);
                            if (fpp.val instanceof ElementaryFormula) {
                                ElementaryFormula formula = (ElementaryFormula) fpp.val;
                                return "visible".equals(formula.prop) || "was-visible".equals(formula.prop);
                            }
                        }
                    }
                }
            }
        }
        return false;
    }
    
    public static double getSalienceForBelief(dBelief bel) {
        if ("visualobject".equals(bel.type)) {
            if (bel.content instanceof CondIndependentDistribs) {
                CondIndependentDistribs d = (CondIndependentDistribs) bel.content;
                ProbDistribution pd = d.distribs.get("salience");
                if (pd != null && pd instanceof BasicProbDistribution) {
                    BasicProbDistribution sald = (BasicProbDistribution) pd;
                    if (sald.values instanceof FormulaValues) {
                        FormulaValues fvs = (FormulaValues) sald.values;
                        if (fvs.values.size() == 1) {
                            FormulaProbPair fpp = fvs.values.get(0);
                            if (fpp.val instanceof FloatFormula) {
                                FloatFormula formula = (FloatFormula) fpp.val;
                                return (double) formula.val;
                            }
                        }
                    }
                }
            }
        }        
        return Double.NaN;
    }

    public static boolean isVisualSalienceRequest(ReferenceResolutionRequest rr) {
        boolean result = false;
        if (ReferenceResolver.SORT_DISCOURSE.equals(rr.sort)) {
            return false;
        }
        for (Constraint c : rr.constraints) {
            if ("salience".equals(c.feature)) {
                result = true;
            }
        }
        return result;
    }

    public class Resolver implements ReferenceResolver {

        private final EpistemicStatus epst = EpistemicStatusFactory.newSharedEpistemicStatus(IntentionManagementConstants.thisAgent, IntentionManagementConstants.humanAgent);
        
        protected List<EpistemicReferenceHypothesis> getHypotheses() {
            List<EpistemicReferenceHypothesis> result = new LinkedList<EpistemicReferenceHypothesis>();

            // TODO: verify that the iterator is thread-safe
            Iterator<Map.Entry<WorkingMemoryAddress, MergedBelief>> it = view.entrySet().iterator();

            double latest = -1.0;
            WorkingMemoryAddress latestAddr = null;
            
            while (it.hasNext()) {
                Map.Entry<WorkingMemoryAddress, MergedBelief> entry = it.next();

                dBelief bel = entry.getValue();
                if (isVisible(bel)) {
                    double sal = getSalienceForBelief(bel);
                    if (!Double.isNaN(sal)) {
                        if (sal > latest) {
                            latest = sal;
                            latestAddr = entry.getKey();
                        }
                    }
                }
            }

            if (latestAddr != null) {
                dFormula referent = new PointerFormula(0, latestAddr, "");
                double score = 10.0;
                result.add(new EpistemicReferenceHypothesis(epst, referent, score));
            }
                
            return result;
        }
        
        public ReferenceResolutionResult resolve(ReferenceResolutionRequest rr, WorkingMemoryAddress origin) {
            ReferenceResolutionResult result = ReferenceUtils.newEmptyResolutionResult(rr, origin, "visualsalience");
            if (isVisualSalienceRequest(rr)) {
                log("this is a request about visual salience");
                result.hypos.addAll(getHypotheses());
            }
            else {
                log("request doesn't seem to be about visual salience");
            }
            return result;
        }
        
    }

    public class DebugWindow extends JFrame {

	final JTextArea entries;

        public DebugWindow() {
            super("Visual salience");
            
            entries = new JTextArea (25, 80);
            entries.setEditable(false);
            entries.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));

            getContentPane().add(new JScrollPane(entries), BorderLayout.CENTER);
        }
        
    }
    
}
