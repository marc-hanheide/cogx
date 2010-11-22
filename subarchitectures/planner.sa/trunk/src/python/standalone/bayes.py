import sys, itertools

from itertools import chain, product
from collections import defaultdict

import config, pddl, dt_problem
import globals as global_vars

from simplegraph import Graph

log = config.logger("switching")

class tee :
    def __init__(self, _fd1, _fd2) :
        self.fd1 = _fd1
        self.fd2 = _fd2

    def __del__(self) :
        if self.fd1 != sys.stdout and self.fd1 != sys.stderr :
            self.fd1.close()
        if self.fd2 != sys.stdout and self.fd2 != sys.stderr :
            self.fd2.close()

    def write(self, text) :
        self.fd1.write(text)
        self.fd2.write(text)

    def flush(self) :
        self.fd1.flush()
        self.fd2.flush()
        
class BnetInterface(object):
    def __init__(self, nodes, edges, debug=False):
        self.process = None
        self.cmd = global_vars.mapsim_config.switching.bnet_executable
        self.out = None

        self.debug = debug
        self.run()
        self.prepare_graph(nodes, edges)
        self.send_network()

    def prepare_graph(self, nodes, edges):
        self.nodes = list(nodes)
        self.edges = edges
        
        self.nodedict = {}
        self.valdict = {}
        for i, n in enumerate(self.nodes):
            self.nodedict[i] = n
            self.nodedict[n.var] = i
            self.valdict[n.var] = {}
            for j, v in enumerate(n.values):
                self.valdict[n.var][v] = j
                self.valdict[n.var][j] = v

    def set_graph(self, nodes, edges):
        self.prepare_graph(nodes, edges)
        self.send_network()

    def run(self):
        import subprocess, atexit
        log.debug("running bnet solver with '%s'", self.cmd)
        self.process = subprocess.Popen(self.cmd.split(), stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=open("bnet.log", "w"))
        if self.debug:
            self.out = tee(open("bnet_input", mode='w'), self.process.stdin)
        else:
            self.out = self.process.stdin
        atexit.register(lambda: self.kill())
        log.debug("process %d created", self.process.pid)

    def kill(self):
        if not self.process or self.process.returncode is not None:
            return
        log.debug("killing process %d", self.process.pid)
        self.process.terminate()
        self.process = None
        self.out = None
        
    def send_network(self):
        print >> self.out, "structure"
        print >> self.out, len(self.nodes) # node count
        print >> self.out, " ".join(str(len(n.values)) for n in self.nodes) # arities
        print >> self.out, len(self.edges) # edge count
        for from_, to in self.edges:
            print >> self.out, self.nodedict[from_.var], self.nodedict[to.var]

        for i, n in enumerate(self.nodes):
            print >> self.out, "node", i
            print >> self.out, len(n.parents) # number of parents
            print >> self.out, " ".join(str(self.nodedict[p]) for p in n.parents) # parents of this node
            for pvals, vals in n.dist.iteritems():
                pstr = " ".join(str(self.valdict[pvar][pval]) for pvar, pval in zip(n.parents, pvals))
                vstr = " ".join(str(v) for v in vals)
                print >> self.out, pstr, vstr
                
    def set_evidence(self, evidence):
        print >> self.out, "evidence"
        print >> self.out, len(evidence)
        for node, val in evidence.iteritems():
            print >> self.out, self.nodedict[node.var], self.valdict[node.var][val]
        
    def evaluate(self):
        print >> self.out, "evaluate"
        result = {}
        i = 0
        while i < len(self.nodes):
            line = self.process.stdout.readline()
            #log.debug("read: %s", line)
            node = self.nodedict[i]
            rvec = [float(s) for s in line.strip().split()]
            result[node] = rvec
            i += 1
        return result
    

class BayesNode(object):
    def __init__(self, var, values, parents):
        self.var = var
        self.values = values
        self.parents = parents
        self.dist = {}

class BayesianState(object):
    def __init__(self, prob_state, problem, pnodes, domain):
        self.obs_id = 0
        self.domain = domain
        self.pnodes = pnodes
        self.problem = problem
        self.state = prob_state
        self.iface = None
        
        # dt_rules = pddl.translators.Translator.get_annotations(self.domain).get('dt_rules', [])
        # objects = self.domain.constants | prob_state.problem.objects
        # trees = dt_problem.StateTreeNode.create_root(self.state, objects, dt_rules)
        self.nodedict = {}
        self.factdict = defaultdict(set)
        for n in pnodes:
            n.get_nodedicts(self.nodedict, self.factdict)

        self.init_bayes(pnodes)
        self.obs = {}

    def init_bayes(self, pnodes):
        self.values = defaultdict(list)
        deps = Graph()
        dists = defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: 0)))

        def extract_values(node):
            vdist = defaultdict(lambda: 0)
            for val, (p, nodes, facts) in node.children.iteritems():
                vdist[val] = p
                if val != pddl.UNKNOWN and val not in self.values[node.svar]:
                    self.values[node.svar].append(val)
                for n in nodes:
                    deps[node.svar].add(n.svar)
                    dists[node.svar, val][n.svar].update(extract_values(n))
            return vdist

        rootdists = [(n.svar, extract_values(n)) for n in pnodes]
        rdeps = deps.reverse()

        simple_vars = set(svar for svar, _ in rootdists if len(self.values[svar]) < 2 and svar not in deps and svar not in rdeps)
        for svar in simple_vars:
            del self.values[svar]

        rootnode = BayesNode("root", [str(True)], [])
        rootnode.dist[tuple()] = [1.0]
        self.nodes = {"root" : rootnode}
        self.edges = []
        for svar, vals in self.values.iteritems():
            vals.append(pddl.UNKNOWN) #this will also modify self.values
            self.nodes[svar] = BayesNode(svar, vals, rdeps[svar])
        for svar, dep in deps.iteritems():
            for d in dep:
                self.edges.append((self.nodes[svar], self.nodes[d]))

        for svar, dist in rootdists:
            if svar in simple_vars:
                continue
            node = self.nodes[svar]
            vec = [dist[v] for v in self.values[svar]]
            vec[-1] += 1-sum(vec)
            node.parents = ["root"]
            self.edges.append((rootnode, node))
            node.dist[tuple([str(True)])] = vec

        for svar, parents in rdeps.iteritems():
            if not parents:
                continue
            
            parents = list(parents)
            vecs = []
            for p in parents:
                #compute a joint cpt when there are more than one parent.
                pvec = []
                vecs.append(pvec)
                for v1 in self.values[p]:
                    probs = dists[p, v1][svar]
                    vec = [probs[v2] for v2 in self.values[svar]]
                    #vec.append(1-sum(vec))
                    pvec.append((v1, vec))
                #pvec.append((len(values[p]), [0]*len(values[svar])))

            combinations = product(*vecs)
            for c in combinations:
                index = []
                resultvec = None
                for val, vec in c:
                    index.append(val)
                    if not resultvec:
                        resultvec = vec
                    else:
                        resultvec = [x+y for x,y in zip(resultvec, vec)]
                vsum = sum(resultvec)
                if vsum > 1:
                    resultvec = [x/vsum for x in resultvec]
                    vsum = sum(resultvec)
                resultvec[-1] += 1-vsum
                #print svar, index, resultvec, map(str,self.nodes[svar].parents)
                self.nodes[svar].dist[tuple(index)] = resultvec

    # def init_bayes(self, subtrees):
    #     self.values = defaultdict(list)
    #     deps = Graph()
    #     dists = defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: 0)))

    #     def extract_values(tn):
    #         vdist = {}
    #         for val, (subtrees, p, marginal) in tn.iteritems():
    #             vdist[val] = p
    #             if val != pddl.UNKNOWN and val not in self.values[tn.svar]:
    #                 self.values[tn.svar].append(val)
    #             for sub in subtrees:
    #                 deps[tn.svar].add(sub.svar)
    #                 dists[tn.svar, val][sub.svar].update(extract_values(sub))
    #         return vdist

    #     rootdists = [(t.svar, extract_values(t)) for t in subtrees]
    #     rdeps = deps.reverse()

    #     simple_vars = set(svar for svar, _ in rootdists if len(self.values[svar]) < 2 and svar not in deps and svar not in rdeps)
    #     for svar in simple_vars:
    #         del self.values[svar]

    #     self.nodes = {}
    #     self.edges = []
    #     for svar, vals in self.values.iteritems():
    #         vals.append(pddl.UNKNOWN) #this will also modify self.values
    #         self.nodes[svar] = BayesNode(svar, vals, rdeps[svar])
    #     for svar, dep in deps.iteritems():
    #         for d in dep:
    #             self.edges.append((self.nodes[svar], self.nodes[d]))

    #     for svar, dist in rootdists:
    #         if svar in simple_vars:
    #             continue
    #         vec = [dist[v] for v in self.values[svar]]
    #         vec[-1] += 1-sum(vec)
    #         self.nodes[svar].dist[tuple()] = vec

    #     for svar, parents in rdeps.iteritems():
    #         if not parents:
    #             continue
            
    #         parents = list(parents)
    #         vecs = []
    #         for p in parents:
    #             #compute a joint cpt when there are more than one parent.
    #             pvec = []
    #             vecs.append(pvec)
    #             for v1 in self.values[p]:
    #                 probs = dists[p, v1][svar]
    #                 vec = [probs[v2] for v2 in self.values[svar]]
    #                 #vec.append(1-sum(vec))
    #                 pvec.append((v1, vec))
    #             #pvec.append((len(values[p]), [0]*len(values[svar])))

    #         combinations = product(*vecs)
    #         for c in combinations:
    #             index = []
    #             resultvec = None
    #             for val, vec in c:
    #                 index.append(val)
    #                 if not resultvec:
    #                     resultvec = vec
    #                 else:
    #                     resultvec = [x+y for x,y in zip(resultvec, vec)]
    #             vsum = sum(resultvec)
    #             if vsum > 1:
    #                 resultvec = [x/vsum for x in resultvec]
    #                 vsum = sum(resultvec)
    #             resultvec[-1] += 1-vsum
    #             #print svar, index, resultvec, map(str,self.nodes[svar].parents)
    #             self.nodes[svar].dist[tuple(index)] = resultvec
                

    def evaluate(self):
        if not self.iface:
            self.iface = BnetInterface(self.nodes.values(), self.edges, debug=True)
        if self.obs:
            self.iface.set_evidence(self.obs)
        results = self.iface.evaluate()
        fresults = {}
        for n, r in results.iteritems():
            if n.var not in self.nodedict:
                continue
            dists = defaultdict(dict)
            pnode = self.nodedict[n.var]
            values = self.values[n.var]
            for p, val in zip(r, values):
                dists[pnode.svar][val] = p
                if val not in pnode.children:
                    continue
                _, nodes, facts = pnode.children[val]
                for svar, fval in facts.iteritems():
                    dists[svar][fval] = p
            for svar, dist in dists.iteritems():
                vdist = pddl.prob_state.ValueDistribution(dist)
                if svar not in fresults:
                    fresults[svar] = vdist
                else:
                    fresults[svar] += vdist
                #print svar, vdist
            
            #log.debug("%-30s %s", n.var, str(r))

        for svar, dist in sorted(fresults.iteritems(), key=lambda (svar,d): str(svar)):
            log.debug("%-30s %s", svar, str(dist))
        #    print svar, dist
        return fresults

    def new_ptree(self, pnodes, newprobs):
        def replace_tree(node, branch_p):
            if node.svar not in newprobs or branch_p <= 0.00001:
                return node
            vdist = newprobs[node.svar]
            new_children = {}
            for var, (p, nodes, facts) in node.children.iteritems():
                assert var in vdist
                next_branch_p = vdist[var]
                new_nodes = [replace_tree(n, next_branch_p) for n in nodes]
                new_children[var] = (next_branch_p/branch_p, new_nodes, facts)
            #print self.svar, [p for p, _, _ in new_children.itervalues()]
            return pddl.dtpddl.PNode(node.svar, new_children)

        return [replace_tree(n, 1.0) for n in pnodes]
        

    def expected_observations(self, action):
        expected_obs = []
        for o in self.domain.observe:
            det_args = {}
            if o.execution:
                exe = [ex for ex in o.execution if ex.action.name == action.name]
                if not exe:
                    continue
                det_args = dict((oarg, aarg.get_instance()) for oarg, aarg in zip(exe[0].args, action.args ))
            
            def get_objects(arg):
                if arg in det_args:
                    return [det_args[arg]]
                return list(self.problem.get_all_objects(arg.type))

            @pddl.visitors.collect
            def cond_visitor(cond, results):
                if isinstance(cond, pddl.LiteralCondition):
                    return pddl.state.Fact.from_literal(cond)

            @pddl.visitors.collect
            def obs_visitor(eff, results):
                if isinstance(eff, pddl.SimpleEffect):
                    return ([], 1.0, pddl.state.Fact.from_literal(eff))
                if isinstance(eff, pddl.effects.ProbabilisticEffect):
                    res = []
                    for p, sub in results:
                        p = p.object.value
                        for cond, p_old, fact in sub:
                            res.append((cond, p*p_old, fact))
                    return res
                if isinstance(eff, pddl.ConditionalEffect):
                    cnew = eff.condition.visit(cond_visitor)
                    res = []
                    #print results
                    for cond, p, fact in results:
                        res.append((cond+cnew, p, fact))
                    return res
                    
            
            for mapping in o.smart_instantiate(o.get_inst_func(self.state), o.args, [get_objects(a) for a in o.args], self.problem):
                prec = pddl.visitors.visit(o.precondition, cond_visitor, [])
                for conds, p, fact in pddl.visitors.visit(o.effect, obs_visitor, []):
                    log.debug("expected: %s (%.2f) (%s)", str(fact.svar), p, " ".join(map(str, conds)))
                    expected_obs.append((fact, p, conds+prec))
        return expected_obs

    def handle_obs(self, action, obs):
        expected = self.expected_observations(action)
        if not expected:
            return False
        
        new_nodes = []
        for fact, p, cond in expected:
            parents = []
            valid_values = []
            for f in cond:
                nodes = list(n for n in self.factdict[f.svar] if n.get_branches_for_fact(f))
                parents.append(nodes)

                valueset = set()
                for n in nodes:
                    for val in n.get_branches_for_fact(f):
                        valueset.add((n,val))
                
                valid_values.append(valueset)

            if not parents:
                continue

            obs_id = "%s-%d" % (str(fact.svar), self.obs_id)
            # use str(True) here because we can't distinguish bool and int in a dict later
            bnode = BayesNode(obs_id, [str(True), str(False)], [n.svar for n in chain(*parents)])
            self.nodes[obs_id] = bnode 
            self.obs_id += 1
            for pnode in chain(*parents):
                self.edges.append((self.nodes[pnode.svar], bnode))

            new_nodes.append((bnode, fact.svar, p, parents))
        
        for bnode, svar, p, parents in new_nodes:
            combinations = [self.values[n.svar] for n in chain(*parents)]
            # print "valid groups:"
            # for val in valid_values:
            #     print ["%s=%s" % (str(n.svar), str(v)) for n,v in val]
            for c in product(*combinations):
                cset = set(itertools.izip(chain(*parents), c))
                if all(valid & cset for valid in valid_values):
                    res = [p, 1-p]
                else:
                    res = [0.0, 1.0]
                bnode.dist[c] = res
            
            self.obs[bnode] = str(svar in obs)
            if svar in obs:
                log.debug("observation node %s (%s) was observed.", str(bnode.var), str(svar)),
            else:
                log.debug("observation node %s (%s) was not observed.", str(bnode.var), str(svar)),

        if self.iface:
            self.iface.set_graph(self.nodes.values(), self.edges)

        return True
                                     
