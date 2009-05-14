#! /usr/bin/env python

from parser import *
from string import Template

target_sing_tmpl = Template("""
(exists (?o0 - $type1 ?p0 ?ip0 - waypoint)
  (and $constraints1
$pos_template))
""")

target_plur_tmpl = Template("""
(forall (?o0 - $type1) (imply 
  $constraints1
  (exists (?p0 ?ip0 - waypoint) 
$pos_template)))
""")

## target_sing_tmpl = Template("""
## (exists (?o0 - $type1 ?p0 - waypoint)
##   (and $constraints1
## $pos_template))
## """)

## target_plur_tmpl = Template("""
## (forall (?o0 - $type1) (imply 
##   (and $constraints1)
##   (exists (?p0 - waypoint) 
## $pos_template)))
## """)

SINGULAR = False
PLURAL = True

target_template = {SINGULAR:target_sing_tmpl, PLURAL:target_plur_tmpl}

pos_single_obj_tmpl_str = """(exists (?o1 - $type2 ?p1 ?ip1 - waypoint)
  (and $constraints2 (initially (pos ?o0 ?ip0)) (pos ?o0 ?p0) (initially (pos ?o1 ?ip1)) (pos ?o1 ?p1)  
($relation ?p0 ?p1)
))"""

pos_several_obj_tmpl_str = """(forall (?o1 - $type2) 
  (imply $constraints2
         (exists (?p1 ?ip1 - waypoint) (and
($relation ?p0 ?p1) (initially (pos ?o0 ?ip0)) (pos ?o0 ?p0) (initially (pos ?o1 ?ip1)) (pos ?o1 ?p1))
)))"""

pos_template = {SINGULAR:pos_single_obj_tmpl_str, PLURAL:pos_several_obj_tmpl_str}




def plural2singular(plu_str):
    """look up singular for a given plural. default case: just remove -s"""
    specials = dict(boxes="box")
    if plu_str in specials:
        return specials(plu_str)
    return plu_str[:-1]
    
def find_symbol(pts, symbols, l=None,verbose=False):
    """return a list of subtrees that contain a symbol from symbols"""
    if not isinstance(symbols, (list,tuple)):
        symbols = [symbols]
    if l is None:
        l = []
        if verbose: print "symbols", symbols
    if not isinstance(pts, (list,tuple)):
        pts = [pts]
    for pt in pts:
        try:
            name = pt.name
        except:
            if verbose: print "name:", pt
            name = pt[0][0]
        if isinstance(name,tuple):
            if verbose: print "name:", name
            if name[0] in symbols:
                l.append(name)
        if verbose:
            print "checking", name
            #print "ot", pt.pretty_print()
        if name in symbols:
            l.append(pt)
        elif isinstance(pt, ParseTreeNode):
            find_symbol(pt.children, symbols, l, verbose=verbose)
    if verbose:
        print "return on this level", l
    return l

def find_preposition_pred(pt):
    NEAR, RIGHT_OF, LEFT_OF = "near", "right_of", "left_of"
    FRONT_OF, BACK_OF = "front_of", "back_of" 
    relations = dict(next=NEAR, near=NEAR,
                     right=RIGHT_OF, left=LEFT_OF,
                     front=FRONT_OF, back=BACK_OF)
    sym = find_symbol(pt, ("Preposition","SpatialRel"))
    if sym:
        rel = sym[0][1]
    return relations[rel]
        
def split_on_conjunction(pt):
    if pt.matches("NP Conjunction NP"):
        conj = pt.children[1].name[1]
        return (conj, (pt.children[0], pt.children[2]))
    return ("and", [pt])

def find_type_and_numerus(pt):
    TYPE_MAP = {"object":"movable", "thing":"movable"}
    numerus = dict(NounSingular=SINGULAR, NounPlural=PLURAL)
    goal_objects = find_symbol(pt, "Noun")
    #assert len(goal_objects) == 1, "Several nouns (eg. relative clause or conjunction) not supported yet."
    num_str, obj_type = goal_objects[0].children[0].name
    is_plural = (numerus[num_str] == PLURAL)
    if is_plural:
        obj_type = plural2singular(obj_type)
    if obj_type in TYPE_MAP:
        obj_type = TYPE_MAP[obj_type]
    # workaround for Stockholm demo
    #obj_type = "movable"
    return obj_type, is_plural

def find_pp_refs(pt):
    pp = find_symbol(pt, "PP")#, verbose=True)
    if not pp:
        return []
    pp = pp[0]
    prep = find_preposition_pred(pp)
    np = find_symbol(pp, "NP")[0]
    return [(prep, np)]

g_IDs = 10
def newID():
    global g_IDs
    g_IDs += 1
    return g_IDs

ref_single_obj_tmpl_str = """(exists (?o$num - $type ?ip$num - waypoint)
  (and $constraints (initially (pos ?o$num ?ip$num))  
($relation ?ip$ref_id ?ip$num)
))"""

ref_several_obj_tmpl_str = """(forall (?o$num - $type)
  (imply $constraints
         (exists (?ip$num - waypoint) (and
(initially (pos ?o$ref_id ?ip$num)) ($relation ?ip$ref_od ?ip$num))
)))"""

ref_template = {SINGULAR:ref_single_obj_tmpl_str, PLURAL:ref_several_obj_tmpl_str}

def make_referential_constraint(prep, np, referring_id):
    id = newID()
    refobj_name = "?ro%d" % id
    props = find_object_property(np)
    ref_tmpl = ref_template[props.numerus]
    subs = dict(type=props.type,
                constraints=make_constraint_str(props.constraints, id, True),
                relation=prep, num=id, ref_id=referring_id)
    constraint = Template(ref_tmpl).safe_substitute(subs)    
    return constraint
    
def find_constraints(pt):
    pp_refs = find_pp_refs(pt)
    if pp_refs:
        nps = find_symbol(pt.children, "NP")
        if len(nps) != 2:
            print "Oops. prepositional phrases should contain 2 NPs, not %d" % len(nps)
            pt.pretty_print()
        pt = nps[0]
    # currently: either an adjective (color, etc.) corresponding to a predicate or nothing!
    adjectives = find_symbol(pt, "Adjective")
    adjs = [adj[1] for adj in adjectives]
    return adjs + pp_refs

def make_constraint_str(constraints, id, refers_to_init_state=False):
    prefix = ""
    if refers_to_init_state:
        prefix = "initially ("
    cstrings = []
    for c in constraints:
        if isinstance(c, basestring):
            cstring = simple_constraint_str(c, id)
            cstrings.append("(%s%s))" % (prefix, cstring))
        else:
            prep, np = c
            cstring = make_referential_constraint(prep, np, id)
            cstrings.append(cstring)
    if len(cstrings) == 1:
        return cstrings[0]
    return "(and " + " ".join(cstrings) + ")"

def simple_constraint_str(constraint, id):
    colours = "red blue green black white".split()
    sizes = "big small".split()
    feature_map = {}
    for c in colours:
        feature_map[c] = "colour"
    for s in sizes:
        feature_map[s] = "size"
    obj_name = "?o%d" % id
    try:
        feature = feature_map[constraint]
        cstring = "%s %s %s" % (feature, obj_name, constraint)
    except KeyError:
        cstring = "%s %s" % (constraint, obj_name)
    return cstring

dnum = {SINGULAR:"exists", PLURAL:"all"}

def simple_goal_description(relation, objs):
    def description(o):
        # attention: simple goal descriptions currently to do not contain
        # referential expresions like "the cube near the red box"
        props = ["(prop %s)" % c for c in o.constraints if isinstance(c,basestring)]
        return "(quant %s) (type %s) %s" % (dnum[o.numerus], o.type, " ".join(props))
    ds = [description(o) for o in objs]
    ds.insert(1, relation)
    simple_goal = " - ".join(ds)
    # The following print statement was used for the IJCAI sample implementation
    # We have to do something more principled for CAAT
    # print simple_goal
    return simple_goal
    
def find_object_property(np):
    type, numerus = find_type_and_numerus(np)
    constraints = find_constraints(np)
    return Struct(type=type, numerus=numerus, constraints=constraints)

def find_object_properties(nps):
    for np in nps:
        yield find_object_property(np)

def make_goal(np1, pp, np2):
    preposition_pred = find_preposition_pred(pp)
    o0, o1 = list(find_object_properties((np1, np2)))
    goal_tmpl1 = target_template[o0.numerus]
    pos_tmpl = pos_template[o1.numerus]
    subs = dict(pos_template=pos_tmpl, type1=o0.type, constraints1=make_constraint_str(o0.constraints, 0, True))
    goal_tmpl1 = goal_tmpl1.safe_substitute(subs)
    subs = dict(type2=o1.type, constraints2=make_constraint_str(o1.constraints, 1, True),
                relation=preposition_pred)
    goal = Template(goal_tmpl1).safe_substitute(subs)
    simple_goal = simple_goal_description(preposition_pred, (o0,o1))
    return goal, simple_goal

def make_goals(pt):
    return [make_goal_formula(child) for child in pt.children]

def make_goal_formula(pt):
    assert pt.children[0].children[0].name[1] == 'put'
    pt = pt.children[0].children[1]
    conjuncts = find_symbol(pt, "VP1end")
    goals = []
    simple_goals = []
    for gt in conjuncts:
        np1, pp = gt.children
        np2 = find_symbol(pp, "NP")[0] 
        pp = find_symbol(pp, "PPstart")[0] 
        conjunction, targets = split_on_conjunction(np1)
        igoals = []
        for t in targets:
            g, sg = make_goal(t, pp, np2)
            igoals.append(g)
            simple_goals.append(sg)
        if len(targets) > 1:
            if conjunction == "or":
                goal = "\n".join(igoals)
                igoals = ["\n".join(("(or", goal, ")"))]            
        goals.extend(igoals)
    goalstr = "\n".join(goals)
    if len(goals) > 1:
        goalstr = "\n".join(("(and", goalstr, ")"))
    goalstr = pretty_indent(goalstr)
    return goalstr, "\n".join(simple_goals)

def clean(orig):
    base = orig.replace("\n", " ")
    while True:
        t = base
        base = t.replace("  ", " ")
        if base == t: break
    return str(base).strip()

def count_parens(s):
    return s.count("(") - s.count(")")


def pretty_indent(orig, max=50, factor=1):
    clean_str = clean(orig)
    s = str(clean_str)
    lines = []
    start = 0
    breakpoint = 0
    parens = 0
    indent = 0
    for i, c in enumerate(s):
        linebreak = False
        if c == '(':
            parens += 1
            breakpoint = i
            if parens < indent:
                linebreak = True
        if c == ')':
            parens -= 1
        if i - start > max:
            linebreak = True
        if i+1 == len(s):
            breakpoint = i+1
            linebreak = True
        if linebreak:
            line = s[start:breakpoint]
            #lines.append(line)
            lines.append("%s%s" % (indent*' ', line))
            indent = count_parens(s[:breakpoint]) 
            start = breakpoint
    s = "\n".join(lines)
    test = clean(s)
    if test != clean_str:
        print "something happened"
        print "clean orig:"
        print clean_str
        print "new"
        print test
        import sys
    return s
    
def eng2goal(phrase, verbose=False):
    chart = Chart(E0)
    parsetree = ParseTreeNode.from_chart(chart.parses(phrase))
    if parsetree.is_ambiguous():
        if verbose:
            print "Found %d parses for phrase '%s'" % (len(parsetree.children), phrase)
    parses = make_goals(parsetree)
    for (gd, simple_gd) in parses:
        if verbose:
            print "Possible goal formula for '%s':\n%s\n" % (phrase, gd)
    return parses

test_phrases = [
    "Put the red things near the green thing to the right of the blue pyramid.",
##     "Put the red block near the blue block.",
##     "Put the red block to the right of the blue block.",
##     "Put the red blocks to the right of the blue block.",
##     "Put a block to the right of the blue block.",
##     "Put the red block to the right of the green block and the green block to the left of the blue block.",
##     "Put the red block and the blue block to the right of the green block.",
##     "Put the red block or the blue block to the right of the green block.",
    ] 

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        phrases = test_phrases
    else:
        phrases = sys.argv[1:]
    parses = [eng2goal(phrase, verbose=True) for phrase in phrases]
    if False in parses:
        sys.exit()
