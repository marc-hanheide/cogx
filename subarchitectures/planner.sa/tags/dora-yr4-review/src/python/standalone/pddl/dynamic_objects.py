import predicates, conditions, effects, actions, domain, problem, scope, visitors, translators
from scope import Scope
from builder import Builder
import mapltypes as types
import builtin

pddl_module = True

created = predicates.Predicate("created", [types.Parameter("?o", builtin.t_object)], builtin=True)
unused = predicates.Predicate("unused", [types.Parameter("?o", builtin.t_object)], builtin=True)
destroyed = predicates.Predicate("not-instantiated", [types.Parameter("?o", builtin.t_object)], builtin=True)

def effect_handler(it, scope):
    first = it.get(None, "effect specification")
    if first.token.string == "create":
        return CreateEffect.parse(it.reset(), scope)
    elif first.token.string == "destroy":
        return DestroyEffect.parse(it.reset(), scope)

parse_handlers = {
    "Effect" : effect_handler,
}

class CreateEffect(scope.Scope, effects.Effect):
    """This class represents a create effect."""
    
    def __init__(self, args, effect, parentScope):
        """Create a new QuantifiedCondition

        Arguments:
        args -- List of variables that created objects will be bound to.
        effect -- The effect on the new variables
        parent -- Scope this Effect resides in.
        """
        Scope.__init__(self, args, parentScope)
        self.args = args
        self.effect = effect

    def visit(self, fn):
        return fn(self, [self.effect.visit(fn)])

    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        if not new_scope:
            new_scope = self.parent
            
        cp = CreateEffect([], None, new_scope)
        cp.args = cp.copy_args(self.args, copy_instance)

        if new_parts == []:
            new_parts = effects.ConjunctiveEffect([])
        elif new_parts:
            cp.effect = new_parts[0]
            cp.effect.set_scope(cp)
        else:
            cp.effect = self.effect.copy(cp, copy_instance=copy_instance)
        return cp

    def set_scope(self, new_scope):
        Scope.set_parent(self, new_scope)
        self.effect.set_scope(self)

    def get_scope(self):
        return self.parent

    def pddl_str_extra(self, results, instantiated=True):
        args = " ".join(sorted(self.iterkeys()))
        return "(create (%s) %s)" % (args, results[0])

    def write_pddl(self, writer):
        strings = writer.write_effect(self.effect)
        head  = "create (%s)" % writer.write_typelist(self.args)
        return writer.section(head, strings)
    
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.args == other.args and self.effect == other.effect

    @staticmethod
    def parse(it, scope):
        first = it.get("create")
        args = predicates.parse_arg_list(iter(it.get(list, "parameter list")), scope.types)
        eff = CreateEffect(args, None, scope)
        
        eff.effect = effects.Effect.parse(iter(it.get(list, "effect specification")), eff)

        return eff

class DestroyEffect(effects.Effect):
    """This class represents a destroy effect."""
    
    def __init__(self, term, scope=None):
        """Create a new DestroyEffect

        Arguments:
        term -- Term reffering to the object that will be destroyed
        scope -- Scope this Effect resides in.
        """
        self.term = term
        self.scope = scope

    def visit(self, fn):
        return fn(self, [])

    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        if not new_scope:
            new_scope = self.scope
        term = self.term.copy(new_scope)
        return DestroyEffect(term, new_scope)

    def set_scope(self, new_scope):
        self.new_scope = new_scope
        if new_scope:
            self.term = self.term.copy(new_scope)

    def get_scope(self):
        return self.scope

    def pddl_str_extra(self, results, instantiated=True):
        return "(destroy %s)" % self.term.pddl_str(instantiated)

    def write_pddl(self, writer):
        return ["(destroy %s)" % writer.write_term(self.term)]
    
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.term == other.term

    @staticmethod
    def parse(it, scope):
        first = it.get("destroy")
        term = predicates.Term.parse(it, scope, 999, function_scope=builtin.SCOPE_EFFECT)
        return DestroyEffect(term, scope)

#TODO handle quantified conditions/effects correctly
class DynamicObjectsCompiler(translators.Translator):
    def __init__(self, copy=True, spare_count=2, **kwargs):
        self.depends = []
        self.spare_count = spare_count
        self.set_copy(copy)

    def detect_dynamic_types(self, domain):
        @visitors.collect
        def create_visitor(eff, results):
            if isinstance(eff, CreateEffect):
                return [a.type for a in eff.args]
        @visitors.collect
        def destroy_visitor(eff, results):
            if isinstance(eff, DestroyEffect):
                if isinstance(eff.term, predicates.FunctionTerm):
                    return eff.term.function.type
                else:
                    return eff.term.get_type()
                
        created_types = set()    
        destroyed_types = set()    
        for a in domain.actions:
            created_types |= set(visitors.visit(a.effect, create_visitor, []))
            destroyed_types |= set(visitors.visit(a.effect, destroy_visitor, []))
        return created_types, destroyed_types

    def translate_action(self, action, types, domain=None):
        a2 = action.copy(newdomain=domain)
        
        added_args = []
        @visitors.replace
        def effect_visitor(eff, results):
            if isinstance(eff, CreateEffect):
                #TODO avoid name collisions
                new_args = a2.copy_args(eff.args)
                added_args.extend(new_args)
                c_effs = [Builder(eff).effect("not", ("not-instantiated", arg)) for arg in new_args]
                unused_effs = [Builder(eff).effect("not", ("unused", arg)) for arg in new_args]
                return effects.ConjunctiveEffect.join(c_effs + unused_effs + [eff.effect])
            if isinstance(eff, DestroyEffect):
                return Builder(eff).effect("not-instantiated", eff.term)

        a2.effect = visitors.visit(a2.effect, effect_visitor)
        b = Builder(a2)

        notdest = [b.cond("not", ("not-instantiated", a)) for a in a2.args if any(t.is_compatible(a.type) for t in types)]
        spare = [b.cond("unused", a) for a in added_args]
        a2.precondition = conditions.Conjunction.new(a2.precondition)
        a2.precondition.parts += notdest
        a2.precondition.parts += spare
        
        a2.args += added_args
        return a2

    def translate_axiom(self, axiom, types, domain=None):
        a2 = axiom.copy(newdomain=domain)

        notdest = [Builder(a2).cond("not", ("not-instantiated", a)) for a in a2.args if any(t.is_compatible(a.type) for t in types)]
        a2.condition = conditions.Conjunction.new(a2.condition)
        a2.condition.parts += notdest
        
        return a2
    
    @translators.requires('dynamic-objects')
    def translate_domain(self, _domain):
        if self.copy:
            dom = _domain.copy_skeleton()
        else:
            dom = _domain
            
        dom.requirements.discard("dynamic-objects")

        destroyed2 = predicates.Predicate("not-instantiated", [types.Parameter("?o", builtin.t_object)], builtin=False)
        unused2 = predicates.Predicate("unused", [types.Parameter("?o", builtin.t_object)], builtin=False)
        dom.predicates.add(destroyed2)
        dom.predicates.add(unused2)

        ctypes, dtypes = self.detect_dynamic_types(_domain)
        dyntypes = ctypes | dtypes

        actions = dom.get_action_like() 
        dom.clear_actions()
        for a in actions:
            dom.add_action(self.translate_action(a, dyntypes, dom))

        dom.axioms = [self.translate_axiom(a, dyntypes, dom) for a in _domain.axioms]
        dom.stratify_axioms()
        return dom

    @translators.requires('dynamic-objects')
    def translate_problem(self, _problem):
        ctypes, dtypes = self.detect_dynamic_types(_problem.domain)
        domain = self.translate_domain(_problem.domain)
        if self.copy:
            p2 = problem.Problem(_problem.name, _problem.objects, _problem.init, _problem.goal, domain, _problem.optimization, _problem.opt_func)
        else:
            _problem.set_parent(domain)
            p2 = _problem
            
        b = Builder(p2)
        for t in ctypes:
            for n in xrange(0, self.spare_count):
                obj = types.TypedObject("spare_%s%d" % (str(t), n), t)
                p2.add_object(obj)
                p2.init.append(b("not-instantiated", obj))
                p2.init.append(b("unused", obj))
                
        return p2

default_compiler = DynamicObjectsCompiler
