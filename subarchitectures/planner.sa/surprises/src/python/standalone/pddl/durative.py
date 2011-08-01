from parser import *
import mapltypes as types
import builtin, predicates, actions, conditions, effects, translators
from builtin import t_number, change, num_change
from conditions import *
from scope import SCOPE_EFFECT

pddl_module = True

total_time = predicates.Function("total-time", [], t_number, builtin=True)

default_predicates = [change, num_change]
default_functions = [total_time]

default_compiler = translators.RemoveTimeCompiler

strict_mode = True

def action_handler(it, domain):
    if "mapl" in domain.requirements:
        return False # let the mapl module handle it
    domain.actions.append(DurativeAction.parse(it, domain))
    return True

def effect_handler(it, scope):
    if not scope.get_tag("durative_action"):
        return None
    
    first = it.get(None, "effect specification")
    if first.token.string == "at":
        try:
            second = it.get()
        except:
            return None
        if second.token.string in ("start", "end"):
            return TimedEffect.parse(it.reset(), scope)

def condition_handler(it, scope):
    if not scope.get_tag("durative_action"):
        return None

    is_durative = False
    first = it.get()
    if first.token.string in ("at", "over"):
        try:
            second = it.get()
        except:
            return None
        if first.token.string == "at" and second.token.string in ("start", "end"):
            is_durative=True
        if first.token.string == "over" and second.token.string == "all":
            is_durative=True
            
    if is_durative:
        if scope.get_tag("timed_condition"):
            raise ParseError(first.token, "Nested timed conditions are not allowed.")
        return TimedCondition.parse(it.reset(), scope)

    if strict_mode and not scope.get_tag("timed_condition") and first.token.string not in ("and", "forall"):
        # only "and" and "forall" are allowed outside a timed specifier
        raise UnexpectedTokenError(first.token, "'and', 'forall' or timed condition ('at' or 'over')")
    
parse_handlers = {
    ":durative-action" : action_handler,
    "Effect" : effect_handler,
    "Condition" : condition_handler
}

def add_hook(self, result, action):
    if action.__class__ == DurativeAction:
        self.actions.append(action)
        self.name2action = None

domain_hooks = {
    'add_action' : add_hook
    }

class DurationConstraint(object):
    def __init__(self, term, timeSpecifier="start"):
        self.term = term
        self.timeSpecifier = timeSpecifier

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.term == other.term and self.timeSpecifier == other.timeSpecifier

    def set_scope(self, new_scope):
        self.term = new_scope.lookup([self.term])[0]
    
    def __ne__(self, other):
        return not __eq__(self, other)

    @staticmethod
    def parse(it, scope):
        try:
            first = it.next().token
        except StopIteration:
            return []
        if first.string == "and":
            result = []
            for elem in it:
                result += DurationConstraint.parse(iter(elem), scope)
            return result
        
        time = None
        if first.string == "at":
            time = it.get("terminal").token
            if not time.string in ("start", "end"):
                raise UnexpectedTokenError(time, "'start' or 'end'")
            time = time.string
            it = iter(it.get(list, "duration constraint"))
            first = it.next().token

        if first.string != "=":
            raise UnexpectedTokenError(first, "=")
        it.get("?duration")
        
        term = predicates.Term.parse(it, scope)
        if not term.get_type().equal_or_subtype_of(types.t_number):
            raise ParseError(first, "Duration must be a number, not %s." % str(term.get_type()))
        
        it.no_more_tokens()
        
        return [DurationConstraint(term, time)]
            
        
class DurativeAction(actions.Action):
    def __init__(self, name, args, duration, precondition, effect, domain, replan=None):
        d = types.Parameter("?duration", types.t_number)
        actions.Action.__init__(self, name, args+[d], precondition, effect, domain, replan=replan)
        self.set_tag("durative_action", True) # proper parsing context
        
        self.duration = duration
        for d in self.duration:
            d.set_scope(self)
            
    def get_total_cost(self):
        return self.duration[0].term
    
    def copy(self, newdomain=None):
        a = actions.Action.copy(self, newdomain)
        a.__class__ = DurativeAction
        a.duration = [DurationConstraint(a.lookup([d.term])[0], d.timeSpecifier) for d in self.duration]
        return a

    def copy_skeleton(self, newdomain=None):
        """Create a copy of this action's skeleton (name, arguments
        but not conditions and effects).

        Arguments:
        newdomain -- if not None, the copy will be created inside this scope."""

        a = actions.Action.copy_skeleton(self, newdomain)
        a.__class__ = DurativeAction
        a.duration = [DurationConstraint(a.lookup([d.term])[0], d.timeSpecifier) for d in self.duration]
        return a
    
    @staticmethod
    def parse(it, scope):
        it.get(":durative-action")
        name = it.get().token.string

        next = it.get()
        if next.token.string == ":parameters":
            params = predicates.parse_arg_list(iter(it.get(list, "parameters")), scope.types)
            next = it.get()
        else:
            params = []
            
        action =  DurativeAction(name, params, [], None, None, scope)
        
        next.token.check_keyword(":duration")
        action.duration = DurationConstraint.parse(iter(it.get(list, "duration constraint")), action)

        try:
            while True:
                next = it.next()
                if next.token.string == ":condition":
                    if action.precondition:
                        raise ParseError(next.token, "precondition already defined.")
                    action.precondition = Condition.parse(iter(it.get(list, "condition")), action)
                elif next.token.string == ":replan":
                    if action.replan:
                        raise ParseError(next.token, "replan condition already defined.")
                    action.set_tag("durative_action", False)
                    action.replan = conditions.Condition.parse(iter(it.get(list, "condition")), action)
                    action.set_tag("durative_action", True)
                elif next.token.string == ":effect":
                    if action.effect:
                        raise ParseError(next.token, "effects already defined.")
                    action.effect = effects.Effect.parse(iter(it.get(list, "effect")), action)
                else:
                    raise UnexpectedTokenError(next.token)
                    
        except StopIteration:
            pass
            
        return action

class TimedCondition(conditions.Condition):
    def __init__(self, time, condition, scope=None):
        assert time in ("start", "end", "all")
        self.time = time
        self.scope = scope
        self.condition = condition

    def visit(self, fn):
        return fn(self, [self.condition.visit(fn)])

    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        if new_parts:
            return TimedCondition(self.time, new_parts[0].copy(new_scope, copy_instance=copy_instance), new_scope)
        return TimedCondition(self.time, self.condition.copy(new_scope, copy_instance=copy_instance), new_scope)
    
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.time == other.time and self.condition == other.condition

    def __hash__(self):
        return hash((self.__class__, self.time, self.condition ))

    
    @staticmethod
    def parse(it, scope):
        scope.set_tag("timed_condition", True)
        
        first = it.get()
        tag = first.token.string
        # if tag == "and":
        #     parts = []
        #     for part in it:
        #         if part.is_terminal():
        #             raise UnexpectedTokenError(part.token, "condition")
        #         parts.append(TimedCondition.parse(iter(part), scope))
        #     return Conjunction(parts)
        # elif tag == "forall":
        #     return QuantifiedCondition.parse(it, scope, UniversalCondition, TimedCondition.parse)

        if not tag in ("at", "over"):
            raise UnexpectedTokenError(first.token, "timed condition ('at' or 'over')")

        specifier = it.get().token
        if tag == "at" and specifier.string not in ("start", "end"):
            raise UnexpectedTokenError(time, "'start' or 'end'")

        if tag == "over" and specifier.string != "all":
            raise UnexpectedTokenError(time, "'all'")

        condition = Condition.parse(iter(it.get(list)), scope)
        
        scope.set_tag("timed_condition", False)
        return TimedCondition(specifier.string, condition)



class TimedEffect(effects.SimpleEffect):
    def __init__(self, predicate, args, timeSpec, scope=None, negated=False):
        predicates.Literal.__init__(self, predicate, args, scope, negated)
        self.time = timeSpec
        
    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        if copy_instance:
            l = self.copy_instance()
            if new_scope:
                l.set_scope(new_scope)
            return l
        return TimedEffect(self.predicate, self.args, self.time, new_scope, self.negated)

    def copy_instance(self, new_scope=None):
        return self.__class__(self.predicate, [a.copy_instance() for a in self.args], self.time, new_scope, negated=self.negated)

    def new_literal(self, predicate=None, args=None, scope=False, negated=None, time=None):
        if predicate is None:
            predicate = self.predicate
        if args is None:
            args = self.args
        if scope is False:
            scope = self.scope
        if negated is None:
            negated = self.negated
        if time is None:
            time = self.time

        return TimedEffect(predicate, args, time, scope, negated)

    def pddl_str(self, instantiated=True):
        """Return a pddl text representation of this Effect.
        
        Arguments:
        instantiated -- if True (which is the default) resolves
        instantiated Parameters before printing the string."""
        return "(at %s %s)" % (self.time, effects.SimpleEffect.pddl_str(self, instantiated))
        
    
    def __str__(self):
        return "TimedEffect at %s: %s" %(self.time, predicates.Literal.__str__(self))
        
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.time == other.time and predicates.Literal.__eq__(self, other)

    def __ne__(self, other):
        return not __eq__(self, other)
        
    @staticmethod
    def parse(it, scope):
        first = it.get("at", "timed effect specification").token

        if first.string != "at":
            eff = effects.SimpleEffect.parse(it.reset(), scope)
            if eff.predicate not in (change, num_change):
                raise UnexpectedTokenError(first, "'at start', 'at end' or 'change'")
            return eff

        timespec = it.get().token

        if timespec.string not in ("start", "end"):
            raise UnexpectedTokenError(time, "'start' or 'end'")
        
        literal = predicates.Literal.parse(iter(it.get(list, "effect")), scope, function_scope=SCOPE_EFFECT)

        if literal.predicate in builtin.assignment_ops + builtin.numeric_ops and literal.negated:
            raise ParseError(first, "Can't negate fluent assignments.")

        if literal.predicate in (change, num_change):
            raise ParseError(first, "'change' can't be a start or end effect.")
        
        if literal.predicate == builtin.equals:
            raise ParseError(first, "Can't use '=' in effects, please use 'assign' instead.")

        return TimedEffect(literal.predicate, literal.args, timespec.string, scope, literal.negated)
