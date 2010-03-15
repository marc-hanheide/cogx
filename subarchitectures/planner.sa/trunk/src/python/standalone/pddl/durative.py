from parser import *
import mapltypes as types
import builtin, predicates, actions, conditions, effects
from conditions import *

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
        actions.Action.__init__(self, name, args, precondition, effect, domain, replan=replan)
        self.add(types.TypedObject("?duration", types.t_number))
        self.duration = duration
        for d in self.duration:
            d.set_scope(self)
        
    def copy(self, newdomain=None):
        a = actions.Action.copy(self, newdomain)
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
                    action.precondition = TimedCondition.parse(iter(it.get(list, "condition")), action)
                elif next.token.string == ":replan":
                    if action.replan:
                        raise ParseError(next.token, "replan condition already defined.")
                    action.replan = conditions.Condition.parse(iter(it.get(list, "condition")), action)
                elif next.token.string == ":effect":
                    if action.effect:
                        raise ParseError(next.token, "effects already defined.")
                    action.effect = effects.Effect.parse(iter(it.get(list, "effect")), action, timed_effects=True)
                else:
                    raise UnexpectedTokenError(next.token)
                    
        except StopIteration, e:
            pass
            
        return action

class TimedCondition(conditions.Condition):
    def __init__(self, time, condition):
        assert time in ("start", "end", "all")
        self.time = time
        self.condition = condition

    def visit(self, fn):
        return fn(self, [self.condition.visit(fn)])

    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        if new_parts:
            return TimedCondition(self.time, new_parts[0].copy(new_scope, copy_instance=copy_instance))
        return TimedCondition(self.time, self.condition.copy(new_scope, copy_instance=copy_instance))
    
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.time == other.time and self.condition == other.condition

    def __hash__(self):
        return hash((self.__class__, self.time, self.condition ))

    
    @staticmethod
    def parse(it, scope):
        
        first = it.get()
        tag = first.token.string
        if tag == "and":
            parts = []
            for part in it:
                if part.is_terminal():
                    raise UnexpectedTokenError(part.token, "condition")
                parts.append(TimedCondition.parse(iter(part), scope))
            return Conjunction(parts)
        elif tag == "forall":
            return QuantifiedCondition.parse(it, scope, UniversalCondition, TimedCondition.parse)

        if not tag in ("at", "over"):
            raise UnexpectedTokenError(first.token, "timed condition ('at' or 'over')")

        specifier = it.get().token
        if tag == "at" and specifier.string not in ("start", "end"):
            raise UnexpectedTokenError(time, "'start' or 'end'")

        if tag == "over" and specifier.string != "all":
            raise UnexpectedTokenError(time, "'all'")

        condition = Condition.parse(iter(it.get(list)), scope)
        
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
    
    def __str__(self):
        return "TimedEffect at %s: %s" %(self.time, predicates.Literal.__str__(self))
        
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.time == other.time and predicates.Literal.__eq__(self, other)

    def __ne__(self, other):
        return not __eq__(self, other)
        
    @staticmethod
    def parse(it, scope):
        first = it.get(None, "effect specification").token

        if first.string != "at":
            scope.predicates.add(builtin.change)
            scope.predicates.add(builtin.num_change)
            eff = effects.SimpleEffect.parse(it.reset(), scope)
            scope.predicates.remove(builtin.change)
            scope.predicates.remove(builtin.num_change)
            return eff

        timespec = it.get().token

        if timespec.string not in ("start", "end"):
            raise UnexpectedTokenError(time, "'start' or 'end'")
        
        ops = [builtin.assign, builtin.change]
        if "fluents" in scope.requirements or "numeric-fluents" in scope.requirements:
            ops += builtin.numeric_ops
            ops.append(builtin.num_change)

        try:
            scope.predicates.add(ops)
            scope.predicates.remove(builtin.equals)
            literal = predicates.Literal.parse(iter(it.get(list, "effect")), scope)
        finally:
            scope.predicates.remove(ops)
            scope.predicates.add(builtin.equals)

        if literal.predicate in ops and literal.negated:
            raise ParseError(first, "Can't negate fluent assignments.")

        if literal.predicate == builtin.equals:
            raise ParseError(first, "Can't use '=' in effects, please use 'assign' instead.")

        return TimedEffect(literal.predicate, literal.args, timespec.string, scope, literal.negated)
