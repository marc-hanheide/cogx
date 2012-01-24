import mapltypes as types
import parser
import predicates, conditions, effects, actions, axioms, domain, problem, scenario, writer, translators, state, prob_state, visitors, builder

from mapltypes import Type, TypedObject, Parameter
from predicates import Predicate, Function, Literal, Term, FunctionTerm, VariableTerm, ConstantTerm
from conditions import LiteralCondition, Conjunction, Disjunction, ExistentialCondition, UniversalCondition
from effects import SimpleEffect, ConjunctiveEffect, UniversalEffect, ConditionalEffect
from actions import Action
from mapl import InitRule
from domain import Domain
from problem import Problem
from builder import Builder

from builtin import t_object, t_boolean, t_number
from builtin import TRUE, FALSE, UNKNOWN
from builtin import equals, assign
from builtin import assignment_ops, numeric_ops, numeric_functions

def load_domain(filename):
    p = parser.Parser.parse_file(filename)
    return domain.Domain.parse(p.root)
    
def parse_domain(domain_desc):
    p = parser.Parser(domain_desc.split("\n"))
    return domain.Domain.parse(p.root)
    

def load_problem(filename, domain):
    p = parser.Parser.parse_file(filename)
    return problem.Problem.parse(p.root, domain)
    
def parse_problem(problem_desc, domain):
    p = parser.Parser(problem_desc.split("\n"))
    return problem.Problem.parse(p.root, domain)


def load_scenario(filename, domain):
    p = parser.Parser.parse_file(filename)
    return scenario.MapsimScenario.parse(p.root, domain)
