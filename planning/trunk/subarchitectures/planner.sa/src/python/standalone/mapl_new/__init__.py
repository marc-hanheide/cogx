import mapltypes as types
import parser
import predicates, conditions, effects, actions, sensors, axioms, domain, problem, writer

def load_domain(filename):
    p = parser.Parser.parseFile(filename)
    return domain.MAPLDomain.parse(p.root)
    
def parse_domain(domain_desc):
    p = mapl.parser.Parser(domain_desc.split("\n"))
    return domain.MAPLDomain.parse(p.root)
    

def load_problem(filename, domain):
    p = parser.Parser.parseFile(filename)
    return problem.Problem.parse(p.root, domain)
    
def parse_problem(problem_desc, domain):
    p = mapl.parser.Parser(problem_desc.split("\n"))
    return problem.Problem.parse(p.root, domain)
    
