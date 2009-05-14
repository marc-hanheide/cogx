import sys
from string import Template
from collections import defaultdict

import config
from config import log
from constants import *
import utils
import commands

from planning.mapl.parser import non_lower_tokens

use_line_numbers = True
#use_line_numbers = False

def time_str():
    return str(config.simulation.time)

SUCCESS = True
LONG = True

BASE_SUBSTITUTIONS = [
    ("DOES_TMPL", "$agt: execute '$ground_action'."),
    ("DOES_TMPL", "$agt tries to execute '$ground_action' - but fails!", {"success":not SUCCESS}),
    ("DOES_TELL_VAL_TMPL", "$speaker: tell_val $hearer $svar($svar_args) = $value'"),
    ("DOES_TELL_VAL_TMPL", "$agt tries to execute '$ground_action' - but fails!", {"success":not SUCCESS}),
    ("REQUEST_TMPL", "$speaker: request $hearer '$ground_action'."),
    ("THANKS_TMPL", "$speaker: ack_achieved '$ground_action'."),
    ("ACK_SG_ACCEPT_TMPL", "$agt: accept_request '$ground_action'."),
    ("ACK_SG_REJECT_TMPL", "$agt: reject_request '$ground_action'."),
    ("REQUEST_TELL_VAL_TMPL", "$speaker: request $hearer 'tell_val $svar($svar_args)'"),
    ]

NL_SUBSTITUTIONS = [
    ("DOES_TMPL", "$agt $does $complement."),
    ("DOES_TMPL", "$agt tries to $do $complement - but fails!", {"success":not SUCCESS}),
    ("REQUEST_TMPL", "$speaker: 'Please $do $complement, $hearer.'"),
    ("ACK_SG_ACCEPT_TMPL", "$agt: 'Okay, $hearer.'"),
    ("ACK_SG_REJECT_TMPL", "$agt: 'Sorry, I can't, $hearer.'"),
    ("ACK_SG_REJECT_TMPL", "$agt: 'Sorry, I don't know, $hearer.'", {"question":True}),
    ("REQUEST_TELL_VAL_TMPL", "$speaker: '$question, $hearer?'"),
    ("DOES_TELL_VAL_TMPL", "$speaker: '$answer, $hearer.'"),
    ("DOES_TELL_VAL_TMPL", "$agt tries to execute '$ground_action' - but fails!", {"success":not SUCCESS}),
    ("THANKS_TMPL", "$speaker: 'Thanks for $doing $complement, $hearer.'"),
    ("THANKS_TMPL", "$speaker: 'Thanks.'", {"long": not LONG}),
    ("question", "What is the $svar of $svar_args"),
    ("answer", "The $svar of $svar_args is $value"),
    ("complement", "$args"),
    ]
    
key_order = ("success", "long", "question")
defaults = dict(success=SUCCESS, long=LONG, question=False)

def prepare_conditional_substitutions(base_subs):
    """ returns a dict of the form: tmplname -> [(tup,tmpl), ...]"""
    def gen_subs_data(base_subs):
        for tup in base_subs: 
            assert 2 <= len(tup) <= 3
            tmpl_name, tmpl = tup[0], tup[1]
            d = dict(defaults)
            if len(tup) == 3:
                d.update(tup[2])
            key_tup = tuple(d[k] for k in key_order)
            yield (tmpl_name, (key_tup, tmpl))
    d = defaultdict(list)
    for (tmpl_name, tup_tmpl_pair) in gen_subs_data(base_subs):
        l = d[tmpl_name]
        l.append(tup_tmpl_pair)
    return d

def prepare_unconditional_substitutions(cond_subs):
    return dict((k,v[0][1]) for k,v in cond_subs.items() if len(v) == 1)

class TemplateManager(object):
    """ helps selecting appropriate reporting templates """
    def __init__ (self, use_natural_language=None):
        if use_natural_language is None:
            use_natural_language = config.reporter_verbalization
        self.use_natural_language = use_natural_language
        substitutions = NL_SUBSTITUTIONS if use_natural_language else BASE_SUBSTITUTIONS
        subs = prepare_conditional_substitutions(substitutions)
        self.conditional_substitutions = subs
        self.unconditional_substitutions = prepare_unconditional_substitutions(subs)
        
    def get_template(self, template_name, modifiers):
        if template_name not in self.conditional_substitutions:
            return template_name
        success = modifiers["success"]
        debug = not success
        possible_matches = self.conditional_substitutions[template_name]
        args = dict(defaults)
        args.update(modifiers)
        key = tuple(args[k] for k in key_order)
        def similarity_tup(tup1, tup2):
            return tuple(e1 == e2 for e1, e2 in zip(tup1, tup2))
        stups = sorted(((similarity_tup(key,tup),tmpl) for tup,tmpl in possible_matches), reverse=True)
        best = stups[0]
        return best[1]



class Reporter(object):
    def __init__(self):
        self.name = "Reporter"
        self.template_manager = TemplateManager()
        self.domain_specific_subs = defaultdict(dict)
        self.line_number = 0
        self.report = []

    def report_on(self, command, execution_successful=True):
        """ the main reporting function """
        base_tmpl = command.reporter_template()
        if not base_tmpl:
            return
        modifiers = command.report_modifiers
        modifiers["success"] = execution_successful
        tmpl = self.template_manager.get_template(base_tmpl, modifiers)
        report_line = self.generate_report_line(command, tmpl)
        assert "negotiates" not in report_line
        self.pretty_print(report_line)
        
    def set_template(self, op_name, sub_name, sub_val):
        op_name = op_name.lower()
        op_subs = self.domain_specific_subs[op_name]
        op_subs[sub_name] = sub_val

    def set_verb_complement(self, op_name, sub_val):
        self.set_template(op_name, "complement", sub_val)
        
    def prepare_substitutions(self, command):
        subs = self.template_manager.unconditional_substitutions
        action = command.mapl_action
        agt = action.agent
        op_name = action.operator
        do = op_name.replace("_", " ")
        verb_parts = do.split()
        speaker = hearer = tell_cmd = None
        if isinstance(command, commands.SpeechAct):
            speaker = str(command.speaker()).lower()
            hearer = str(command.hearer()).lower()
        if isinstance(command, commands.TellValue):
            tell_cmd = command
        elif isinstance(command, commands.Request) and isinstance(command.requested_command(), commands.TellValue):
            tell_cmd = command.requested_command()
        if tell_cmd:
            svar, value = tell_cmd.svar, tell_cmd.value
            svar_args = svar.args
            svar = svar.name
            subs.update(dict(("svarg%d" % i, arg) for (i, arg) in enumerate(svar_args)))
            subs.update(self.domain_specific_subs[svar])
            subs["svar_args"] = " ".join(svar_args)
        stem = verb_parts[0]
        does_suffix = "es" if stem[-1] == "s" else "s" 
        does = [stem+does_suffix] + verb_parts[1:]
        doing = [(stem if stem[-1] != "e" else stem[:-1]) + "ing"] + verb_parts[1:]
        does = " ".join(does)
        doing = " ".join(doing)
        subs.update(dict(("verb%d" % i, part) for (i, part) in enumerate(verb_parts)))
        arglist = list(action.arguments)
        for (i, arg) in enumerate(arglist):
            argstr = "arg%d" % i
            # small hack: when the speaker refers to himself in a request use "me"
            if arg == speaker and speaker != hearer:
                arg = "me"
                arglist[i] = arg
            subs[argstr] = arg
        ground_action = action.description
        args = " ".join(arglist)
        names = "speaker hearer agt op_name do does doing args ground_action svar value".split()
        loc = locals()
        subs.update((name,loc[name]) for name in names if name in loc)
        subs.update(self.domain_specific_subs[op_name])
        return subs

    def generate_report_line(self, command, tmpl):
        subs = self.prepare_substitutions(command)
        while True:
            report_line = tmpl
            #print "report line so far:", tmpl
            tmpl = Template(tmpl).safe_substitute(subs)
            if tmpl == report_line:
                break
            
        return report_line

    def add_line_number(self, report_line):
        self.line_number += 1
        ln_str = ("(%d)" % self.line_number).rjust(4)
        report_line = "%s %s" % (ln_str, report_line)
        return report_line

    def pretty_print(self, report_line):
        if not report_line:
            return
        if any(t in report_line for t in non_lower_tokens):
            report_line = utils.multiple_replace(report_line, non_lower_tokens)
        if config.reporter_verbalization:
            report_line = report_line.replace("_", " ")
        if use_line_numbers:
            report_line = self.add_line_number(report_line)
        self.log_and_report(report_line)

    def log_and_report(self, report_line):
        """ store a report line, print it to the report log and possibly to the screen"""
        self.report.append(report_line)
        print >>config.report_file, report_line
        log(report_line, vlevel=REPORTER, agent="")
        if config.speech:
            import speak
            speak.main(report_line, verbosity=False)
        
    def report_sensing(self, agt, svar, val, negative=False, active_sense=False):
        comp = "="
        if negative:
            comp = "!="
        if isinstance(val,tuple):
            val = " ".join(val)
        if active_sense:
            report_line = "%s actively senses (%s) %s %s." % (str(agt), str(svar), comp, str(val))
        else:
            report_line = "%s senses (%s) %s %s." % (str(agt), str(svar), comp, str(val))
        self.pretty_print(report_line)


import re
with_numbers = r"\s*\(\w*\)\s*(.*)"
regexp = re.compile(with_numbers)

def remove_line_number(line):
    match = regexp.match(line)
    tidy_line = match.group(1) if match else line
    return tidy_line

def remove_line_numbers(report):
    for line in report:
        tidy_line = remove_line_number(line)
        if tidy_line:
            yield tidy_line

