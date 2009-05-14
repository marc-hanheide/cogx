"""Text2speech code for MAPSIM.

Warning: This is Linux-specific code.  In particular, some tools must be installed
  - festival (text-to-speech engine)
  - some additional voices  (for Ubuntu a good tutorial on how to do this is
    http://ubuntu-utah.ubuntuforums.org/showthread.php?t=751169 )
  - sox (combine wav files) (ubuntu packages: sox libsox-fmt-all)
  - lame (make mp3s) (ubuntu package: lame)
"""


import re
import os

import config
import reporter

counter = 1
tmp_prefix = ".festival_data"
tmp_wav = tmp_prefix+"_single"
tmp_combined_wav = tmp_prefix+"_combined.wav"

simulator = "MAPSIM"
MALE, FEMALE, ROBOT  = "male female robot".split()

# the first male voice in the following list will be chosen for the
# simulator who verbalizes physical behaviour.  if you want to change that
# just put another (maybe female) voice here

male_voices = """
nitech_us_rms_arctic_hts
nitech_us_awb_arctic_hts
nitech_us_bdl_arctic_hts
nitech_us_jmk_arctic_hts 
"""

female_voices = """
nitech_us_clb_arctic_hts
nitech_us_slt_arctic_hts
us1_mbrola
"""

robot_voices = """
us2_mbrola
us3_mbrola
don_diphone 
kal_diphone 
ked_diphone
rab_diphone
"""

def setup_voice_dict():
    """ prepare voice data """
    tups = ((MALE,male_voices), (FEMALE,female_voices), (ROBOT,robot_voices))
    for (sex,voices) in tups:
        voice_commands = ["(voice_%s)" % voice for voice in voices.split() if voice]
        yield sex, voice_commands
voice_commands = dict(setup_voice_dict())
voices = {}

sexes = {}   # this can be set when calling main()

some_female_names = "Anne Lilli".split()
for name in some_female_names:
    sexes[name] = FEMALE

verbose = True

example_report = """
 MAPSIM run starts.  There are 2 agents, Anne and R2D2.
 (1) Anne: 'Please give me the coffee, R2D2.'
 (2) R2D2: 'Okay, Anne.'
 (3) R2D2: 'Where is the coffee, Anne?'
 (4) Anne: 'The coffee is in the kitchen, R2D2.'
 (5) R2D2: 'Thanks.'
 (6) R2D2: 'Please open the kitchen_door, Anne.'
 (7) Anne: 'Okay, R2D2.'
 (8) Anne opens the kitchen_door.
 (9) R2D2: 'Thanks for opening the kitchen_door, Anne.'
(10) R2D2 moves to the kitchen.
(11) R2D2 grasps the coffee.
(12) R2D2 moves to the living_room.
(13) R2D2 gives Anne the coffee.
(14) Anne: 'Thanks for giving me the coffee, R2D2.'
MAPSIM terminates successfully.
"""

example_report = """
MAPSIM run starts.  There are 3 agents: Bill, Oven and Robot.
 (1) Bill goes home.
 (2) Bill: 'Please bake the pizza, Oven.'
 (3) Oven: 'Okay, Bill.'
 (4) Bill: 'Please bring me the pizza, Robot.'
 (5) Robot: 'Okay, Bill.'
 (6) Robot brings Bill the pizza.
 (7) Oven tries to bake the pizza - but fails!
MAPSIM terminates unsuccessfully.
"""

def clean(line):
    """ removes stuff that cannot be nicely pronounced """
    line = line.replace("_", " ")
    return line

def split_report(report):
    """ Splits a MAPSIM report into a list of tuples (speaker, text)."""
    report = reporter.remove_line_numbers(report)
    dialogue_line = r"(\w*)\s*:\s*'(.*)'"
    regexp = re.compile(dialogue_line)
    for line in report:
        match = regexp.search(line)
        if match is None:
            yield simulator, line
        else:
            assert len(match.groups()) == 2
            yield match.groups()

def determine_voice(name):
    """ determine the voice to use for an agent"""
    global voices
    if name not in voices:
        sex = determine_sex(name)
        voice_list = voice_commands[sex]
        try:
            voice = voice_list.pop(0)
        except IndexError:
            voice = voices[simulator]
        voices[name] = voice
    return voices[name]

def determine_sex(name):
    """ tries to determine the sex of an agent from the global 'sexes' dict
    or by magically guessing it from his/her/its name"""
    if name == simulator:
        return MALE
    if name in sexes:
        return sexes[name]
    if name.isupper():
        return ROBOT
    if name.endswith("a"):
        return FEMALE
    return MALE   # more male voices available currently

# old version (maybe helpful again if we can write out the dialogue into
# one wav file directly...

# def produce_festival_data(dialogue):
#     """ turn dialogue into input for the 'festival' engine """
#     for speaker, text in dialogue:
#         yield determine_voice(speaker)
#         yield saytext_cmd % clean(text)
#         yield "(utt.save.wave utt 'filename)"
            
# def speak(festival_data):
#     """ save data temporarily and call festival to speak it."""
#     f = open(tmp_file, "w")
#     for line in festival_data:
#         print >>f, line
#     f.close()
#     cmd = "festival -b %s" % tmp_file
#     os.system(cmd)
#     #os.remove(tmp_file)

def produce_festival_data(dialogue):
    """ turn dialogue into input for the 'festival' engine """
    for speaker, text in dialogue:
        yield determine_voice(speaker), clean(text)
            
def speak(festival_data):
    global counter
    for voice_cmd, msg in festival_data:
        fn = tmp_wav + "_%s.wav" % str(counter).zfill(3)
        counter += 1
        create_wav = 'echo "%(msg)s" | text2wave -o %(fn)s -eval "%(voice_cmd)s" 2> /dev/null' % locals()
        os.system(create_wav)
        if not config.mp3:
            # assuming that if you want to create an mp3, you don't want to listen to it NOW
            speak_cmd = 'play %(fn)s -q' % locals()
            os.system(speak_cmd)
        
def voice_report(report):
    """ voice a MAPSIM report"""
    dialogue = list(split_report(report))
    festival_data = produce_festival_data(dialogue)
    speak(festival_data)

def main(args, verbosity=None, agent_sexes=None):
    """ can be called with
        - a file name to verbalize
        - no args; will verbalize an example report then
        - any number of args to be directly verbalized. 
          e.g. python speak.py This is a test."""
    global verbose, sexes
    if agent_sexes is not None:
        sexes = agent_sexes
    if verbosity is not None:
        verbose = verbosity
    try:
        report_fn = args[1]
        report = open(report_fn).readlines()
    except IndexError:
        report = example_report.splitlines()
    except IOError:
        if isinstance(args, list):
            report = " ".join(args[1:])
        else:
            report = args
        report = report.splitlines()
    report = [line.strip() for line in report]
    for line in report:
        if verbose:
            print line
        voice_report([line])
    if verbose:
        print "speak.py done."

def after_run():
    """ either clean up temporary files or use them to produce an mp3"""
    if config.mp3:
        join_wavs = "sox %s* %s" % (tmp_wav, tmp_combined_wav)
        make_mp3 = "lame --preset medium --quiet %s %s" % (tmp_combined_wav, config.mp3)
        os.system(join_wavs)
        os.system(make_mp3)
    rm_cmd = "rm %s*" % tmp_prefix
    os.system(rm_cmd)

# just want to create an mp3? then just run "python speak.py"
# with the following 2 lines uncommented

#import utils
#config = utils.Struct(mp3="test.mp3")

if __name__ == "__main__":
    import sys
    main(sys.argv)
    after_run()
        
