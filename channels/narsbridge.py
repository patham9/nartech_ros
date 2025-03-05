from copy import deepcopy
from hyperon import *
import io
import contextlib
#ONA import:
cwd = os.getcwd()
os.chdir('/home/nartech/OpenNARS-for-Applications/misc/Python/')
sys.path.append(os.getcwd())
from MeTTa import *
os.chdir(cwd)

def nars_set_metta_runner(runnerinstance):
    global runner
    runner = runnerinstance

def extract_metta_values(d):
    values = []
    if isinstance(d, dict):
        for key, value in d.items():
            if key == 'metta':
                values.append(value.replace("#","$"))
            else:
                values.extend(extract_metta_values(value))
    elif isinstance(d, list):  # Handle lists of dictionaries
        for item in d:
            values.extend(extract_metta_values(item))
    return values

#helper function for debugging
def exit42(s="42"):
    return SExprParser(s).parse(runner.tokenizer())

def call_narsIO(*a):
    global runner
    cmd = str(a[0])
    unknownCommand = True
    threshold = ""
    for narscommand in ["AddBeliefEvent", "AddBeliefEternal", "AddGoalEvent",
                        "EventQuestion", "EternalQuestion", "(EternalQuestionMultiple", "(EventQuestionMultiple"]:
        if cmd.startswith(f"({narscommand} "):
            if narscommand == "(EternalQuestionMultiple":
                threshold = cmd.split("(EternalQuestionMultiple ")[1].split(" ")[0]
            ret = NAR_AddInput(f"!"+cmd)
            narsret = "("
            for category in ['input', 'derivations', 'answers']:
                renamed = {'input': "INPUT", "derivations": "DERIVATION", "answers": "ANSWER"}
                if category in ret:
                    for s in ret[category]:
                        if 'metta' in s:
                            addition = f"({renamed[category]} " + s['metta'] + ')'
                            opening_parantheses = len([x for x in addition if x=="("])
                            closing_parantheses = len([x for x in addition if x==")"])
                            if "$" not in addition and "#" not in addition: #TODO investigate
                                narsret += addition
            narsret += ")"
            parser = SExprParser(narsret)
            unknownCommand = False
            break
    if unknownCommand:
        parser = SExprParser(f"(Unknown command: {cmd})")
    return parser.parse(runner.tokenizer())

def call_narsinput(*a):
    global runner
    tokenizer = runner.tokenizer()
    cmd = str(a[0])
    unknownCommand = True
    for narscommand in ["AddBeliefEvent", "AddBeliefEternal", "AddGoalEvent", "EventQuestion", "EternalQuestion"]:
        if cmd.startswith(f"({narscommand} "):
            ret = NAR_AddInput(f"!({narscommand} "+cmd.split(f"({narscommand} ")[1])
            parser = SExprParser("()")
            unknownCommand = False
            break
    if unknownCommand:
        parser = SExprParser(f"(Unknown command: {cmd})")
    return parser.parse(tokenizer)
