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

def narsbridge_init(runnerinstance):
    global runner
    runner = runnerinstance
    runner.run("""
(= (nartech.tuple.space $space $x)
   (let $y (superpose $x) (add-atom $space $y)))
(= (nartech.nars.space $space ($Command $Content))
   (nartech.tuple.space $space (nartech.nars.tuple ($Command $Content))))
    """)

def call_nars_tuple(*a):
    global runner
    cmd = str(a[0])
    unknownCommand = True
    threshold = ""
    for narscommand in ["AddBeliefEvent", "AddBeliefEternal", "AddGoalEvent",
                        "EventQuestion", "EternalQuestion", "(EternalQuestionAboveExpectation", "(EventQuestionAboveExpectation"]:
        if cmd.startswith(f"({narscommand} "):
            if narscommand == "(EternalQuestionAboveExpectation":
                threshold = cmd.split("(EternalQuestionAboveExpectation ")[1].split(" ")[0]
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

def call_nars(*a):
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
