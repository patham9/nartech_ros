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
                values.append(value)
            else:
                values.extend(extract_metta_values(value))
    elif isinstance(d, list):  # Handle lists of dictionaries
        for item in d:
            values.extend(extract_metta_values(item))
    return values

def call_narsinput(*a):
    global runner
    tokenizer = runner.tokenizer()
    cmd = str(a[0])
    unknownCommand = True
    for narscommand in ["AddBeliefEvent", "AddBeliefEternal", "AddGoalEvent", "EventQuestion", "EternalQuestion"]:
        if cmd.startswith(f"({narscommand} "):
            ret = NAR_AddInput(f"!({narscommand} "+cmd.split(f"({narscommand} ")[1])
            narsret = "(" + (" ".join(extract_metta_values(ret))) + ")"
            parser = SExprParser(narsret)
            unknownCommand = False
            break
    if unknownCommand:
        parser = SExprParser(f"(Unknown command: {cmd})")
    return parser.parse(tokenizer)
