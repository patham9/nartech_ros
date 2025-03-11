from copy import deepcopy
from hyperon import *
#ONA import:
cwd = os.getcwd()
os.chdir('/home/nartech/OpenNARS-for-Applications/misc/Python/')
sys.path.append(os.getcwd())
from MeTTa import *
os.chdir(cwd)

def narsplugin_init(runnerinstance):
    global runner
    runner = runnerinstance
    runner.run("""
;puts the elements of a tuple into a space
(= (nartech.tuple.space $space $x)
   (let $y (superpose $x) (add-atom $space $y)))

;adds input to NARS returning reasoning results into a space
(= (nartech.nars.space $space ($Command $Content))
   (nartech.tuple.space $space (nartech.nars.tuple ($Command $Content))))

;turns operation into command for our ROS layer
(= (nartech.nars.ros ($op {}))
   (nartech.ros $op))

(= (nartech.nars.ros (go ($arg1 * $arg2)))
   (nartech.ros (go (coordinates $arg1 $arg2))))

;simple operation format:
(= (nartech.nars.operation $x)
   (let (INPUT (.: ((^ $op) (1.0 0.9)))) (superpose $x)
        ($op ())))

;operations with args format:
(= (nartech.nars.operation $x)
   (let (INPUT (.: ((($self * $args) --> (^ $op)) (1.0 0.9)))) (superpose $x)
        ($op $args)))

;execute the operation that NARS wants to execute
(= (nartech.nars.executehelper $iteration ($Command $Content))
   (case (nartech.nars.ros (nartech.nars.operation (nartech.nars.tuple ($Command $Content))))
         (((nartech.ros.command $x) (nartech.ros.command $x))
          (Empty (if (> $iteration 1)
                     (nartech.nars.executehelper (- $iteration 1) ($Command $Content))
                     ())))))

(= (nartech.nars.execute $cycles ($Command $Content))
   (nartech.nars.executehelper $cycles ($Command $Content)))

;perceive the seen and remembered objects as percept events
(= (nartech.nars.perceive $objects)
   (let* (($obj (superpose $objects))
                ((detection $category (coordinates $x $y)) $obj))
         (superpose ((nartech.nars (AddBeliefEvent (((($x * $y) * $category) --> perceive) (1.0 0.9))))
                     (nartech.nars (Command concurrent))))))
    """)

commandlist = ["Cycles", "Command", "AddBeliefEvent", "AddBeliefEternal", "AddGoalEvent",
               "EventQuestion", "EternalQuestion", "(EternalQuestionAboveExpectation", "(EventQuestionAboveExpectation"]

def call_nars_tuple(*a):
    global runner
    cmd = str(a[0])
    unknownCommand = True
    threshold = ""
    for narscommand in commandlist:
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
    for narscommand in commandlist:
        if cmd.startswith(f"({narscommand} "):
            ret = NAR_AddInput(f"!({narscommand} "+cmd.split(f"({narscommand} ")[1])
            parser = SExprParser("()")
            unknownCommand = False
            break
    if unknownCommand:
        parser = SExprParser(f"(Unknown command: {cmd})")
    return parser.parse(tokenizer)
