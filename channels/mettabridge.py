from exploration import *
from copy import deepcopy
from narsbridge import narsbridge_init, call_nars, call_nars_tuple
from hyperon.ext import register_atoms
from hyperon import *
import time
import io

NAV_STATE_FAIL = "FAIL"
NAV_STATE_BUSY = "BUSY"
NAV_STATE_SUCCESS = "SUCCESS"
NAV_STATE = NAV_STATE_SUCCESS #"busy", "success" and "fail" (initially set to success)
def NAV_STATE_SET(NAV_STATE_ARG):
    global NAV_STATE
    NAV_STATE = NAV_STATE_ARG
    return None

class PatternOperation(OperationObject):
    def __init__(self, name, op, unwrap=False, rec=False):
        super().__init__(name, op, unwrap)
        self.rec = rec
    def execute(self, *args, res_typ=AtomType.UNDEFINED):
        return super().execute(*args, res_typ=res_typ)

def wrapnpop(func):
    def wrapper(*args):
        a = [str("'"+arg) if arg is SymbolAtom else str(arg) for arg in args]
        res = func(*a)
        return [res]
    return wrapper

MeTTaROS2Command = ""
def call_ros(*a):
    global runner, MeTTaROS2Command
    tokenizer = runner.tokenizer()
    cmd = str(a[0])
    parser = SExprParser(f"(nartech.ros.command {cmd})")
    MeTTaROS2Command = cmd
    return parser.parse(tokenizer)

def space_init():
    global runner
    with open("space.metta", "r") as f:
        metta_code = f.read()
    runner = MeTTa()
    runner.run("""
(= (nartech.ros.objects.filter $category $objects)
   (collapse (let* (($obj (superpose $objects))
                    ((detection $category (coordinates $x $y)) $obj))
                   $obj)))
    """)
    call_ros_atom = G(PatternOperation('nartech.ros', wrapnpop(call_ros), unwrap=False))
    call_nars_atom = G(PatternOperation('nartech.nars', wrapnpop(call_nars), unwrap=False))
    call_nars_tuple_atom = G(PatternOperation('nartech.nars.tuple', wrapnpop(call_nars_tuple), unwrap=False))
    runner.register_atom("nartech.ros", call_ros_atom)
    runner.register_atom("nartech.nars", call_nars_atom)
    runner.register_atom("nartech.nars.tuple", call_nars_tuple_atom)
    narsbridge_init(runner)
    result = runner.run(metta_code)
    for x in result:
        print(x)  # Only prints the return value

currentTime = 0
start_time = time.time()

def space_tick(node):
    global currentTime, MeTTaROS2Command
    elapsedTime = round(time.time() - start_time, 2)
    if elapsedTime < 10:
        return
    cmd = MeTTaROS2Command
    MeTTaROS2Command = ""
    if cmd == "right":
        node.start_navigation_by_moves("right")
    if cmd == "left":
        node.start_navigation_by_moves("left")
    if cmd == "up":
        node.start_navigation_by_moves("up")
    if cmd == "down":
        node.start_navigation_by_moves("down")
    if cmd.startswith("(go (coordinates "):
        x_y = cmd.split("(go (coordinates ")[1].split(")")[0]
        x = int(x_y.split(" ")[0])
        y = int(x_y.split(" ")[1])
        node.start_navigation_to_coordinate((x, y))
    alldetections = deepcopy(node.semantic_slam.previous_detections)
    objects = "("
    if "self" in node.semantic_slam.previous_detections:
        (t, object_grid_x, object_grid_y, origin_x, origin_y) = node.semantic_slam.previous_detections["self"]
        x_y_unknown = BFS_for_nearest_unknown_cell(node.semantic_slam.low_res_grid, node.semantic_slam.new_width, node.semantic_slam.new_height, object_grid_x, object_grid_y)
        if x_y_unknown:
            (x_unknown,y_unknown) =  x_y_unknown
            alldetections["unknown"] = (time.time(), x_unknown, y_unknown, origin_x, origin_y)
    print(alldetections)
    for category in alldetections:
        (t, object_grid_x, object_grid_y, _, __) = alldetections[category]
        SEXP = f"(detection {category} (coordinates {object_grid_x} {object_grid_y}))"
        objects += SEXP
        if category == "self":
            SELF_position = (object_grid_x, object_grid_y)
    objects += ")"
    currentTime += 1
    print("NAV_STATE", NAV_STATE, runner.run(f"!(Step {currentTime} {elapsedTime} {NAV_STATE} {objects})"))

space_init()
if __name__ == "__main__":
    if len(sys.argv) <= 1:
        print("pass metta file name")
    with open(sys.argv[1]) as f:
        code = f.read()
    result = runner.run(code)
    for x in result:
        print(x)  # Only prints the return value

