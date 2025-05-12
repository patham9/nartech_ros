from hyperon import *
from MeTTa import *
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QTextEdit, QPushButton, QVBoxLayout
import multiprocessing, threading, sys

def guiplugin_init(runnerinstance):
    global runner
    runner = runnerinstance
    runner.run("!(add-atom &self (nartech.gui.value ()))")

gui_lock, gui_ret = threading.Lock(), {}

def gui_finished(val, rid):
    with gui_lock:
        gui_ret[rid] = val

def gui_process(conn, title, hint, w, h):
    app = QApplication(sys.argv)
    win = QWidget(); win.setWindowTitle(title); win.resize(w, h)
    lay = QVBoxLayout(win)
    lay.addWidget(QLabel(title))
    txt = QTextEdit(); txt.setPlainText(hint); lay.addWidget(txt)
    btn = QPushButton("Submit"); lay.addWidget(btn)
    btn.clicked.connect(lambda: (conn.send(txt.toPlainText().strip() or "()"), conn.close(), win.close(), app.quit()))
    win.show(); app.exec_()

def wait_proc(proc, conn, rid):
    gui_finished(conn.recv(), rid); proc.join()

def create_gui(title, hint, rid, w=800, h=200):
    pc, cc = multiprocessing.Pipe()
    p = multiprocessing.Process(target=gui_process, args=(cc, title, hint, w, h)); p.start()
    threading.Thread(target=wait_proc, args=(p, pc, rid), daemon=True).start()

request_id = 1
def call_gui(*a):
    global request_id
    cmd = str(a[0])
    if cmd.isnumeric():
        val = gui_ret.get(int(cmd), "()")
        return SExprParser(val).parse(runner.tokenizer())
    title = cmd.split(")")[0][2:]; hint = ")".join(cmd.split(")")[1:])[1:-1].strip()
    title = title.replace("(", "")
    rid_str = cmd.split("(")[1].split(" ")[0]
    if rid_str.isnumeric(): request_id = int(rid_str)
    create_gui(title, hint, request_id)
    parser = SExprParser(str(request_id)); request_id += 1
    return parser.parse(runner.tokenizer())

if __name__ == "__main__":
    multiprocessing.freeze_support()
    create_gui("Enter MeTTa expression:", "(+ 1 1)", 0)
