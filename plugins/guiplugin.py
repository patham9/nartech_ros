from hyperon import *
from MeTTa import *
import tkinter as tk
import multiprocessing
import threading

def guiplugin_init(runnerinstance):
    global runner
    runner = runnerinstance
    runner.run("!(add-atom &self (nartech.gui.value ()))")

gui_lock = threading.Lock()
gui_ret = dict([])
def gui_finished(value, request_id):
    global gui_ret
    with gui_lock:
        gui_ret[request_id] = value

def gui_process(conn, title, inputhint, width, height):
    """Run the Tkinter GUI in a separate process and send result back."""
    root = tk.Tk()
    root.title(title)
    root.geometry(f"{width}x{height}")
    returntext = tk.StringVar()
    def get_text():
        returntext.set(text_entry.get("1.0", tk.END).strip())
        conn.send(returntext.get() if returntext.get() else "()")  # Send result
        conn.close()
        root.destroy()
    tk.Label(root, text=title).pack()
    text_entry = tk.Text(root, width=width // 10, height=height // 40)
    text_entry.insert("1.0", inputhint)
    text_entry.pack()
    tk.Button(root, text="Submit", command=get_text).pack()
    root.mainloop()

def wait_for_gui_result(process, conn, request_id):
    """Thread that waits for GUI input and calls callback."""
    result = conn.recv()  # Wait for user input
    process.join()  # Ensure process exits cleanly
    gui_finished(result, request_id)  # Call the callback with result

def create_gui(title, inputhint, request_id, width=800, height=200):
    """Start a GUI subprocess and return immediately."""
    parent_conn, child_conn = multiprocessing.Pipe()
    p = multiprocessing.Process(target=gui_process, args=(child_conn, title, inputhint, width, height))
    p.start()
    threading.Thread(target=wait_for_gui_result, args=(p, parent_conn, request_id), daemon=True).start()

request_id = 1
def call_gui(*a):
    global runner, request_id
    cmd = str(a[0])
    if cmd.isnumeric():
        with gui_lock:
            previous_request_id = int(cmd)
            if previous_request_id not in gui_ret:
                value = "()"
            else:
                value = gui_ret[previous_request_id] #fetch entered value by user
            parser = SExprParser(value)
    else:
        title = cmd.split(")")[0][2:]
        inputhint = ")".join(cmd.split(")")[1:])[1:-1]
        dictated_request_id = cmd.split("(")[1].split(" ")[0]
        if dictated_request_id.isnumeric():
            request_id = int(dictated_request_id)
        create_gui(title, inputhint, request_id)
        parser = SExprParser(str(request_id))
        request_id += 1
    return parser.parse(runner.tokenizer())

# Example usage from a ROS thread:
if __name__ == "__main__":
    print("User input:", create_gui("Enter MeTTa expression:", "(+ 1 1)"))
