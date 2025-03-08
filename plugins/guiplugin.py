from hyperon import *
from MeTTa import *
import tkinter as tk
import multiprocessing
import threading

def guiplugin_init(runnerinstance, guifinishedinstance):
    global runner, guifinished
    runner = runnerinstance
    guifinished = guifinishedinstance

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

def wait_for_gui_result(process, conn, callwhendone):
    """Thread that waits for GUI input and calls callback."""
    result = conn.recv()  # Wait for user input
    process.join()  # Ensure process exits cleanly
    callwhendone(result)  # Call the callback with result

def create_gui(title, inputhint, callwhendone, width=800, height=200):
    """Start a GUI subprocess and return immediately."""
    parent_conn, child_conn = multiprocessing.Pipe()
    p = multiprocessing.Process(target=gui_process, args=(child_conn, title, inputhint, width, height))
    p.start()
    threading.Thread(target=wait_for_gui_result, args=(p, parent_conn, callwhendone), daemon=True).start()

def call_gui(*a):
    global runner
    cmd = str(a[0])
    title = cmd.split(")")[0][2:]
    inputhint = ")".join(cmd.split(")")[1:])[1:-1]
    create_gui(title, inputhint, guifinished)
    parser = SExprParser("(GUI result awaited in (nartech.gui.value x))")
    return parser.parse(runner.tokenizer())

# Example usage from a ROS thread:
if __name__ == "__main__":
    print("User input:", create_gui("Enter MeTTa expression:", "(+ 1 1)"))
