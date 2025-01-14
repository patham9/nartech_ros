import tkinter as tk

def submit_input():
    input_text = entry.get()
    if input_text == "":
        label.config(text="Input missing.")
        return
    with open("/home/nartech/NACE/input.metta", "w") as file:
        file.write(input_text)
    #label.config(text="Input submitted.")

def add_wait():
    entry.delete(0, tk.END)  # Clear the current content
    entry.insert(0, "!(wait)")  # Set the new value

def add_reach_human():
    entry.delete(0, tk.END)  # Clear the current content
    entry.insert(0, "!(AddGoalEvent (((x x w) --> near) (1.0 0.90)))")  # Set the new value

def add_reach_chair():
    entry.delete(0, tk.END)  # Clear the current content
    entry.insert(0, "!(AddGoalEvent (((x x T) --> near) (1.0 0.90)))")  # Set the new value


# Create the main window
root = tk.Tk()
root.title("Input GUI")

# Add "Examples" category
examples_frame = tk.LabelFrame(root, text="Examples", padx=10, pady=10)
examples_frame.pack(pady=10)

# Add buttons to the "Examples" category
add_wait_button = tk.Button(examples_frame, text="Wait!", command=add_wait)
add_wait_button.pack(pady=5)
add_reach_human_button = tk.Button(examples_frame, text="Reach human!", command=add_reach_human)
add_reach_human_button.pack(pady=5)
add_reach_chair_button = tk.Button(examples_frame, text="Reach chair!", command=add_reach_chair)
add_reach_chair_button.pack(pady=5)

# Add input field
entry = tk.Entry(root, width=50)
entry.pack(pady=10)

# Add submit button
submit_button = tk.Button(root, text="Submit", command=submit_input)
submit_button.pack(pady=10)

# Add a label for status
#label = tk.Label(root, text="")
#label.pack(pady=10)

# Run the application
root.mainloop()
