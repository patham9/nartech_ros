import tkinter as tk

M = {100: 'o', 127: 'x', -126: 'T', -125: "u", -124: 'w'} #reversed mapping from grid.py
objects = {"wall" : 'o', "agent" : 'x', "robot": "x",
           "chair" : "T", "table": "T",
           "cup"  : "u", "can": "u", "bottle": "u", "person": "w"}

AllObjects = list(objects.keys())

OpenAI_API_KEY_missing = False
from openai import OpenAI
try:
    client = OpenAI()
except:
    OpenAI_API_KEY_missing = True
    print("OpenAI client is not defined!")

def encode_sentence(Sentence = "The wall should be near the table"):
    Prompt = f"Objects: {str(AllObjects)}"
    Prompt += f"\n Sentence: {Sentence}"
    Prompt += "\nStatements can be of the form !(AddGoalEvent (((SUBJECT x PREDICATE) --> near) (1.0 0.90)))"
    Prompt += "\n Please build a statement that is close in meaning to the sentence, whereby SUBJECT and PREDICATE need to be among the objects."
    Prompt += "\n and include only the statement in your response, nothing else!"
    completion = client.chat.completions.create(
      model="o1-preview", #gpt-4o, o1-preview
      messages=[
        {"role": "user", "content": Prompt}
      ]
    )
    response = completion.choices[0].message.content
    for obj in AllObjects:
        response = response.replace(obj, objects[obj])
    return response

def comm_input():
    obtainedtext = encode_sentence(textinput.get())
    entry.delete(0, tk.END)  # Clear the current content
    entry.insert(0, obtainedtext)  # Set the new value

def submit_input():
    label.config(text=label_text)
    input_text = entry.get()
    if input_text == "":
        label.config(text="Input missing.")
        return
    with open("/home/nartech/NACE/input.metta", "w") as file:
        file.write(input_text)
    #label.config(text="Input submitted.")
    entry.delete(0, tk.END)

def add_wait():
    label.config(text=label_text)
    entry.delete(0, tk.END)  # Clear the current content
    entry.insert(0, "!(wait)")  # Set the new value

def add_reach_human():
    label.config(text=label_text)
    entry.delete(0, tk.END)  # Clear the current content
    entry.insert(0, "!(AddGoalEvent (((x x w) --> near) (1.0 0.90)))")  # Set the new value

def add_reach_chair():
    label.config(text=label_text)
    entry.delete(0, tk.END)  # Clear the current content
    entry.insert(0, "!(AddGoalEvent (((x x T) --> near) (1.0 0.90)))")  # Set the new value


# Create the main window
root = tk.Tk()
root.title("Input GUI")

if OpenAI_API_KEY_missing:
    issuelabel = tk.Label(root, text="OpenAI API key missing, export it in .bashrc to allow for NL input!")
    issuelabel.pack(pady=10)
    label_text = ''
else:
    label_text = "Add natural language command:"

# Add a label for status
label = tk.Label(root, text=label_text)
label.pack(pady=10)

# Add input field
if not OpenAI_API_KEY_missing:
    textinput = tk.Entry(root, width=50)
    textinput.pack(pady=10)
    # Add submit button
    comm_button = tk.Button(root, text="Communicate", command=comm_input)
    comm_button.pack(pady=10)

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

entry = tk.Entry(root, width=50)
entry.pack(pady=10)

# Add submit button
submit_button = tk.Button(root, text="Submit", command=submit_input)
submit_button.pack(pady=10)

# Run the application
root.mainloop()
