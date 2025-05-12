import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout,
    QLineEdit, QGroupBox, QHBoxLayout
)
from openai import OpenAI

M = {100: 'o', 127: 'x', -126: 'T', -125: "u", -124: 'w'}
objects = {"wall": 'o', "agent": 'x', "robot": "x",
           "chair": "T", "table": "T",
           "cup":  "u", "can": "u", "bottle": "u", "person": "w"}
AllObjects = list(objects.keys())

OpenAI_API_KEY_missing = False
try:
    client = OpenAI()
except:
    OpenAI_API_KEY_missing = True
    print("OpenAI client is not defined!")

def encode_sentence(Sentence="The wall should be near the table"):
    Prompt = f"Objects: {str(AllObjects)}"
    Prompt += f"\n Sentence: {Sentence}"
    Prompt += "\nStatements can be of the form !(AddGoalEvent (((SUBJECT x PREDICATE) --> near) (1.0 0.90)))"
    Prompt += "\n Please build a statement that is close in meaning to the sentence, whereby SUBJECT and PREDICATE need to be among the objects."
    Prompt += "\n and include only the statement in your response, nothing else!"
    completion = client.chat.completions.create(
        model="o1-preview",
        messages=[
            {"role": "user", "content": Prompt}
        ]
    )
    response = completion.choices[0].message.content
    for obj in AllObjects:
        response = response.replace(obj, objects[obj])
    return response

class InputGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Input GUI")

        self.layout = QVBoxLayout()
        self.label = QLabel("Add natural language command:" if not OpenAI_API_KEY_missing else "OpenAI API key missing!")
        self.layout.addWidget(self.label)

        if not OpenAI_API_KEY_missing:
            self.text_input = QLineEdit()
            self.layout.addWidget(self.text_input)
            self.comm_button = QPushButton("Communicate")
            self.comm_button.clicked.connect(self.comm_input)
            self.layout.addWidget(self.comm_button)

        self.examples_box = QGroupBox("Examples")
        self.examples_layout = QVBoxLayout()

        self.add_wait_button = QPushButton("Wait!")
        self.add_wait_button.clicked.connect(lambda: self.set_entry("!(wait)"))
        self.examples_layout.addWidget(self.add_wait_button)

        self.add_reach_human_button = QPushButton("Reach human!")
        self.add_reach_human_button.clicked.connect(lambda: self.set_entry("!(AddGoalEvent (((x x w) --> near) (1.0 0.90)))"))
        self.examples_layout.addWidget(self.add_reach_human_button)

        self.add_reach_chair_button = QPushButton("Reach chair!")
        self.add_reach_chair_button.clicked.connect(lambda: self.set_entry("!(AddGoalEvent (((x x T) --> near) (1.0 0.90)))"))
        self.examples_layout.addWidget(self.add_reach_chair_button)

        self.examples_box.setLayout(self.examples_layout)
        self.layout.addWidget(self.examples_box)

        self.entry = QLineEdit()
        self.layout.addWidget(self.entry)

        self.submit_button = QPushButton("Submit")
        self.submit_button.clicked.connect(self.submit_input)
        self.layout.addWidget(self.submit_button)

        self.setLayout(self.layout)
        self.resize(400, 300)

    def set_entry(self, text):
        self.entry.setText(text)

    def comm_input(self):
        sentence = self.text_input.text()
        obtainedtext = encode_sentence(sentence)
        self.entry.setText(obtainedtext)

    def submit_input(self):
        input_text = self.entry.text()
        if input_text == "":
            self.label.setText("Input missing.")
            return
        with open("/home/nartech/NACE/input.metta", "w") as file:
            file.write(input_text)
        self.entry.clear()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = InputGUI()
    gui.show()
    sys.exit(app.exec_())
