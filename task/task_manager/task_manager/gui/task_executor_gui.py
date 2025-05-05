from PyQt5.QtWidgets import (QWidget, QApplication, QLabel, 
                             QVBoxLayout, QPlainTextEdit, QPushButton)
from PyQt5.QtCore import Qt

class TaskExecutorGUI(QWidget):

    VERSION = "0.1"

    def __init__(self, send_text_func):
        super().__init__()
        self.init_components()
        self.send_text_func = send_text_func

    def btn_click_handler(self):
        self.send_text_func(self.editor.toPlainText())

    def init_components(self):
        m_layout = QVBoxLayout(self)

        label = QLabel()
        label.setStyleSheet("font-size: 25px; font-weight: bold")
        label.setText("Task Executor Viewer")
        label.setAlignment(Qt.AlignmentFlag.AlignHCenter)

        self.editor = QPlainTextEdit()
        self.editor.setStyleSheet("font-family: monospace")
        self.editor.setPlainText("""[
    {
        "task": "move",
        "args": {
            "x": -1,
            "y": 0,
            "theta": 1
        }
    },
    {
        "task": "talk",
        "args": {
            "speech": "Hi! I am Sancho, nice to meet you!"
        }
    }
]""")

        button = QPushButton()
        button.setText("Send data")
        button.clicked.connect(self.btn_click_handler)

        m_layout.addWidget(label)
        m_layout.addWidget(self.editor)
        m_layout.addWidget(button)


        self.setWindowTitle(f"Task Executor Viewer - v{self.VERSION}")

def a(msg):
    print(msg)

def main():
    app = QApplication([])

    gui = TaskExecutorGUI(a)
    gui.show()

    return app.exec_()

if __name__ == "__main__":
    main()