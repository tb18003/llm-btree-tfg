import sys
from PyQt5.QtWidgets import (QApplication, QSplashScreen, QMainWindow, 
                             QProgressBar, QHBoxLayout, QWidget, QLabel,
                             QVBoxLayout, QPlainTextEdit, QPushButton,
                             QDial, QFrame, QSizePolicy, QMessageBox)
from PyQt5.QtGui import QPixmap, QFont, QPainter, QColor
from PyQt5.QtSvg import QSvgWidget
from PyQt5.QtCore import Qt, pyqtSignal

import time
import random

class SplashScreen(QSplashScreen):
    def __init__(self):
        pixmap = QPixmap("./assets/splash.png")  # Reemplaza con tu imagen
        super().__init__(pixmap, Qt.WindowStaysOnTopHint)
        
        # Configurar barra de progreso
        self.progress = QProgressBar(self)
        self.progress.setMaximum(100)
        self.progress.setGeometry(50, pixmap.height() - 50, pixmap.width() - 100, 20)
        
        # Configurar texto de estado
        self.status_label = QLabel("Cargando...", self)
        self.status_label.setGeometry(50, pixmap.height() - 80, pixmap.width() - 100, 30)
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("QLabel { color: white; }")
        font = QFont()
        font.setBold(True)
        self.status_label.setFont(font)
        
    def update_progress(self, value, message=None):
        self.progress.setValue(value)
        if message:
            self.status_label.setText(message)
        QApplication.processEvents()  # Actualizar la interfaz

class MonitoringMainWindow(QWidget):

    tasks_update_signal = pyqtSignal(list)

    status_colors = {
        'done': 'green',
        'failure': 'red',
        'running': 'orange',
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Tasks Monitoring")

        self.setMinimumWidth(600)

        self.tasks_update_signal.connect(self.add_tasks)

        base_l = QVBoxLayout()
        self.setLayout(base_l)

        m_label = QLabel()
        m_label.setText("¡Hola! ¿En qué puedo ayudarte?")
        m_label.setStyleSheet("font-size: 36px")
        m_label.setAlignment(Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignHCenter)

        base_l.addWidget(m_label)
        base_l.addWidget(self._input_field_widget())
        base_l.addWidget(self._switch_widget())
        base_l.addWidget(self._task_widget())

        # Configura tu ventana principal aquí

    def set_node(self, node):
        self.node = node

    def _input_field_widget(self):
        c = QWidget()
        l = QHBoxLayout()

        c.setLayout(l)
        c.setMaximumHeight(80)

        self.tfield = QPlainTextEdit()
        self.tfield.setMaximumHeight(40)
        self.tfield.setStyleSheet("QPlainTextEdit{ font-size: 20px; font-family: system }")
        self.tfield.setEnabled(False)
        
        self.send_btn = QPushButton()
        self.send_btn.setText("Enviar")
        self.send_btn.setMinimumHeight(40)
        self.send_btn.clicked.connect(self.__simulate_tasks)

        l.addWidget(self.tfield)
        l.addWidget(self.send_btn)

        return c
    
    def __simulate_tasks(self):
        self.add_tasks(random.choice([[
            {
                "task": "move",
                "args": {
                    "x": random.random()*10,
                    "y": random.random()*10
                },
                "status": "done"
            }, {
                "task": "talk",
                "args": {
                    "speech": random.choice(["¡Hola! Soy Sancho, ¿Qué puedo hacer por ti?", "Lo siento, no puedo ir a ese sitio"])
                },
                "status": "failure"
            }, {
                "task": "face-recognition",
                "args": {
                    "target": random.choice(["Antonio", "Toni", "Román", "Eulogio"])
                },
                "status": "running"
            }, {
                "task": "talk",
                "args": {
                    "speech": "hola :)"
                },
                "status": "pending"
            }
        ], []]))

    def _dial_value_changed(self, value):
        value = True if value == 1 else False
        self.tfield.setEnabled(value)
        self.tfield.setPlainText("")
    
    def _switch_widget(self):
        c = QWidget()
        l = QHBoxLayout()

        c.setLayout(l)
        c.setMaximumHeight(70)

        whisper_label = QLabel()
        whisper_label.setText("Utilizar voz")
        whisper_label.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight)

        txt_label = QLabel()
        txt_label.setText("Utilizar teclado")
        txt_label.setAlignment(Qt.AlignmentFlag.AlignVCenter)

        dial = QDial()
        dial.setRange(0,1)
        dial.setValue(0)
        dial.setMaximumHeight(60)
        dial.setMaximumWidth(60)
        dial.valueChanged.connect(self._dial_value_changed)

        l.addWidget(whisper_label)
        l.addWidget(dial)
        l.addWidget(txt_label)

        return c
    
    # Create each task function
    def create_task(self, task_name, text, status):
        frame = QFrame()
        frame.setFrameShape(QFrame.Box)
        frame.setStyleSheet(f"QFrame{{ border: 2px solid {self.status_colors.get(status, 'gray')}; border-radius: 10px; }}")
        frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QHBoxLayout(frame)

        #icon = QSvgWidget(f"./assets/{task_name}/{status}.svg")
        #icon.setFixedSize(25,25)

        status_label = QLabel()
        status_label.setText(f"({status.capitalize()})")
        status_label.setStyleSheet("QLabel{ border: none, font-size: 20px }")

        layout.addWidget(status_label)

        label = QLabel(text)
        label.setFont(QFont("system", 10))
        label.setStyleSheet("QLabel{ border: none, font-size: 20px }")
        layout.addWidget(label)

        layout.addStretch()
        return frame
    
    def _task_widget(self):
        container = QWidget()
        layout = QVBoxLayout(container)

        title = QLabel("Tareas")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFont(QFont("system", 12))
        title.setMaximumHeight(20)
        layout.addWidget(title)


        task_frame = QFrame()
        task_frame.setFrameShape(QFrame.Box)
        self.task_layout = QVBoxLayout(task_frame)

        self.add_tasks([])

        layout.addWidget(task_frame)
        return container
    
    def add_tasks(self, tasks: list[dict[str,str]]):
        for i in reversed(range(self.task_layout.count())): 
            w = self.task_layout.itemAt(i).widget()
            w.hide()
            w.deleteLater()

        if len(tasks) == 0:
            label = QLabel()
            label.setText("Aún no se están ejecutando tareas.")
            label.setStyleSheet("font-size: 20px")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            
            self.task_layout.addWidget(label)
            return

        for task in tasks:
            if task['task'] == "move":
                self.task_layout.addWidget(self.create_task('move', f"Mover a X={task['args']['x']} Y={task['args']['y']}", task['status']))
            elif task['task'] == "talk":
                self.task_layout.addWidget(self.create_task('talk', f"El robot dice '{task['args']['speech']}'", task['status']))
            elif task['task'] == "face-recognition":
                self.task_layout.addWidget(self.create_task('face-recognition', f"El robot busca a '{task['args']['target']}'", task['status']))
            else:
                self.node.get_logger().info("Ignoring task %s" % task['task'])

        l_status = tasks[len(tasks) - 1]['status']
        if l_status == 'done':
            QMessageBox.information(None, 'Información', 'La ejecución de las tareas se ha completado')

        QApplication.processEvents()

def load_resources(splash):
    """Simula la carga de recursos"""
    resources = ["Base de datos", "Interfaz de usuario", "Configuración", "Módulos"]
    for i, resource in enumerate(resources):
        progress = (i + 1) * (100 // len(resources))
        splash.update_progress(progress, f"Cargando {resource}...")
        #time.sleep(1)

def main():
    app = QApplication(sys.argv)
    
    # Mostrar splash screen
    splash = SplashScreen()
    splash.show()
    
    # Simular carga de recursos
    load_resources(splash)
    
    # Crear ventana principal
    main_window = MonitoringMainWindow()
    
    # Cerrar splash y mostrar ventana principal
    splash.finish(main_window)
    main_window.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()