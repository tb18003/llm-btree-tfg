import sys
from PyQt5.QtWidgets import (QApplication, QSplashScreen, QMainWindow, 
                             QProgressBar, QHBoxLayout, QWidget, QLabel,
                             QVBoxLayout, QPlainTextEdit, QPushButton,
                             QDial, QFrame, QSizePolicy)
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtSvg import QSvgWidget
from PyQt5.QtCore import Qt, QTimer

import time

class SplashScreen(QSplashScreen):
    def __init__(self):
        pixmap = QPixmap("sancho_splash.png")  # Reemplaza con tu imagen
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

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Tasks Monitoring")

        self.setMinimumWidth(600)

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
        self.send_btn.setEnabled(False)

        l.addWidget(self.tfield)
        l.addWidget(self.send_btn)

        return c

    def _dial_value_changed(self, value):
        value = True if value == 1 else False
        self.tfield.setEnabled(value)
        self.tfield.setPlainText("")
        #self.send_btn.setEnabled(value)
        
    
    def _switch_widget(self):
        c = QWidget()
        l = QHBoxLayout()

        c.setLayout(l)
        c.setMaximumHeight(70)

        whisper_label = QLabel()
        whisper_label.setText("Utilizar voz")
        whisper_label.setStyleSheet("border: 1px solid black")
        whisper_label.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight)

        txt_label = QLabel()
        txt_label.setText("Utilizar teclado")
        txt_label.setStyleSheet("border: 1px solid black")
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
    
    def _task_widget(self):
        widget_principal = QWidget()
        layout_principal = QVBoxLayout(widget_principal)

        # Título centrado
        titulo = QLabel("Tareas")
        titulo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        titulo.setFont(QFont("system", 12))
        titulo.setMaximumHeight(20)
        layout_principal.addWidget(titulo)

        # Contenedor de tareas
        contenedor_tareas = QFrame()
        contenedor_tareas.setFrameShape(QFrame.Box)
        layout_tareas = QVBoxLayout(contenedor_tareas)

        # Función auxiliar para crear cada tarea
        def crear_tarea(ruta_icono, texto):
            frame = QFrame()
            frame.setFrameShape(QFrame.Box)
            frame.setStyleSheet("QFrame{ border: 2px solid gray; border-radius: 10px; }")
            frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            layout = QHBoxLayout(frame)

            icono = QSvgWidget(ruta_icono)
            icono.setFixedSize(32,32)
            icono.setStyleSheet("color: green")
            layout.addWidget(icono)

            etiqueta = QLabel(texto)
            etiqueta.setFont(QFont("system", 10))
            etiqueta.setStyleSheet("QLabel{ border: none }")
            layout.addWidget(etiqueta)

            layout.addStretch()
            return frame

        # Tareas
        tarea1 = crear_tarea("./assets/move-icon.svg", "Mover a (0,1)")
        tarea2 = crear_tarea("./assets/talk-icon.svg", 'Decir "Buenas tardes!"')

        for tarea in [tarea1, tarea2]:
            layout_tareas.addWidget(tarea)

        layout_principal.addWidget(contenedor_tareas)
        return widget_principal

def load_resources(splash):
    """Simula la carga de recursos"""
    resources = ["Base de datos", "Interfaz de usuario", "Configuración", "Módulos"]
    for i, resource in enumerate(resources):
        progress = (i + 1) * (100 // len(resources))
        splash.update_progress(progress, f"Cargando {resource}...")
        time.sleep(1)

def main():
    app = QApplication(sys.argv)
    
    # Mostrar splash screen
    splash = SplashScreen()
    splash.show()
    
    # Simular carga de recursos
    load_resources(splash)
    
    # Crear ventana principal
    main_window = MainWindow()
    
    # Cerrar splash y mostrar ventana principal
    splash.finish(main_window)
    main_window.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()