import sys
from PyQt5.QtWidgets import (QApplication, QSplashScreen, QAction,
                             QProgressBar, QHBoxLayout, QWidget, QLabel,
                             QVBoxLayout, QPlainTextEdit, QPushButton,
                             QRadioButton, QFrame, QSizePolicy, QMessageBox, QMenuBar,
                             QDialog)
from PyQt5.QtGui import QPixmap, QFont, QIcon
from PyQt5.QtSvg import QSvgWidget
from PyQt5.QtCore import Qt, pyqtSignal

import os

from ament_index_python import get_package_share_directory

BASE_DIR = get_package_share_directory('monitor_gui')


class SplashScreen(QSplashScreen):
    def __init__(self):
        splash_path = os.path.join(BASE_DIR, "assets", "splash.png")
        pixmap = QPixmap(splash_path)

        super().__init__(pixmap, Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        

        self.progress = QProgressBar(self)
        self.progress.setMaximum(100)
        self.progress.setTextVisible(False)  # Oculta el texto tipo "30%"
        
        self.progress.setGeometry(10, pixmap.height() - 20, pixmap.width() - 20, 10)

        self.progress.setStyleSheet("""
            QProgressBar {
                background-color: rgba(255, 255, 255, 40);  /* fondo translúcido */
                border: none;
                border-radius: 5px;
            }
            QProgressBar::chunk {
                background-color: #88c0d0;  /* azul suave tipo nord */
                border-radius: 5px;
            }
        """)

        self.status_label = QLabel("Cargando...", self)
        self.status_label.setGeometry(20, pixmap.height() - 50, pixmap.width() - 40, 25)
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("""
            QLabel {
                color: white;
                font-weight: bold;
                font-size: 14px;
            }
        """)
        
    def update_progress(self, value, message=None):
        self.progress.setValue(value)
        if message:
            self.status_label.setText(message)
        QApplication.processEvents()  # Actualizar la interfaz

class AboutDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Acerca de")

        # Layout principal
        main_layout = QVBoxLayout()

        # Imagen superior
        image_label = QLabel()
        pixmap = QPixmap(os.path.join(BASE_DIR, "assets", "icon.png"))  # Asegúrate de tener 'logo.png' en tu ruta
        pixmap = pixmap.scaled(150, 150, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        image_label.setPixmap(pixmap)
        image_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(image_label)

        # Información textual
        info_label = QLabel("Robot Tasks Monitoring<br>Versión 1.0<br>Desarrollado por tbms@uma.es")
        info_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(info_label)

        lbl = QLabel("Los códigos de colores del estado de las tareas significan:")
        main_layout.addWidget(lbl)

        # Leyenda de colores
        legend_layout = QVBoxLayout()

        for color, meaning in [("red", "Tarea fallida"),
                               ("green", "Tarea completada correctamente"),
                               ("orange", "Tarea en ejecución")]:
            item_layout = QHBoxLayout()
            color_box = QLabel()
            color_box.setFixedSize(20, 20)
            color_box.setStyleSheet(f"background-color: {color}; border: 1px solid gray;")
            text_label = QLabel(meaning)
            item_layout.addWidget(color_box)
            item_layout.addWidget(text_label)
            legend_layout.addLayout(item_layout)

        main_layout.addLayout(legend_layout)
        self.setLayout(main_layout)

class MonitoringMainWindow(QWidget):

    tasks_update_signal = pyqtSignal(list)
    order_update_signal = pyqtSignal(str)

    status_colors = {
        'done': 'green',
        'failure': 'red',
        'running': 'orange',
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Tasks Monitoring")

        self.setMinimumWidth(600)

        self.setWindowIcon(QIcon(os.path.join(BASE_DIR, "assets", "icon.png")))

        mb = QMenuBar(self)
        help_menu = mb.addMenu("Ayuda")
        about_action = QAction("Acerca de ", self)
        about_action.triggered.connect(self._show_more_about)
        help_menu.addAction(about_action)

        self.tasks_update_signal.connect(self.add_tasks)
        self.order_update_signal.connect(self.update_order_input)

        base_l = QVBoxLayout()
        self.setLayout(base_l)

        m_label = QLabel()
        m_label.setText("¡Hola! ¿En qué puedo ayudarte?")
        m_label.setStyleSheet("font-size: 36px")
        m_label.setAlignment(Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignHCenter)

        base_l.setMenuBar(mb)
        base_l.addWidget(m_label)
        base_l.addWidget(self._input_field_widget())
        base_l.addWidget(self._switch_widget())
        base_l.addWidget(self._task_widget())

    def _show_more_about(self):
        AboutDialog(self).exec_()

    def set_node(self, node):
        self.node = node
        self.send_btn.setEnabled(True)

    def update_order_input(self, order_input):
        """Actualiza el campo de entrada de órdenes"""
        if self.voice_radio.isChecked() and self.send_btn.text() != "Ejecutando...":
            self.tfield.setPlainText(order_input)
            self.send_btn.setEnabled(False)
            self.send_btn.setText("Ejecutando...")

    def _input_field_widget(self):
        c = QWidget()
        l = QHBoxLayout()

        c.setLayout(l)
        c.setMaximumHeight(80)

        self.tfield = QPlainTextEdit()
        self.tfield.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.tfield.setMaximumHeight(50)
        self.tfield.setEnabled(False)

        self.tfield.setStyleSheet("""
            QPlainTextEdit {
                background-color: #f4f4f4;
                border: 2px solid #cccccc;
                border-radius: 10px;
                padding: 8px;
                font-size: 18px;
                font-family: 'Segoe UI', 'Helvetica Neue', 'Arial';
                color: #333;
            }
                                  
            QPlainTextEdit:focus {
                border-color: #1f6aa7;  /* Cambia el color del borde al enfocar */
            }
            
            QPlainTextEdit::disabled {
                background-color: #e4e4e4;  /* Color de fondo cuando está deshabilitado */
                color: #999999;  /* Color del texto cuando está deshabilitado */
            }
        """)
        
        self.send_btn = QPushButton()
        self.send_btn.setText("Enviar")
        self.send_btn.setMinimumHeight(40)
        self.send_btn.setMinimumWidth(120)
        self.send_btn.clicked.connect(self._send_task)
        self.send_btn.setEnabled(False)
        self.send_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.send_btn.setStyleSheet("QPushButton{ font-family: system; border-radius: 5px; color: white; background-color: #1f6aa7;} QPushButton:hover{ background-color: #1b5f96 } QPushButton:pressed{ background-color: #2277bd; }")

        l.addWidget(self.tfield)
        l.addWidget(self.send_btn)

        return c
    
    def _send_task(self):
        """Envía la orden por el topic de whisper"""
        self.node.send_whisper(self.tfield.toPlainText())
        self.send_btn.setEnabled(False)
        self.send_btn.setText("Ejecutando...")
        self.tfield.setEnabled(False)
        self.text_radio.setEnabled(False)
        self.voice_radio.setEnabled(False)

    def _radio_btn_text(self, value):
        if value:
            self.tfield.setEnabled(True)
            self.send_btn.setEnabled(True)
    
    def _radio_btn_voice(self, value):
        if value:
            self.tfield.setEnabled(False)
            self.send_btn.setEnabled(False)
    
    def _switch_widget(self):
        c = QWidget()
        l = QHBoxLayout()

        c.setLayout(l)
        c.setMaximumHeight(70)

        radio_container = QWidget()
        radio_layout = QHBoxLayout()
        radio_container.setLayout(radio_layout)

        self.text_radio = QRadioButton()
        self.text_radio.setText("Utilizar teclado")
        self.text_radio.setChecked(False)
        self.text_radio.toggled.connect(self._radio_btn_text)

        self.voice_radio = QRadioButton()
        self.voice_radio.setText("Utilizar voz")
        self.voice_radio.setChecked(True)
        self.voice_radio.toggled.connect(self._radio_btn_voice)

        radio_layout.addWidget(self.text_radio)
        radio_layout.addWidget(self.voice_radio)

        l.addWidget(radio_container)

        return c
    
    # Create each task function
    def create_task(self, task_name, text, status):
        frame = QFrame()
        frame.setFrameShape(QFrame.Box)
        frame.setStyleSheet(f"QFrame{{ border: 2px solid {self.status_colors.get(status, 'gray')}; border-radius: 10px; }}")
        frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QHBoxLayout(frame)

        icon = QSvgWidget(os.path.join(BASE_DIR, "assets", task_name, status + ".svg"))
        icon.setFixedSize(25,25)

        layout.addWidget(icon)

        label = QLabel(text)
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
        if l_status == 'done' or l_status == 'failure':
            if len([task['status'] for task in tasks if task['status'] == 'failure']) > 0:
                QMessageBox.warning(None, 'Información', 'La ejecución de las tareas no ha finalizado correctamente para todas las tareas') 
            else:
                QMessageBox.information(None, 'Información', 'La ejecución de las tareas se ha completado')

            if self.text_radio.isChecked():
                self.tfield.setEnabled(True)
                self.send_btn.setEnabled(True)

            self.send_btn.setText("Enviar")
            self.tfield.setPlainText("")
            self.text_radio.setEnabled(True)
            self.voice_radio.setEnabled(True)


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