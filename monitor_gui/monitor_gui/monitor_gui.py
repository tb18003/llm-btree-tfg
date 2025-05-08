import sys
from PyQt5.QtWidgets import (QApplication, QSplashScreen, QMainWindow, 
                             QProgressBar, QVBoxLayout, QWidget, QLabel)
from PyQt5.QtGui import QPixmap, QFont
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

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Aplicación Principal")
        self.resize(800, 600)
        # Configura tu ventana principal aquí

def load_resources(splash):
    """Simula la carga de recursos"""
    resources = ["Base de datos", "Interfaz de usuario", "Configuración", "Módulos"]
    for i, resource in enumerate(resources):
        progress = (i + 1) * (100 // len(resources))
        splash.update_progress(progress, f"Cargando {resource}...")
        time.sleep(2)

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