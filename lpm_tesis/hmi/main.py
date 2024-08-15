#!/usr/bin/env python3

import sys
from PyQt5 import QtWidgets, QtCore
import rospy
import rospy
from nav_msgs.msg import Odometry
from Window_Tesis import Ui_Interfaz_Tesis

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_Interfaz_Tesis()
        self.ui.setupUi(self)
        
        # Inicializar el nodo ROS
        rospy.init_node('hmi_node', anonymous=True)
        
        # Suscribirse al tópico /odom
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Configurar temporizador para actualizar la UI
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)  # Actualiza cada 100 ms
        
        # Variables para almacenar los datos del tópico
        self.odom_data = None
        
        # Configurar botones
        self.ui.Boton_Mapeo.clicked.connect(self.handle_map_button)
        self.ui.Boton_NavReact.clicked.connect(self.handle_nav_react_button)
        self.ui.Boton_Menu.clicked.connect(self.handle_menu_button)
        
    def odom_callback(self, msg):
        """ Callback para actualizar los valores de odometría """
        self.odom_data = msg
        
    def update_ui(self):
        """ Actualiza los widgets de la UI con los datos más recientes """
        if self.odom_data:
            # Obtener datos de odometría
            x = self.odom_data.pose.pose.position.x
            y = self.odom_data.pose.pose.position.y
            z = self.odom_data.pose.pose.orientation.z
            
            # Actualizar widgets
            self.ui.Display_X.setText(f"{x:.2f}")
            self.ui.Display_Y.setText(f"{y:.2f}")
            self.ui.Display_Z.setText(f"{z:.2f}")
    
    def handle_map_button(self):
        # Lógica para el botón de mapeo
        rospy.loginfo("Botón de mapeo presionado")
    
    def handle_nav_react_button(self):
        # Lógica para el botón de navegación reactiva
        rospy.loginfo("Botón de navegación reactiva presionado")
    
    def handle_menu_button(self):
        # Lógica para el botón de menú
        rospy.loginfo("Botón de menú presionado")

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
