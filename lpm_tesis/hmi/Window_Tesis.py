# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'HMI_Tesis.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Interfaz_Tesis(object):
    def setupUi(self, Interfaz_Tesis):
        Interfaz_Tesis.setObjectName("Interfaz_Tesis")
        Interfaz_Tesis.resize(903, 596)
        self.menus = QtWidgets.QTabWidget(Interfaz_Tesis)
        self.menus.setGeometry(QtCore.QRect(0, 0, 901, 571))
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(186, 189, 182))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(186, 189, 182))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(186, 189, 182))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(186, 189, 182))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Window, brush)
        self.menus.setPalette(palette)
        self.menus.setObjectName("menus")
        self.ventana_principal = QtWidgets.QWidget()
        self.ventana_principal.setObjectName("ventana_principal")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.ventana_principal)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(510, 210, 281, 91))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.Opciones_Control = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.Opciones_Control.setSizeConstraint(QtWidgets.QLayout.SetMaximumSize)
        self.Opciones_Control.setContentsMargins(0, 0, 0, 0)
        self.Opciones_Control.setSpacing(10)
        self.Opciones_Control.setObjectName("Opciones_Control")
        self.Label_Opciones = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.Label_Opciones.setEnabled(True)
        self.Label_Opciones.setMaximumSize(QtCore.QSize(16777204, 16777215))
        self.Label_Opciones.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.Label_Opciones.setAlignment(QtCore.Qt.AlignCenter)
        self.Label_Opciones.setObjectName("Label_Opciones")
        self.Opciones_Control.addWidget(self.Label_Opciones)
        self.Boton_Mapeo = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.Boton_Mapeo.setObjectName("Boton_Mapeo")
        self.Opciones_Control.addWidget(self.Boton_Mapeo)
        self.Boton_NavReact = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.Boton_NavReact.setObjectName("Boton_NavReact")
        self.Opciones_Control.addWidget(self.Boton_NavReact)
        self.Autores_Tesis = QtWidgets.QTextBrowser(self.ventana_principal)
        self.Autores_Tesis.setGeometry(QtCore.QRect(525, 360, 251, 61))
        self.Autores_Tesis.setObjectName("Autores_Tesis")
        self.SelloESPE = QtWidgets.QTextBrowser(self.ventana_principal)
        self.SelloESPE.setGeometry(QtCore.QRect(200, 0, 511, 151))
        self.SelloESPE.setObjectName("SelloESPE")
        self.RobotImagen = QtWidgets.QTextBrowser(self.ventana_principal)
        self.RobotImagen.setGeometry(QtCore.QRect(70, 160, 360, 360))
        self.RobotImagen.setObjectName("RobotImagen")
        self.menus.addTab(self.ventana_principal, "")
        self.ventada_mapeo = QtWidgets.QWidget()
        self.ventada_mapeo.setObjectName("ventada_mapeo")
        self.gridLayoutWidget = QtWidgets.QWidget(self.ventada_mapeo)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(30, 160, 271, 151))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.Matriz_Controles = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.Matriz_Controles.setContentsMargins(0, 0, 0, 0)
        self.Matriz_Controles.setSpacing(3)
        self.Matriz_Controles.setObjectName("Matriz_Controles")
        self.Boton_Arriba = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.Boton_Arriba.setObjectName("Boton_Arriba")
        self.Matriz_Controles.addWidget(self.Boton_Arriba, 0, 1, 1, 1)
        self.Boton_Derecha = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.Boton_Derecha.setObjectName("Boton_Derecha")
        self.Matriz_Controles.addWidget(self.Boton_Derecha, 1, 2, 1, 1)
        self.Boton_Izquierda = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.Boton_Izquierda.setObjectName("Boton_Izquierda")
        self.Matriz_Controles.addWidget(self.Boton_Izquierda, 1, 0, 1, 1)
        self.Boton_Abajo = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.Boton_Abajo.setObjectName("Boton_Abajo")
        self.Matriz_Controles.addWidget(self.Boton_Abajo, 1, 1, 1, 1)
        self.Boton_Antihorario = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.Boton_Antihorario.setObjectName("Boton_Antihorario")
        self.Matriz_Controles.addWidget(self.Boton_Antihorario, 0, 0, 1, 1)
        self.Boton_Horario = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.Boton_Horario.setObjectName("Boton_Horario")
        self.Matriz_Controles.addWidget(self.Boton_Horario, 0, 2, 1, 1)
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.ventada_mapeo)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(70, 70, 181, 41))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.Matriz_Parametros = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.Matriz_Parametros.setContentsMargins(0, 0, 0, 0)
        self.Matriz_Parametros.setObjectName("Matriz_Parametros")
        self.Label_Avance = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.Label_Avance.setObjectName("Label_Avance")
        self.Matriz_Parametros.addWidget(self.Label_Avance)
        self.Input_Avance = QtWidgets.QDoubleSpinBox(self.horizontalLayoutWidget)
        self.Input_Avance.setObjectName("Input_Avance")
        self.Matriz_Parametros.addWidget(self.Input_Avance)
        self.horizontalLayoutWidget_2 = QtWidgets.QWidget(self.ventada_mapeo)
        self.horizontalLayoutWidget_2.setGeometry(QtCore.QRect(410, 470, 391, 72))
        self.horizontalLayoutWidget_2.setObjectName("horizontalLayoutWidget_2")
        self.Matriz_Mapa = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_2)
        self.Matriz_Mapa.setContentsMargins(0, 0, 0, 0)
        self.Matriz_Mapa.setObjectName("Matriz_Mapa")
        self.Label_NombreMapa = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.Label_NombreMapa.setObjectName("Label_NombreMapa")
        self.Matriz_Mapa.addWidget(self.Label_NombreMapa)
        self.Input_NombreMapa = QtWidgets.QTextEdit(self.horizontalLayoutWidget_2)
        self.Input_NombreMapa.setObjectName("Input_NombreMapa")
        self.Matriz_Mapa.addWidget(self.Input_NombreMapa)
        self.Boton_Mapa = QtWidgets.QPushButton(self.horizontalLayoutWidget_2)
        self.Boton_Mapa.setObjectName("Boton_Mapa")
        self.Matriz_Mapa.addWidget(self.Boton_Mapa)
        self.Graficador_Mapa = QtWidgets.QGraphicsView(self.ventada_mapeo)
        self.Graficador_Mapa.setGeometry(QtCore.QRect(330, 20, 541, 431))
        self.Graficador_Mapa.setObjectName("Graficador_Mapa")
        self.gridLayoutWidget_2 = QtWidgets.QWidget(self.ventada_mapeo)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(50, 340, 231, 224))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")
        self.Matriz_Coordenadas = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.Matriz_Coordenadas.setContentsMargins(0, 0, 0, 0)
        self.Matriz_Coordenadas.setObjectName("Matriz_Coordenadas")
        self.Label_RZ = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.Label_RZ.setObjectName("Label_RZ")
        self.Matriz_Coordenadas.addWidget(self.Label_RZ, 2, 0, 1, 1)
        self.Label_DX = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.Label_DX.setObjectName("Label_DX")
        self.Matriz_Coordenadas.addWidget(self.Label_DX, 0, 0, 1, 1)
        self.Label_DY = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.Label_DY.setObjectName("Label_DY")
        self.Matriz_Coordenadas.addWidget(self.Label_DY, 1, 0, 1, 1)
        self.Display_X = QtWidgets.QTextEdit(self.gridLayoutWidget_2)
        self.Display_X.setEnabled(True)
        self.Display_X.setObjectName("Display_X")
        self.Matriz_Coordenadas.addWidget(self.Display_X, 0, 1, 1, 1)
        self.Display_Y = QtWidgets.QTextEdit(self.gridLayoutWidget_2)
        self.Display_Y.setObjectName("Display_Y")
        self.Matriz_Coordenadas.addWidget(self.Display_Y, 1, 1, 1, 1)
        self.Display_Z = QtWidgets.QTextEdit(self.gridLayoutWidget_2)
        self.Display_Z.setObjectName("Display_Z")
        self.Matriz_Coordenadas.addWidget(self.Display_Z, 2, 1, 1, 1)
        self.Boton_Menu = QtWidgets.QPushButton(self.ventada_mapeo)
        self.Boton_Menu.setGeometry(QtCore.QRect(110, 20, 89, 25))
        self.Boton_Menu.setObjectName("Boton_Menu")
        self.menus.addTab(self.ventada_mapeo, "")
        self.Fecha_Hora = QtWidgets.QDateTimeEdit(Interfaz_Tesis)
        self.Fecha_Hora.setGeometry(QtCore.QRect(710, 570, 194, 26))
        self.Fecha_Hora.setObjectName("Fecha_Hora")

        self.retranslateUi(Interfaz_Tesis)
        self.menus.setCurrentIndex(1)
        QtCore.QMetaObject.connectSlotsByName(Interfaz_Tesis)

    def retranslateUi(self, Interfaz_Tesis):
        _translate = QtCore.QCoreApplication.translate
        Interfaz_Tesis.setWindowTitle(_translate("Interfaz_Tesis", "Window_Tesis"))
        self.Label_Opciones.setText(_translate("Interfaz_Tesis", "Opciones de control"))
        self.Boton_Mapeo.setText(_translate("Interfaz_Tesis", "Mapeo"))
        self.Boton_NavReact.setText(_translate("Interfaz_Tesis", "Navegación Reactiva"))
        self.Autores_Tesis.setHtml(_translate("Interfaz_Tesis", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:600;\">Integrantes:</span></p>\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Wagner Cabezas</p>\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">William Juiña</p></body></html>"))
        self.SelloESPE.setHtml(_translate("Interfaz_Tesis", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><img src=\":/imagenes/espe.png\" /></p></body></html>"))
        self.RobotImagen.setHtml(_translate("Interfaz_Tesis", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><img src=\":/imagenes/tesis_render.jpeg\" /></p></body></html>"))
        self.Boton_Arriba.setText(_translate("Interfaz_Tesis", "up"))
        self.Boton_Derecha.setText(_translate("Interfaz_Tesis", "right"))
        self.Boton_Izquierda.setText(_translate("Interfaz_Tesis", "left"))
        self.Boton_Abajo.setText(_translate("Interfaz_Tesis", "down"))
        self.Boton_Antihorario.setText(_translate("Interfaz_Tesis", "Antihorario"))
        self.Boton_Horario.setText(_translate("Interfaz_Tesis", "Horario"))
        self.Label_Avance.setText(_translate("Interfaz_Tesis", "Avance"))
        self.Label_NombreMapa.setText(_translate("Interfaz_Tesis", "Nombre del mapa"))
        self.Boton_Mapa.setText(_translate("Interfaz_Tesis", "Guardar Mapa"))
        self.Label_RZ.setText(_translate("Interfaz_Tesis", "Rotación Z"))
        self.Label_DX.setText(_translate("Interfaz_Tesis", "Coordenada X"))
        self.Label_DY.setText(_translate("Interfaz_Tesis", "Coordenada Y"))
        self.Boton_Menu.setText(_translate("Interfaz_Tesis", "Menu"))
import recursos_rc
