# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'policy.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


from tt import Ui_Form as graphForm
class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(689, 473)

        self.checkBox = QtWidgets.QCheckBox(Form)
        self.checkBox.setGeometry(QtCore.QRect(70, 380, 467, 17))
        self.checkBox.setObjectName("checkBox")

        self.Proceed = QtWidgets.QPushButton(Form)
        self.Proceed.setGeometry(QtCore.QRect(510, 420, 151, 31))
        self.Proceed.setObjectName("pushButton")

        self.Title1 = QtWidgets.QLabel(Form)
        self.Title1.setGeometry(QtCore.QRect(20, 20, 221, 71))
        self.Title1.setObjectName("label")

        self.Title2 = QtWidgets.QLabel(Form)
        self.Title2.setGeometry(QtCore.QRect(240, 0, 431, 161))
        self.Title2.setFrameShape(QtWidgets.QFrame.Box)
        self.Title2.setTextFormat(QtCore.Qt.RichText)
        self.Title2.setObjectName("label_2")

        self.Policytext1 = QtWidgets.QLabel(Form)
        self.Policytext1.setGeometry(QtCore.QRect(40, 80, 221, 41))
        self.Policytext1.setObjectName("label_3")

        self.graph = QtWidgets.QLabel(Form)
        self.graph.setGeometry(QtCore.QRect(30, 130, 641, 181))
        self.graph.setObjectName("label_4")

        self.Policytext2 = QtWidgets.QLabel(Form)
        self.Policytext2.setGeometry(QtCore.QRect(30, 270, 601, 71))
        self.Policytext2.setObjectName("label_5")

        self.Policytext3 = QtWidgets.QLabel(Form)
        self.Policytext3.setGeometry(QtCore.QRect(30, 350, 551, 16))
        self.Policytext3.setObjectName("label_6")
        
        self.Quit = QtWidgets.QPushButton(Form)
        self.Quit.setGeometry(QtCore.QRect(410, 420, 91, 31))
        self.Quit.setObjectName("pushButton_2")

        self.retranslateUi(Form)
        self.Proceed.clicked.connect(self.forthscr)
        self.Quit.clicked.connect(Form.close) # type: ignore
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.checkBox.setText(_translate("Form", "    I understand the terms and want to get access to the smart helmat graph plotting window"))
        self.Proceed.setText(_translate("Form", "Proceed to the graph"))
        self.Title1.setText(_translate("Form", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600; color:#0000ff;\">Smart Helmet plot system</span></p></body></html>"))
        self.Title2.setText(_translate("Form", "<html><head/><body><p><img src=\":/newPrefix/images.jpg\"/></p></body></html>"))
        self.Policytext1.setText(_translate("Form", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">Terms &amp; Policies</span></p></body></html>"))
        self.graph.setText(_translate("Form", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'MS Shell Dlg 2\'; font-size:8pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1. The device may record your personal impact data during sports activities, the data will be plotted in the form of 2D graph. </p>\n"
"<p style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">The data will be used for impact level analyse. An Excel file will also be produced to help evaluation and might be sent to medical </p>\n"
"<p style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">professionals in the form of email to provie research samples. Please make sure you agree to share the data mentioned above</p>\n"
"<p style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"> for public use.</p></body></html>"))
        self.Policytext2.setText(_translate("Form", "<html><head/><body><p>2. The smart helmet device and plot program are all developed by the University of Surrey third-year lab project students, </p><p>Group M. Please do not directly copy the design without permission or use for commercial circumstances. The whole system is </p><p>still in testing and developing. Group M members reserve the right of final interpretation.</p></body></html>"))
        self.Policytext3.setText(_translate("Form", "<html><head/><body><p>3. By clicking the box below means that you completely understand and agree with the terms mentioned above. </p></body></html>"))
        self.Quit.setText(_translate("Form", "Exit"))
        
    def forthscr(self):
        self.Form = QtWidgets.QWidget()
        self.ui = graphForm()
        self.ui.setupUi(self.Form)
        self.Form.show()
        
import ref

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())


