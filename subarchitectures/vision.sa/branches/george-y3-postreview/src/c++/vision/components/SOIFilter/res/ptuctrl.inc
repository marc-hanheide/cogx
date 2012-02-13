// Generated from ptucontroller.ui
const char res_ptucontroller_ui[] =
   "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
   "<ui version=\"4.0\">\n"
   " <class>PtuController</class>\n"
   " <widget class=\"QWidget\" name=\"PtuController\">\n"
   "  <property name=\"geometry\">\n"
   "   <rect>\n"
   "    <x>0</x>\n"
   "    <y>0</y>\n"
   "    <width>536</width>\n"
   "    <height>443</height>\n"
   "   </rect>\n"
   "  </property>\n"
   "  <property name=\"sizePolicy\">\n"
   "   <sizepolicy hsizetype=\"Minimum\" vsizetype=\"Minimum\">\n"
   "    <horstretch>0</horstretch>\n"
   "    <verstretch>0</verstretch>\n"
   "   </sizepolicy>\n"
   "  </property>\n"
   "  <property name=\"windowTitle\">\n"
   "   <string>PTU Controller</string>\n"
   "  </property>\n"
   "  <widget class=\"QWidget\" name=\"wctrls\" native=\"true\">\n"
   "   <property name=\"geometry\">\n"
   "    <rect>\n"
   "     <x>4</x>\n"
   "     <y>4</y>\n"
   "     <width>331</width>\n"
   "     <height>91</height>\n"
   "    </rect>\n"
   "   </property>\n"
   "   <layout class=\"QFormLayout\" name=\"formLayout_2\">\n"
   "    <item row=\"0\" column=\"0\">\n"
   "     <widget class=\"QLabel\" name=\"label_3\">\n"
   "      <property name=\"text\">\n"
   "       <string>Pan:</string>\n"
   "      </property>\n"
   "      <property name=\"buddy\">\n"
   "       <cstring>sliderPan</cstring>\n"
   "      </property>\n"
   "     </widget>\n"
   "    </item>\n"
   "    <item row=\"0\" column=\"1\">\n"
   "     <layout class=\"QHBoxLayout\" name=\"horizontalLayout\">\n"
   "      <item>\n"
   "       <widget class=\"QSlider\" name=\"sliderPan\">\n"
   "        <property name=\"sizePolicy\">\n"
   "         <sizepolicy hsizetype=\"Expanding\" vsizetype=\"Fixed\">\n"
   "          <horstretch>1</horstretch>\n"
   "          <verstretch>0</verstretch>\n"
   "         </sizepolicy>\n"
   "        </property>\n"
   "        <property name=\"minimum\">\n"
   "         <number>-90</number>\n"
   "        </property>\n"
   "        <property name=\"maximum\">\n"
   "         <number>90</number>\n"
   "        </property>\n"
   "        <property name=\"orientation\">\n"
   "         <enum>Qt::Horizontal</enum>\n"
   "        </property>\n"
   "        <property name=\"invertedAppearance\">\n"
   "         <bool>true</bool>\n"
   "        </property>\n"
   "        <property name=\"invertedControls\">\n"
   "         <bool>false</bool>\n"
   "        </property>\n"
   "       </widget>\n"
   "      </item>\n"
   "      <item>\n"
   "       <widget class=\"QDoubleSpinBox\" name=\"spinPan\">\n"
   "        <property name=\"minimum\">\n"
   "         <double>-90.000000000000000</double>\n"
   "        </property>\n"
   "        <property name=\"maximum\">\n"
   "         <double>90.000000000000000</double>\n"
   "        </property>\n"
   "       </widget>\n"
   "      </item>\n"
   "     </layout>\n"
   "    </item>\n"
   "    <item row=\"1\" column=\"0\">\n"
   "     <widget class=\"QLabel\" name=\"label_2\">\n"
   "      <property name=\"text\">\n"
   "       <string>Tilt:</string>\n"
   "      </property>\n"
   "      <property name=\"buddy\">\n"
   "       <cstring>sliderTilt</cstring>\n"
   "      </property>\n"
   "     </widget>\n"
   "    </item>\n"
   "    <item row=\"1\" column=\"1\">\n"
   "     <layout class=\"QHBoxLayout\" name=\"horizontalLayout_2\">\n"
   "      <item>\n"
   "       <widget class=\"QSlider\" name=\"sliderTilt\">\n"
   "        <property name=\"sizePolicy\">\n"
   "         <sizepolicy hsizetype=\"Expanding\" vsizetype=\"Fixed\">\n"
   "          <horstretch>1</horstretch>\n"
   "          <verstretch>0</verstretch>\n"
   "         </sizepolicy>\n"
   "        </property>\n"
   "        <property name=\"minimum\">\n"
   "         <number>-90</number>\n"
   "        </property>\n"
   "        <property name=\"maximum\">\n"
   "         <number>90</number>\n"
   "        </property>\n"
   "        <property name=\"orientation\">\n"
   "         <enum>Qt::Horizontal</enum>\n"
   "        </property>\n"
   "       </widget>\n"
   "      </item>\n"
   "      <item>\n"
   "       <widget class=\"QDoubleSpinBox\" name=\"spinTilt\">\n"
   "        <property name=\"minimum\">\n"
   "         <double>-90.000000000000000</double>\n"
   "        </property>\n"
   "        <property name=\"maximum\">\n"
   "         <double>90.000000000000000</double>\n"
   "        </property>\n"
   "       </widget>\n"
   "      </item>\n"
   "     </layout>\n"
   "    </item>\n"
   "    <item row=\"2\" column=\"0\">\n"
   "     <widget class=\"QLabel\" name=\"label\">\n"
   "      <property name=\"text\">\n"
   "       <string>Zoom:</string>\n"
   "      </property>\n"
   "      <property name=\"buddy\">\n"
   "       <cstring>sliderZoom</cstring>\n"
   "      </property>\n"
   "     </widget>\n"
   "    </item>\n"
   "    <item row=\"2\" column=\"1\">\n"
   "     <layout class=\"QHBoxLayout\" name=\"horizontalLayout_3\">\n"
   "      <item>\n"
   "       <widget class=\"QSlider\" name=\"sliderZoom\">\n"
   "        <property name=\"minimum\">\n"
   "         <number>-10</number>\n"
   "        </property>\n"
   "        <property name=\"maximum\">\n"
   "         <number>10</number>\n"
   "        </property>\n"
   "        <property name=\"orientation\">\n"
   "         <enum>Qt::Horizontal</enum>\n"
   "        </property>\n"
   "       </widget>\n"
   "      </item>\n"
   "      <item>\n"
   "       <widget class=\"QDoubleSpinBox\" name=\"spinZoom\">\n"
   "        <property name=\"minimum\">\n"
   "         <double>-10.000000000000000</double>\n"
   "        </property>\n"
   "        <property name=\"maximum\">\n"
   "         <double>10.000000000000000</double>\n"
   "        </property>\n"
   "       </widget>\n"
   "      </item>\n"
   "     </layout>\n"
   "    </item>\n"
   "   </layout>\n"
   "  </widget>\n"
   "  <widget class=\"QPushButton\" name=\"btSetPosition\">\n"
   "   <property name=\"geometry\">\n"
   "    <rect>\n"
   "     <x>240</x>\n"
   "     <y>100</y>\n"
   "     <width>90</width>\n"
   "     <height>23</height>\n"
   "    </rect>\n"
   "   </property>\n"
   "   <property name=\"text\">\n"
   "    <string>Set Position</string>\n"
   "   </property>\n"
   "   <property name=\"default\">\n"
   "    <bool>true</bool>\n"
   "   </property>\n"
   "  </widget>\n"
   "  <widget class=\"QPushButton\" name=\"btGetPosition\">\n"
   "   <property name=\"geometry\">\n"
   "    <rect>\n"
   "     <x>4</x>\n"
   "     <y>100</y>\n"
   "     <width>90</width>\n"
   "     <height>23</height>\n"
   "    </rect>\n"
   "   </property>\n"
   "   <property name=\"text\">\n"
   "    <string>Get Position</string>\n"
   "   </property>\n"
   "  </widget>\n"
   " </widget>\n"
   " <tabstops>\n"
   "  <tabstop>sliderPan</tabstop>\n"
   "  <tabstop>spinPan</tabstop>\n"
   "  <tabstop>sliderTilt</tabstop>\n"
   "  <tabstop>spinTilt</tabstop>\n"
   "  <tabstop>sliderZoom</tabstop>\n"
   "  <tabstop>spinZoom</tabstop>\n"
   "  <tabstop>btSetPosition</tabstop>\n"
   " </tabstops>\n"
   " <resources/>\n"
   " <connections/>\n"
   "</ui>\n"
   ; // res_ptucontroller_ui

// Generated from ptucontroller.js
const char res_ptucontroller_js[] =
   "\n"
   "Function.prototype.bind = function() {\n"
   "   var func = this;\n"
   "   var thisObject = arguments[0];\n"
   "   var args = Array.prototype.slice.call(arguments, 1);\n"
   "   return function() {\n"
   "      return func.apply(thisObject, args);\n"
   "   }\n"
   "}\n"
   "\n"
   "\n"
   "function PtuController(ui)\n"
   "{\n"
   "   this.ui = ui;\n"
   "\n"
   "   with (this.ui.wctrls) {\n"
   "\n"
   "      sliderPan.valueChanged.connect(spinPan, spinPan.setValue);\n"
   "      spinPan['valueChanged(double)'].connect(sliderPan, sliderPan.setValue);\n"
   "\n"
   "      sliderTilt.valueChanged.connect(spinTilt, spinTilt.setValue);\n"
   "      spinTilt['valueChanged(double)'].connect(sliderTilt, sliderTilt.setValue);\n"
   "\n"
   "      sliderZoom.valueChanged.connect(spinZoom, spinZoom.setValue);\n"
   "      spinZoom['valueChanged(double)'].connect(sliderZoom, sliderZoom.setValue);\n"
   "   }\n"
   "\n"
   "   with (this.ui) {\n"
   "      btSetPosition.clicked.connect(this, this.onSetPosition_clicked);\n"
   "      btGetPosition.clicked.connect(this, this.onGetPosition_clicked);\n"
   "   }\n"
   "}\n"
   "\n"
   "PtuController.prototype.onSetPosition_clicked = function()\n"
   "{\n"
   "   with (this.ui.wctrls) {\n"
   "      dialogOwner.setValue(\"PTZ\", [ spinPan.value, spinTilt.value, spinZoom.value ]);\n"
   "   }\n"
   "}\n"
   "\n"
   "PtuController.prototype.onGetPosition_clicked = function()\n"
   "{\n"
   "   with (this.ui.wctrls) {\n"
   "      dialogOwner.call(\"sendStateToDialog\", 0);\n"
   "   }\n"
   "}\n"
   ; // res_ptucontroller_js
