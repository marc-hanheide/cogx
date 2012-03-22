// C++11 include file
// --------------------------------------------------

// Generated from ptucontroller.ui
const char res_ptucontroller_ui[] = R"((<?xml version="1.0" encoding="UTF-8"?>
<ui version="4.0">
 <class>PtuController</class>
 <widget class="QWidget" name="PtuController">
  <property name="geometry">
   <rect>
    <x>0</x>
    <y>0</y>
    <width>536</width>
    <height>443</height>
   </rect>
  </property>
  <property name="sizePolicy">
   <sizepolicy hsizetype="Minimum" vsizetype="Minimum">
    <horstretch>0</horstretch>
    <verstretch>0</verstretch>
   </sizepolicy>
  </property>
  <property name="windowTitle">
   <string>PTU Controller</string>
  </property>
  <widget class="QWidget" name="wctrls" native="true">
   <property name="geometry">
    <rect>
     <x>4</x>
     <y>4</y>
     <width>331</width>
     <height>91</height>
    </rect>
   </property>
   <layout class="QFormLayout" name="formLayout_2">
    <item row="0" column="0">
     <widget class="QLabel" name="label_3">
      <property name="text">
       <string>Pan:</string>
      </property>
      <property name="buddy">
       <cstring>sliderPan</cstring>
      </property>
     </widget>
    </item>
    <item row="0" column="1">
     <layout class="QHBoxLayout" name="horizontalLayout">
      <item>
       <widget class="QSlider" name="sliderPan">
        <property name="sizePolicy">
         <sizepolicy hsizetype="Expanding" vsizetype="Fixed">
          <horstretch>1</horstretch>
          <verstretch>0</verstretch>
         </sizepolicy>
        </property>
        <property name="minimum">
         <number>-90</number>
        </property>
        <property name="maximum">
         <number>90</number>
        </property>
        <property name="orientation">
         <enum>Qt::Horizontal</enum>
        </property>
        <property name="invertedAppearance">
         <bool>true</bool>
        </property>
        <property name="invertedControls">
         <bool>false</bool>
        </property>
       </widget>
      </item>
      <item>
       <widget class="QDoubleSpinBox" name="spinPan">
        <property name="minimum">
         <double>-90.000000000000000</double>
        </property>
        <property name="maximum">
         <double>90.000000000000000</double>
        </property>
       </widget>
      </item>
     </layout>
    </item>
    <item row="1" column="0">
     <widget class="QLabel" name="label_2">
      <property name="text">
       <string>Tilt:</string>
      </property>
      <property name="buddy">
       <cstring>sliderTilt</cstring>
      </property>
     </widget>
    </item>
    <item row="1" column="1">
     <layout class="QHBoxLayout" name="horizontalLayout_2">
      <item>
       <widget class="QSlider" name="sliderTilt">
        <property name="sizePolicy">
         <sizepolicy hsizetype="Expanding" vsizetype="Fixed">
          <horstretch>1</horstretch>
          <verstretch>0</verstretch>
         </sizepolicy>
        </property>
        <property name="minimum">
         <number>-90</number>
        </property>
        <property name="maximum">
         <number>90</number>
        </property>
        <property name="orientation">
         <enum>Qt::Horizontal</enum>
        </property>
       </widget>
      </item>
      <item>
       <widget class="QDoubleSpinBox" name="spinTilt">
        <property name="minimum">
         <double>-90.000000000000000</double>
        </property>
        <property name="maximum">
         <double>90.000000000000000</double>
        </property>
       </widget>
      </item>
     </layout>
    </item>
    <item row="2" column="0">
     <widget class="QLabel" name="label">
      <property name="text">
       <string>Zoom:</string>
      </property>
      <property name="buddy">
       <cstring>sliderZoom</cstring>
      </property>
     </widget>
    </item>
    <item row="2" column="1">
     <layout class="QHBoxLayout" name="horizontalLayout_3">
      <item>
       <widget class="QSlider" name="sliderZoom">
        <property name="minimum">
         <number>-10</number>
        </property>
        <property name="maximum">
         <number>10</number>
        </property>
        <property name="orientation">
         <enum>Qt::Horizontal</enum>
        </property>
       </widget>
      </item>
      <item>
       <widget class="QDoubleSpinBox" name="spinZoom">
        <property name="minimum">
         <double>-10.000000000000000</double>
        </property>
        <property name="maximum">
         <double>10.000000000000000</double>
        </property>
       </widget>
      </item>
     </layout>
    </item>
   </layout>
  </widget>
  <widget class="QPushButton" name="btSetPosition">
   <property name="geometry">
    <rect>
     <x>240</x>
     <y>100</y>
     <width>90</width>
     <height>23</height>
    </rect>
   </property>
   <property name="text">
    <string>Set Position</string>
   </property>
   <property name="default">
    <bool>true</bool>
   </property>
  </widget>
  <widget class="QPushButton" name="btGetPosition">
   <property name="geometry">
    <rect>
     <x>4</x>
     <y>100</y>
     <width>90</width>
     <height>23</height>
    </rect>
   </property>
   <property name="text">
    <string>Get Position</string>
   </property>
  </widget>
 </widget>
 <tabstops>
  <tabstop>sliderPan</tabstop>
  <tabstop>spinPan</tabstop>
  <tabstop>sliderTilt</tabstop>
  <tabstop>spinTilt</tabstop>
  <tabstop>sliderZoom</tabstop>
  <tabstop>spinZoom</tabstop>
  <tabstop>btSetPosition</tabstop>
 </tabstops>
 <resources/>
 <connections/>
</ui>
))"; // res_ptucontroller_ui
// --------------------------------------------------

// Generated from ptucontroller.js
const char res_ptucontroller_js[] = R"((
Function.prototype.bind = function() {
   var func = this;
   var thisObject = arguments[0];
   var args = Array.prototype.slice.call(arguments, 1);
   return function() {
      return func.apply(thisObject, args);
   }
}


function PtuController(ui)
{
   this.ui = ui;

   with (this.ui.wctrls) {

      sliderPan.valueChanged.connect(spinPan, spinPan.setValue);
      spinPan['valueChanged(double)'].connect(sliderPan, sliderPan.setValue);

      sliderTilt.valueChanged.connect(spinTilt, spinTilt.setValue);
      spinTilt['valueChanged(double)'].connect(sliderTilt, sliderTilt.setValue);

      sliderZoom.valueChanged.connect(spinZoom, spinZoom.setValue);
      spinZoom['valueChanged(double)'].connect(sliderZoom, sliderZoom.setValue);
   }

   with (this.ui) {
      btSetPosition.clicked.connect(this, this.onSetPosition_clicked);
      btGetPosition.clicked.connect(this, this.onGetPosition_clicked);
   }
}

PtuController.prototype.onSetPosition_clicked = function()
{
   with (this.ui.wctrls) {
      dialogOwner.setValue("PTZ", [ spinPan.value, spinTilt.value, spinZoom.value ]);
   }
}

PtuController.prototype.onGetPosition_clicked = function()
{
   with (this.ui.wctrls) {
      dialogOwner.call("sendStateToDialog", 0);
   }
}
))"; // res_ptucontroller_js
// --------------------------------------------------

