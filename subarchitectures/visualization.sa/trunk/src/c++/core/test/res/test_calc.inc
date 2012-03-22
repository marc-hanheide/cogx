// C++11 include file
// --------------------------------------------------

// Generated from calculator.ui
const char res_calculator_ui[] = R"((<ui version="4.0" >
 <class>Calculator</class>
 <widget class="QWidget" name="Calculator" >
  <property name="geometry" >
   <rect>
    <x>0</x>
    <y>0</y>
    <width>314</width>
    <height>301</height>
   </rect>
  </property>
  <property name="sizePolicy" >
   <sizepolicy vsizetype="Fixed" hsizetype="Fixed" >
    <horstretch>0</horstretch>
    <verstretch>0</verstretch>
   </sizepolicy>
  </property>
  <property name="minimumSize" >
   <size>
    <width>314</width>
    <height>301</height>
   </size>
  </property>
  <property name="maximumSize" >
   <size>
    <width>314</width>
    <height>301</height>
   </size>
  </property>
  <property name="windowTitle" >
   <string>Calculator</string>
  </property>
  <widget class="QToolButton" name="backspaceButton" >
   <property name="geometry" >
    <rect>
     <x>10</x>
     <y>50</y>
     <width>91</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>Backspace</string>
   </property>
  </widget>
  <widget class="QToolButton" name="clearButton" >
   <property name="geometry" >
    <rect>
     <x>110</x>
     <y>50</y>
     <width>91</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>Clear</string>
   </property>
  </widget>
  <widget class="QToolButton" name="clearAllButton" >
   <property name="geometry" >
    <rect>
     <x>210</x>
     <y>50</y>
     <width>91</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>Clear All</string>
   </property>
  </widget>
  <widget class="QToolButton" name="clearMemoryButton" >
   <property name="geometry" >
    <rect>
     <x>10</x>
     <y>100</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>MC</string>
   </property>
  </widget>
  <widget class="QToolButton" name="readMemoryButton" >
   <property name="geometry" >
    <rect>
     <x>10</x>
     <y>150</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>MR</string>
   </property>
  </widget>
  <widget class="QToolButton" name="setMemoryButton" >
   <property name="geometry" >
    <rect>
     <x>10</x>
     <y>200</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>MS</string>
   </property>
  </widget>
  <widget class="QToolButton" name="addToMemoryButton" >
   <property name="geometry" >
    <rect>
     <x>10</x>
     <y>250</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>M+</string>
   </property>
  </widget>
  <widget class="QToolButton" name="sevenButton" >
   <property name="geometry" >
    <rect>
     <x>60</x>
     <y>100</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>7</string>
   </property>
  </widget>
  <widget class="QToolButton" name="eightButton" >
   <property name="geometry" >
    <rect>
     <x>110</x>
     <y>100</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>8</string>
   </property>
  </widget>
  <widget class="QToolButton" name="nineButton" >
   <property name="geometry" >
    <rect>
     <x>160</x>
     <y>100</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>9</string>
   </property>
  </widget>
  <widget class="QToolButton" name="fourButton" >
   <property name="geometry" >
    <rect>
     <x>60</x>
     <y>150</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>4</string>
   </property>
  </widget>
  <widget class="QToolButton" name="fiveButton" >
   <property name="geometry" >
    <rect>
     <x>110</x>
     <y>150</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>5</string>
   </property>
  </widget>
  <widget class="QToolButton" name="sixButton" >
   <property name="geometry" >
    <rect>
     <x>160</x>
     <y>150</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>6</string>
   </property>
  </widget>
  <widget class="QToolButton" name="oneButton" >
   <property name="geometry" >
    <rect>
     <x>60</x>
     <y>200</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>1</string>
   </property>
  </widget>
  <widget class="QToolButton" name="twoButton" >
   <property name="geometry" >
    <rect>
     <x>110</x>
     <y>200</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>2</string>
   </property>
  </widget>
  <widget class="QToolButton" name="threeButton" >
   <property name="geometry" >
    <rect>
     <x>160</x>
     <y>200</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>3</string>
   </property>
  </widget>
  <widget class="QToolButton" name="zeroButton" >
   <property name="geometry" >
    <rect>
     <x>60</x>
     <y>250</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>0</string>
   </property>
  </widget>
  <widget class="QToolButton" name="pointButton" >
   <property name="geometry" >
    <rect>
     <x>110</x>
     <y>250</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>.</string>
   </property>
  </widget>
  <widget class="QToolButton" name="changeSignButton" >
   <property name="geometry" >
    <rect>
     <x>160</x>
     <y>250</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>+-</string>
   </property>
  </widget>
  <widget class="QToolButton" name="plusButton" >
   <property name="geometry" >
    <rect>
     <x>210</x>
     <y>250</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>+</string>
   </property>
  </widget>
  <widget class="QToolButton" name="divisionButton" >
   <property name="geometry" >
    <rect>
     <x>210</x>
     <y>100</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>/</string>
   </property>
  </widget>
  <widget class="QToolButton" name="timesButton" >
   <property name="geometry" >
    <rect>
     <x>210</x>
     <y>150</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>*</string>
   </property>
  </widget>
  <widget class="QToolButton" name="minusButton" >
   <property name="geometry" >
    <rect>
     <x>210</x>
     <y>200</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>-</string>
   </property>
  </widget>
  <widget class="QToolButton" name="squareRootButton" >
   <property name="geometry" >
    <rect>
     <x>260</x>
     <y>100</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>Sqrt</string>
   </property>
  </widget>
  <widget class="QToolButton" name="powerButton" >
   <property name="geometry" >
    <rect>
     <x>260</x>
     <y>150</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>x^2</string>
   </property>
  </widget>
  <widget class="QToolButton" name="reciprocalButton" >
   <property name="geometry" >
    <rect>
     <x>260</x>
     <y>200</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>1/x</string>
   </property>
  </widget>
  <widget class="QToolButton" name="equalButton" >
   <property name="geometry" >
    <rect>
     <x>260</x>
     <y>250</y>
     <width>41</width>
     <height>41</height>
    </rect>
   </property>
   <property name="text" >
    <string>=</string>
   </property>
  </widget>
  <widget class="QLineEdit" name="display" >
   <property name="geometry" >
    <rect>
     <x>10</x>
     <y>10</y>
     <width>291</width>
     <height>31</height>
    </rect>
   </property>
   <property name="maxLength" >
    <number>15</number>
   </property>
   <property name="alignment" >
    <set>Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter</set>
   </property>
   <property name="readOnly" >
    <bool>true</bool>
   </property>
  </widget>
 </widget>
 <resources/>
 <connections/>
</ui>
))"; // res_calculator_ui
// --------------------------------------------------

// Generated from calculator.js
const char res_calculator_js[] = R"((
Function.prototype.bind = function() {
   var func = this;
   var thisObject = arguments[0];
   var args = Array.prototype.slice.call(arguments, 1);
   return function() {
      return func.apply(thisObject, args);
   }
}

function Calculator(ui)
{
   this.ui = ui;

   this.pendingAdditiveOperator = Calculator.NO_OPERATOR;
   this.pendingMultiplicativeOperator = Calculator.NO_OPERATOR;
   this.sumInMemory = 0;
   this.sumSoFar = 0;
   this.factorSoFar = 0;
   this.waitingForOperand = true;

   with (ui) {
      display.text = "0";

      zeroButton.clicked.connect(this.digitClicked.bind(this, 0));
      oneButton.clicked.connect(this.digitClicked.bind(this, 1));
      twoButton.clicked.connect(this.digitClicked.bind(this, 2));
      threeButton.clicked.connect(this.digitClicked.bind(this, 3));
      fourButton.clicked.connect(this.digitClicked.bind(this, 4));
      fiveButton.clicked.connect(this.digitClicked.bind(this, 5));
      sixButton.clicked.connect(this.digitClicked.bind(this, 6));
      sevenButton.clicked.connect(this.digitClicked.bind(this, 7));
      eightButton.clicked.connect(this.digitClicked.bind(this, 8));
      nineButton.clicked.connect(this.digitClicked.bind(this, 9));

      pointButton.clicked.connect(this, "pointClicked");
      changeSignButton.clicked.connect(this, "changeSignClicked");

      backspaceButton.clicked.connect(this, "backspaceClicked");
      clearButton.clicked.connect(this, "clear");
      clearAllButton.clicked.connect(this, "clearAll");

      clearMemoryButton.clicked.connect(this, "clearMemory");
      readMemoryButton.clicked.connect(this, "readMemory");
      setMemoryButton.clicked.connect(this, "setMemory");
      addToMemoryButton.clicked.connect(this, "addToMemory");

      divisionButton.clicked.connect(this.multiplicativeOperatorClicked.bind(this, Calculator.DIVISION_OPERATOR));
      timesButton.clicked.connect(this.multiplicativeOperatorClicked.bind(this, Calculator.TIMES_OPERATOR));
      minusButton.clicked.connect(this.additiveOperatorClicked.bind(this, Calculator.MINUS_OPERATOR));
      plusButton.clicked.connect(this.additiveOperatorClicked.bind(this, Calculator.PLUS_OPERATOR));

      squareRootButton.clicked.connect(this.unaryOperatorClicked.bind(this, Calculator.SQUARE_OPERATOR));
      powerButton.clicked.connect(this.unaryOperatorClicked.bind(this, Calculator.POWER_OPERATOR));
      reciprocalButton.clicked.connect(this.unaryOperatorClicked.bind(this, Calculator.RECIPROCAL_OPERATOR));
      equalButton.clicked.connect(this, "equalClicked");
   }
}

Calculator.NO_OPERATOR = 0;
Calculator.SQUARE_OPERATOR = 1;
Calculator.POWER_OPERATOR = 2;
Calculator.RECIPROCAL_OPERATOR = 3;
Calculator.DIVISION_OPERATOR = 4;
Calculator.TIMES_OPERATOR = 5;
Calculator.MINUS_OPERATOR = 6;
Calculator.PLUS_OPERATOR = 7;

Calculator.prototype.abortOperation = function()
{
   this.clearAll();
   this.ui.display.text = "####";
}

Calculator.prototype.calculate = function(rightOperand, pendingOperator)
{
   if (pendingOperator == Calculator.PLUS_OPERATOR) {
      this.sumSoFar += rightOperand;
   } else if (pendingOperator == Calculator.MINUS_OPERATOR) {
      this.sumSoFar -= rightOperand;
   } else if (pendingOperator == Calculator.TIMES_OPERATOR) {
      this.factorSoFar *= rightOperand;
   } else if (pendingOperator == Calculator.DIVISION_OPERATOR) {
      if (rightOperand == 0)
         return false;
      this.factorSoFar /= rightOperand;
   }
   return true;
}

Calculator.prototype.digitClicked = function(digitValue)
{
   if ((digitValue == 0) && (this.ui.display.text == "0"))
      return;
   if (this.waitingForOperand) {
      this.ui.display.clear();
      this.waitingForOperand = false;
   }
   this.ui.display.text += digitValue;
}

Calculator.prototype.unaryOperatorClicked = function(op)
{
   var operand = this.ui.display.text - 0;
   var result = 0;
   if (op == Calculator.SQUARE_OPERATOR) {
      if (operand < 0) {
         this.abortOperation();
         return;
      }
      result = Math.sqrt(operand);
   } else if (op == Calculator.POWER_OPERATOR) {
      result = Math.pow(operand, 2);
   } else if (op == Calculator.RECIPROCAL_OPERATOR) {
      if (operand == 0.0) {
         this.abortOperation();
         return;
      }
      result = 1 / operand;
   }
   this.ui.display.text = result + "";
   this.waitingForOperand = true;
}

Calculator.prototype.additiveOperatorClicked = function(op)
{
   var operand = this.ui.display.text - 0;

   if (this.pendingMultiplicativeOperator != Calculator.NO_OPERATOR) {
      if (!this.calculate(operand, this.pendingMultiplicativeOperator)) {
         this.abortOperation();
         return;
      }
      this.ui.display.text = this.factorSoFar + "";
      operand = this.factorSoFar;
      this.factorSoFar = 0;
      this.pendingMultiplicativeOperator = Calculator.NO_OPERATOR;
   }

   if (this.pendingAdditiveOperator != Calculator.NO_OPERATOR) {
      if (!this.calculate(operand, this.pendingAdditiveOperator)) {
         this.abortOperation();
         return;
      }
      this.ui.display.text = this.sumSoFar + "";
   } else {
      this.sumSoFar = operand;
   }

   this.pendingAdditiveOperator = op;
   this.waitingForOperand = true;
}

Calculator.prototype.multiplicativeOperatorClicked = function(op)
{
   var operand = this.ui.display.text - 0;

   if (this.pendingMultiplicativeOperator != Calculator.NO_OPERATOR) {
      if (!this.calculate(operand, this.pendingMultiplicativeOperator)) {
         this.abortOperation();
         return;
      }
      this.ui.display.text = this.factorSoFar + "";
   } else {
      this.factorSoFar = operand;
   }

   this.pendingMultiplicativeOperator = op;
   this.waitingForOperand = true;
}

Calculator.prototype.equalClicked = function()
{
   var operand = this.ui.display.text - 0;

   if (this.pendingMultiplicativeOperator != Calculator.NO_OPERATOR) {
      if (!this.calculate(operand, this.pendingMultiplicativeOperator)) {
         this.abortOperation();
         return;
      }
      operand = this.factorSoFar;
      this.factorSoFar = 0.0;
      this.pendingMultiplicativeOperator = Calculator.NO_OPERATOR;
   }
   if (this.pendingAdditiveOperator != Calculator.NO_OPERATOR) {
      if (!this.calculate(operand, this.pendingAdditiveOperator)) {
         this.abortOperation();
         return;
      }
      this.pendingAdditiveOperator = Calculator.NO_OPERATOR;
   } else {
      this.sumSoFar = operand;
   }

   this.ui.display.text = this.sumSoFar + "";
   this.sumSoFar = 0.0;
   this.waitingForOperand = true;

   dialogOwner.setValue("display.text", this.ui.display.text);
}

Calculator.prototype.pointClicked = function()
{
   if (this.waitingForOperand)
      this.ui.display.text = "0";
   if (this.ui.display.text.indexOf(".") == -1)
      this.ui.display.text += ".";
   this.waitingForOperand = false;
}

Calculator.prototype.changeSignClicked = function()
{
   var text = this.ui.display.text;
   var value = text - 0;

   if (value > 0) {
      text = "-" + text;
   } else if (value < 0) {
      text = text.slice(1);
   }
   this.ui.display.text = text;
}

Calculator.prototype.backspaceClicked = function()
{
   if (this.waitingForOperand)
      return;

   var text = this.ui.display.text;
   text = text.slice(0, -1);
   if (text.length == 0) {
      text = "0";
      this.waitingForOperand = true;
   }
   this.ui.display.text = text;
}

Calculator.prototype.clear = function()
{
   if (this.waitingForOperand)
      return;

   this.ui.display.text = "0";
   this.waitingForOperand = true;
}

Calculator.prototype.clearAll = function()
{
   this.sumSoFar = 0.0;
   this.factorSoFar = 0.0;
   this.pendingAdditiveOperator = Calculator.NO_OPERATOR;
   this.pendingMultiplicativeOperator = Calculator.NO_OPERATOR;
   this.ui.display.text = "0";
   this.waitingForOperand = true;
}

Calculator.prototype.clearMemory = function()
{
   this.sumInMemory = 0.0;
}

Calculator.prototype.readMemory = function()
{
   this.ui.display.text = this.sumInMemory + "";
   this.waitingForOperand = true;
}

Calculator.prototype.setMemory = function()
{
   this.equalClicked();
   this.sumInMemory = this.ui.display.text - 0;
}

Calculator.prototype.addToMemory = function()
{
   this.equalClicked();
   this.sumInMemory += this.ui.display.text - 0;
}
))"; // res_calculator_js
// --------------------------------------------------

// Generated from chat.ui
const char res_chat_ui[] = R"((<?xml version="1.0" encoding="UTF-8"?>
<ui version="4.0">
 <class>Form</class>
 <widget class="QWidget" name="Form">
  <property name="geometry">
   <rect>
    <x>0</x>
    <y>0</y>
    <width>400</width>
    <height>300</height>
   </rect>
  </property>
  <property name="windowTitle">
   <string>Form</string>
  </property>
  <layout class="QVBoxLayout" name="verticalLayout">
   <item>
    <layout class="QHBoxLayout" name="horizontalLayout">
     <item>
      <widget class="QComboBox" name="comboBox">
       <property name="sizePolicy">
        <sizepolicy hsizetype="Expanding" vsizetype="Fixed">
         <horstretch>1</horstretch>
         <verstretch>1</verstretch>
        </sizepolicy>
       </property>
       <item>
        <property name="text">
         <string>red</string>
        </property>
       </item>
       <item>
        <property name="text">
         <string>green</string>
        </property>
       </item>
       <item>
        <property name="text">
         <string>blue</string>
        </property>
       </item>
      </widget>
     </item>
     <item>
      <widget class="QComboBox" name="comboBox_2">
       <property name="sizePolicy">
        <sizepolicy hsizetype="Expanding" vsizetype="Fixed">
         <horstretch>1</horstretch>
         <verstretch>1</verstretch>
        </sizepolicy>
       </property>
       <item>
        <property name="text">
         <string>compact</string>
        </property>
       </item>
       <item>
        <property name="text">
         <string>elongated</string>
        </property>
       </item>
      </widget>
     </item>
     <item>
      <widget class="QComboBox" name="comboBox_3">
       <property name="sizePolicy">
        <sizepolicy hsizetype="MinimumExpanding" vsizetype="Fixed">
         <horstretch>1</horstretch>
         <verstretch>0</verstretch>
        </sizepolicy>
       </property>
       <item>
        <property name="text">
         <string>teabox</string>
        </property>
       </item>
      </widget>
     </item>
    </layout>
   </item>
   <item>
    <layout class="QHBoxLayout" name="horizontalLayout_2">
     <item>
      <widget class="QLineEdit" name="lineEdit">
       <property name="text">
        <string>What shape is this</string>
       </property>
      </widget>
     </item>
     <item>
      <widget class="QPushButton" name="pushButton">
       <property name="text">
        <string>Say</string>
       </property>
      </widget>
     </item>
    </layout>
   </item>
   <item>
    <widget class="QTextEdit" name="textEdit">
     <property name="html">
      <string>&lt;!DOCTYPE HTML PUBLIC &quot;-//W3C//DTD HTML 4.0//EN&quot; &quot;http://www.w3.org/TR/REC-html40/strict.dtd&quot;&gt;
&lt;html&gt;&lt;head&gt;&lt;meta name=&quot;qrichtext&quot; content=&quot;1&quot; /&gt;&lt;style type=&quot;text/css&quot;&gt;
p, li { white-space: pre-wrap; }
&lt;/style&gt;&lt;/head&gt;&lt;body style=&quot; font-family:'Liberation Sans'; font-size:9pt; font-weight:400; font-style:normal;&quot;&gt;
&lt;p style=&quot;-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;&quot;&gt;&lt;/p&gt;&lt;/body&gt;&lt;/html&gt;</string>
     </property>
    </widget>
   </item>
  </layout>
 </widget>
 <resources/>
 <connections/>
</ui>
))"; // res_chat_ui
// --------------------------------------------------

