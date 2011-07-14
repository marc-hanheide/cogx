
Function.prototype.bind = function() {
   var func = this;
   var thisObject = arguments[0];
   var args = Array.prototype.slice.call(arguments, 1);
   return function() {
      return func.apply(thisObject, args);
   }
}


function GazeboJuggler(ui)
{
   this.ui = ui;
   this.placeCount = 0;
   this.emptyObject = "<empty>";
   this.placeContent = {};

   //with (this.ui.wctrls) {
   //   ////sliderPan.valueChanged.connect(this, this.onSliderPan_valueChanged);
   //   sliderPan.valueChanged.connect(spinPan, spinPan.setValue);
   //   spinPan['valueChanged(double)'].connect(sliderPan, sliderPan.setValue);

   //   sliderTilt.valueChanged.connect(spinTilt, spinTilt.setValue);
   //   spinTilt['valueChanged(double)'].connect(sliderTilt, sliderTilt.setValue);

   //   sliderZoom.valueChanged.connect(spinZoom, spinZoom.setValue);
   //   spinZoom['valueChanged(double)'].connect(sliderZoom, sliderZoom.setValue);
   //}

   //with (this.ui) {
   //   btSetPosition.clicked.connect(this, this.onSetPosition_clicked);
   //   btGetPosition.clicked.connect(this, this.onGetPosition_clicked);
   //}
}

GazeboJuggler.prototype.setPlaceCount = function(n)
{
   var i, num, ctrl, name;
   if (n > 10) n = 10;
   this.placeCount = n;
   for(i = 1; i <= 10; i++) {
      if (i < 10) num = '0' + i.toString();
      else num = i.toString();

      ctrl = this.ui.findChild('labelPlace' + num);
      ctrl.visible = (i <= n);
      name = 'cbxPlace' + num;
      ctrl = this.ui.findChild(name);
      ctrl.visible = (i <= n);
      ctrl['currentIndexChanged(QString)'].connect(this.placeIndexChanged.bind(this, name));
      this.placeContent[name] = ctrl.value;
   }
}

GazeboJuggler.prototype.setObjectNames = function(itemList)
{
   var i, num, ctrl, items, name;
   for(i = 1; i <= this.placeCount; i++) {
      if (i < 10) num = '0' + i.toString();
      else num = i.toString();

      name = 'cbxPlace' + num;
      dialogOwner.setComboBoxItems(name, [this.emptyObject].concat(itemList));

      ctrl = this.ui.findChild(name);
      ctrl.currentIndex = 0;
   }
}

GazeboJuggler.prototype.placeIndexChanged = function(cbxName)
{
   var ctrl = this.ui.findChild(cbxName);
   var wasMoved = false;
   var text = ctrl.currentText;

   if (ctrl.currentIndex != 0) {
      var name, i, numOther, cbOther;

      for(i = 1; i <= this.placeCount; i++) {
         if (i < 10) numOther = '0' + i.toString();
         else numOther = i.toString();
         name = 'cbxPlace' + numOther;
         if (name == cbxName)
            continue;
         cbOther = this.ui.findChild(name);
         if (cbOther.currentText == text) {
            // The object in the scene moves to another location
            cbOther.currentIndex = 0;
            wasMoved = true;
         }
      }
   }
   if (text != this.placeContent[cbxName]) {
      // move the object that was previously on place cbxName to cbxPlace00 (off the scene)
      dialogOwner.setValue('cbxPlace00', this.placeContent[cbxName]);
      dialogOwner.setValue(cbxName, text);
   }
   this.placeContent[cbxName] = text;
}


//PtuController.prototype.onSetPosition_clicked = function()
//{
//   with (this.ui.wctrls) {
//      dialogOwner.setValue("PTZ", [ spinPan.value, spinTilt.value, spinZoom.value ]);
//   }
//}

//PtuController.prototype.onGetPosition_clicked = function()
//{
//   with (this.ui.wctrls) {
//      dialogOwner.call("sendStateToDialog", 0);
//   }
//}
