// @author: Marko Mahnič
// @created: July 2011
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
      //this.ui.ckBlockUpdates.checked = false;
   }
}

PtuController.prototype.onGetPosition_clicked = function()
{
   with (this.ui.wctrls) {
      dialogOwner.call("sendStateToDialog", 0);
   }
}

PtuController.prototype.setPtzPosition = function(fPan, fTilt, fZoom, bForce)
{
   if (bForce || ! this.ui.ckBlockUpdates.checked) {
      this.ui.wctrls.spinPan.value  = fPan;
      this.ui.wctrls.spinTilt.value = fTilt;
      this.ui.wctrls.spinZoom.value = fZoom;
   }
}
