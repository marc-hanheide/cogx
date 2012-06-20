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

function dump(name, object)
{
   var output = '<h3>Dump of ' + name + '</h3>';
   output += '<ul>'
   for (property in object) {
      output += '<li>' + property + ': ' + object[property]+';';
   }
   output += '</ul>'
   dialogOwner.setHtml('@dump', name, output);
}

function PtuController(ui)
{
   this.ui = ui;
   //dump("PtuController", this);
   //dump("PtuController.ui", this.ui);
   //dump("PtuController.ui.wbts", this.ui.wbts);
   //dump("PtuController.ui.wbts.ckBlockUpdates", this.ui.wbts.ckBlockUpdates);

   with (this.ui.wctrls) {
      sliderPan.valueChanged.connect(spinPan, spinPan.setValue);
      spinPan['valueChanged(double)'].connect(sliderPan, sliderPan.setValue);

      sliderTilt.valueChanged.connect(spinTilt, spinTilt.setValue);
      spinTilt['valueChanged(double)'].connect(sliderTilt, sliderTilt.setValue);

      sliderZoom.valueChanged.connect(spinZoom, spinZoom.setValue);
      spinZoom['valueChanged(double)'].connect(sliderZoom, sliderZoom.setValue);
   }

   with (this.ui.wbts) {
      btSetPosition.clicked.connect(this, this.onSetPosition_clicked);
      btGetPosition.clicked.connect(this, this.onGetPosition_clicked);
   }

}

PtuController.prototype.onSetPosition_clicked = function()
{
   with (this.ui.wctrls) {
      dialogOwner.setValue("PTZ", [ spinPan.value, spinTilt.value, spinZoom.value ]);
      //this.ui.wbts.ckBlockUpdates.checked = false;
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
   if (bForce || ! this.ui.wbts.ckBlockUpdates.checked) {
      this.ui.wctrls.spinPan.value  = fPan;
      this.ui.wctrls.spinTilt.value = fTilt;
      this.ui.wctrls.spinZoom.value = fZoom;
   }
}

PtuController.prototype.setPtzIsMoving = function(bMoving)
{
   if (bMoving) {
      this.ui.wctrls.labStatus.setText("Moving");
   }
   else {
      this.ui.wctrls.labStatus.setText("Still");
   }
}

