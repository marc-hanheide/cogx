// Author: Marko Mahnič
// Created: 2012-06-15

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

function DialogueInteraction(ui)
{
  var ctrl;
  this.ui = ui;
  this.edHuman = this.ui.findChild('edHuman');
  this.txtHistory = this.ui.findChild('txtHistory');
  this.itemList = new Array();

  //dump('edHuman', this.edHuman);
  //dump('txtHistory', this.txtHistory);

  var cbs = [ 'cbQuestion', 'cbAssertion', 'cbInstruction', 'cbColor', 'cbShape', 'cbType' ];
  //dump('cbs', cbs);
  for ( i = 0; i < cbs.length; i++) {
    name = cbs[i];
    ctrl = this.ui.findChild(name);
    //dump(name, ctrl);
    //ctrl['currentIndexChanged(QString)'].connect(this.stringSelected.bind(this, name));
    ctrl['activated(QString)'].connect(this.stringSelected.bind(this, name));
  }

  var bts = [ 'Yes', 'No', 'The', 'Is', 'Not', 'A', 'It', 'Object' ];
  for ( i = 0; i < bts.length; i++) {
    name = 'bt' + bts[i];
    ctrl = this.ui.findChild(name);
    if (ctrl != null) {
      ctrl['clicked()'].connect(this.wordSelected.bind(this, bts[i].toLowerCase()));
    }
  }
  this.ui.findChild('btSay').clicked.connect(this, this.onSay_clicked);
  this.ui.findChild('btClear').clicked.connect(this, this.onClear_clicked);
}

DialogueInteraction.prototype.stringSelected = function(name)
{
  var cbctrl = this.ui.findChild(name);
  var text = cbctrl.currentText;
  if (text[0] != '<') {
    this.edHuman.setEditText(this.edHuman.currentText + ' ' + text);
  }
}

DialogueInteraction.prototype.wordSelected = function(name)
{
  this.edHuman.setEditText(this.edHuman.currentText + ' ' + name);
}

DialogueInteraction.prototype.onSay_clicked = function()
{
  var text = this.edHuman.currentText;
  if (text != '') {
    //this.edHuman.insertItem(0, text);
    this.itemList = [text].concat(this.itemList);
    if (this.itemList.length > 20) {
      this.itemList.pop();
    }
    dialogOwner.setComboBoxItems('edHuman', this.itemList);
    this.edHuman.clearEditText();
    dialogOwner.setValue("HumanSaid", text);
  }
}

DialogueInteraction.prototype.onClear_clicked = function()
{
  this.edHuman.clearEditText();
}

DialogueInteraction.prototype.addSpokenText = function(text)
{
  this.txtHistory.appendPlainText(text);
}

DialogueInteraction.prototype.clearSpokenText = function()
{
  this.txtHistory.clear();
}

// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et :vim
