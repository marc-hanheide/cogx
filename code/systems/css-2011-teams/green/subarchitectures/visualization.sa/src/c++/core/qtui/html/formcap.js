// function from: http://www.qtcentre.org/threads/31376-QtWebKit-Hijack-post-variables-and-dump-request
$.fn.serializeObject = function(){
    var object = {};
    var sArray = this.serializeArray();
    $.each(sArray , function() {
        if (object[this.name]) {
            if (!object[this.name].push) {
                object[this.name] = [object[this.name]];
            }
            object[this.name].push(this.value || '');
        } else {
            object[this.name] = this.value || '';
        }
    });
    return object;
};

// function from: http://www.learningjquery.com/2007/08/clearing-form-data
$.fn.clearForm = function() {
    return this.each(function() {
        var type = this.type, tag = this.tagName.toLowerCase();
        if (tag == 'form')
            return $(':input',this).clearForm();
        if (type == 'text' || type == 'password' || tag == 'textarea')
            this.value = '';
        else if (type == 'checkbox' || type == 'radio')
            this.checked = false;
        else if (tag == 'select')
            this.selectedIndex = -1;
        });
};

function CogxJsSubmit(form_selector)
{
    var ob = $(form_selector).serializeObject();
    if ($(form_selector).attr('method').toLowerCase() == 'post') {
        MyQObject.setPost(form_selector, ob);
    } else {
        MyQObject.setGet(form_selector, ob);
    }
    return true;
}

function CogxJsSubmitAndClick(form_selector, clickid)
{
    CogxJsSubmit(form_selector);
    CogxJsOnClick(form_selector, clickid);
}

function CogxJsBoolValue(truelist, name)
{
    var s;
    if (!truelist.push) { // test if it's an array
        return (truelist == name) ? true : false;
    }
    else {
        for (s = 0; s < truelist.length; s++) {
            if (truelist[s] == name) return true;
        }
        return false;
    }
}

function CogxJsOnClick(htmlid, clickid)
{
    MyQObject.onClick(htmlid, clickid);
}

function CogxJsSendValue(form_selector, clickid, valueid)
{
    var ob = $(form_selector).serializeObject();
    MyQObject.onSendValue(form_selector, clickid, valueid, ob);
}

$.fn.fillForm = function(vals) {
    return this.each(function() {
        var type = this.type, tag = this.tagName.toLowerCase(), i;
        if (tag == 'form')
            return $(':input', this).fillForm(vals);

        var safeval = vals[this.name];
        if (! safeval) safeval = "";
        if (type == 'text' || type == 'password' || tag == 'textarea') {
            this.value = safeval;
        }
        else if (type == 'checkbox' || type == 'radio') {
            this.checked = CogxJsBoolValue(safeval, this.value);
        }
        else if (tag == 'select') {
            for(i=0; i<this.options.length; i++) {
               this.options[i].selected = CogxJsBoolValue(safeval, this.options[i].text);
            }
        }
    });
};

function CogxJsFillFormV(form_selector, vals)
{
    //_dumpvals(vals);
    var form = $(form_selector);
    form.clearForm();
    form.fillForm(vals);
}

function CogxJsFillForm(form_selector)
{
    var vals = MyQObject.getValues(form_selector);
    CogxJsFillFormV(form_selector, vals);
}

function CogxJsSave(form_selector)
{
    var ob = $(form_selector).serializeObject();
    MyQObject.saveFormData(form_selector, ob);
}

function CogxJsLoad(form_selector)
{
    var vals = MyQObject.getSavedFormData(form_selector);
    //_dumpvals(vals);
    CogxJsFillFormV(form_selector, vals);
}

function _dumpvals(vals) {
    var dbo = $('#debugout');
    if (! dbo) return;
    dbo.append("<br>" + jQuery.param(vals, true));
}
// vim:sw=4:ts=8:et
