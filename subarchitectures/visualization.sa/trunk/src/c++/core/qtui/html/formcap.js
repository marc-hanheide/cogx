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

function CogxJsFillFormV(form_selector, vals)
{
    var form = $(form_selector);
    form.clearForm();
    $.each(vals, function(key, value){
        form.find("[name='" + key + "']").val(value);
        });
}

function CogxJsFillForm(form_selector)
{
    var vals = MyQObject.getValues(form_selector);
    CogxJsFillFormV(form_selector, vals);
}
