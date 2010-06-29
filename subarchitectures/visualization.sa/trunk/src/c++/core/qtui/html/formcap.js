// Based on:
// http://www.qtcentre.org/threads/31376-QtWebKit-Hijack-post-variables-and-dump-request
// Code based on that written by Tobias Cohen, tobiascohen.com 
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
//var POST = MyQObject.getPost();
//var GET = MyQObject.getGet();
function MyLibSubmit(form_selector) {
    var ob = $(form_selector).serializeObject();
    if ($(form_selector).attr('method').toLowerCase() == 'post') {
	MyQObject.setPost(form_selector, ob);
    } else {
	MyQObject.setGet(form_selector, ob);
    }
    return true;
}
