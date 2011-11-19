function check_all_by_class(class_id) {
    var elem = document.getElementsByClassName(class_id);
    for (var i=0; i<elem.length; i++) {
        elem[i].checked = true;
    }
}

function clear_all_by_class(class_id) {
    var elem = document.getElementsByClassName(class_id);
    for (var i=0; i<elem.length; i++) {
        elem[i].checked=false;
    }
}
