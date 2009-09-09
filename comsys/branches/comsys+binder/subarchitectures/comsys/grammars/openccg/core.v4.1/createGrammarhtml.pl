$script = " <head>
	<title>Lexical Families</title>
<script type=\"text/javascript\">

/* CLOSED_IMAGE - the image to be displayed when the sublists are closed
 * OPEN_IMAGE   - the image to be displayed when the sublists are opened
 */
CLOSED_IMAGE='plus.png';
OPEN_IMAGE='minus.png';

/* makeCollapsible - makes a list have collapsible sublists
 * 
 * listElement - the element representing the list to make collapsible
 */
function makeCollapsible(listElement){

  // removed list item bullets and the sapce they occupy
  listElement.style.listStyle='none';
  listElement.style.marginLeft='0';
  listElement.style.paddingLeft='0';

  // loop over all child elements of the list
  var child=listElement.firstChild;
  while (child!=null){

    // only process li elements (and not text elements)
    if (child.nodeType==1){

      // build a list of child ol and ul elements and hide them
      var list=new Array();
      var grandchild=child.firstChild;
      while (grandchild!=null){
        if (grandchild.tagName=='OL' || grandchild.tagName=='UL'){
          grandchild.style.display='none';
          list.push(grandchild);
        }
        grandchild=grandchild.nextSibling;
      }

      // add toggle buttons
      var node=document.createElement('img');
      node.setAttribute('src',CLOSED_IMAGE);
      node.setAttribute('class','collapsibleClosed');
      node.onclick=createToggleFunction(node,list);
      child.insertBefore(node,child.firstChild);

    }

    child=child.nextSibling;
  }

}

/* createToggleFunction - returns a function that toggles the sublist display
 * 
 * toggleElement  - the element representing the toggle gadget
 * sublistElement - an array of elements representing the sublists that should
 *                  be opened or closed when the toggle gadget is clicked
 */
function createToggleFunction(toggleElement,sublistElements){

  return function(){

    // toggle status of toggle gadget
    if (toggleElement.getAttribute('class')=='collapsibleClosed'){
      toggleElement.setAttribute('class','collapsibleOpen');
      toggleElement.setAttribute('src',OPEN_IMAGE);
    }else{
      toggleElement.setAttribute('class','collapsibleClosed');
      toggleElement.setAttribute('src',CLOSED_IMAGE);
    }

    // toggle display of sublists
    for (var i=0;i<sublistElements.length;i++){
      sublistElements[i].style.display=
          (sublistElements[i].style.display=='block')?'none':'block';
    }

  }

}

</script> ";


opendir(DIR, ".") or die;
open(FILE_OUT, '>', "GRAMMAR_CONTENTS.html");

print FILE_OUT "<html>\n", $script;
print FILE_OUT "<body onLoad=\"makeCollapsible(document.getElementById('files'));\">", "\n<ul id=files>";
$start_flag = "yes";

foreach $file (grep /\.xsl$/i, readdir(DIR)) {

    
	
	my $path = "./".$file;
	$family_found = "no";
	$entry_string = "";

	open (FILE, $path);
	
	while ( <FILE> ) {
  	   chomp;
         if(/family name=\"(.+?)\"/ ) {     # If its a start of family line, print family name
		   if($start_flag eq "yes"){$start_flag="no";}  # close previous list of family entries, unless this is the first
		   else{ $entry_string .= "</ul>\n"; }
		 $entry_string .=  "<li><font size=\"4\">". $1 . "\n</font><ul>"; # </li> here works
	     $family_found = "yes";
	   }
	   if($family_found eq "yes" && /nodeset\(\$(.+?)\)/ ){   # If you have found a family, look for syntactic cat
		  $syncat = $1;
		  $whitespace = "";
		  for ($i = 1; $i < 30 - length($syncat); $i++){$whitespace .= "&nbsp"; } ; 
		  $entry_string .= "<li> <font size=\"3\" face=\"courier\"color =\"#FF0000\"> ". $syncat . $whitespace."</font>";
	   }
	   if($family_found eq "yes" && /select=\"\$(.+?)\"/ ){   # then semantic
	 	  $entry_string .= "<font size=\"3\" color =\"#0000FF\">". $1. "</font></li>\n";
	   }		

	}
	
	# handles the last list of family entries by closing it and setting start flag back to yes
    $entry_string .= "</ul>\n";
	$start_flag = "yes";
    
	if ($family_found eq "yes"){
	   print FILE_OUT "<li><font size=\"2\" color =\"#616D7E\">\n", $file, "</font><ul>\n";
	   print FILE_OUT $entry_string;
	   print FILE_OUT "</ul>\n</li>\n";
	}
	
	close (FILE);
}

print FILE_OUT "</ul>\n</body>\n</html>";

closedir (DIR);

#if (/^\s*<!--/)   NOTE: SHOULD HANDLE comment blocks. Must have another marker, inside_comment
