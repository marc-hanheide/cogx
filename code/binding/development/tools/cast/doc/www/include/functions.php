<?php

function get_include_contents($filename) {
  if (is_file($filename)) {
    ob_start();
    include $filename;
    $contents = ob_get_contents();
    ob_end_clean();
    return $contents;
  }
  return false;
}



function write_dload_data($name, $email, $affiliation, $filename) {

  // Let's make sure the file exists and is writable first.
  if (is_writable($filename)) {
    
    // In our example we're opening $filename in append mode.
    // The file pointer is at the bottom of the file hence
    // that's where $somecontent will go when we fwrite() it.
    if (!$handle = fopen($filename, 'a')) {
      echo "Cannot open file ($filename)";
      exit;
    }

    // Write $somecontent to our opened file.
    if (fwrite($handle, "CAST: ") === FALSE) {
      echo "Cannot write CAST to file ($filename)";
      exit;
    }

    // Write $somecontent to our opened file.
    if (fwrite($handle, $name) === FALSE) {
      echo "Cannot write name to file ($filename)";
      exit;
    }

    // Write $somecontent to our opened file.
    if (fwrite($handle, " | ") === FALSE) {
      echo "Cannot write divider to file ($filename)";
      exit;
    }

    // Write $somecontent to our opened file.
    if (fwrite($handle, $email) === FALSE) {
      echo "Cannot write name to file ($filename)";
      exit;
    }

    if(isset($affiliation) && !empty($affiliation)) {

      // Write $somecontent to our opened file.
      if (fwrite($handle, " | ") === FALSE) {
	echo "Cannot write divider to file ($filename)";
	exit;
      }
      
      // Write $somecontent to our opened file.
      if (fwrite($handle, $affiliation) === FALSE) {
	echo "Cannot write name to file ($filename)";
      exit;
      }
      
    }
    
    // Write $somecontent to our opened file.
    if (fwrite($handle, "\n") === FALSE) {
      echo "Cannot write divider to file ($filename)";
      exit;
    }


    //echo "Success, wrote download data to file ($filename)";
    
    fclose($handle);
    
  } 
  else {
    echo "The file $filename is not writable";
  }
  
}

?>