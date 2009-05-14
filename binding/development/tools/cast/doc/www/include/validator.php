<?php

class FormValidator {
  
  // Error count
  var $err;
  
  // Messages 
  var $msg;

  function FormValidator() {
    $this->err = 0;
  }

  function ifPost($post) {     
    // If the post is set, echo it 
    echo (isset($post)) ? stripslashes($post) : ""; 
    //echo stripslashes($post); 
    //echo "blah";
  } 
  
  function ifValid() { 
    // Checks to see if form passes 
    return ($this->err == 0) ? true : false; 
  } 
  
  function showErr() { 
    // Shows the amount of errors as a number 
    return $this->err; 
  } 
  
  function showMsg() { 
    // Goes through all messages 
    $message = "";     
    foreach ($this->msg as $value) {       
      $message .= "<li>". $value ."</li>\n"; 
    } 
    return $message; 
    
  } 
  
  function isEmpty($data, $message, $req=true) { 
    // Checks to see if field is empty 
    if ($req || $data) { 
      if (empty($data)) { 	
	$this->err++; 
	$this->msg[] = $message; 
      } else { 
	return $data;  
      } 
    }     
  } 
  
}

?>