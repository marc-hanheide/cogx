<?php
require_once 'include/validator.php';
require_once 'include/functions.php'; 

$validator = new FormValidator();  

if (isset($_REQUEST["submit"])) { 

  // Validation Checking Goes Here, Continue Below 
  
  $validator->isEmpty($_REQUEST["name"], 'Please enter your name', true); 
  $validator->isEmpty($_REQUEST["email"], 'Please enter your email address', true); 
  $validator->isEmpty($_REQUEST["affiliation"], 'Please enter your affiliation', false); 
  
  // If no errors then passes 
  if ($validator->ifValid()) { 
      $passed = "This form has passed!"; 
  } 
  // If errors then show error messages 
  else { 
    $errors = $validator->showMsg(); 
  } 
  
  
  // If it passes, display message 
  if (isset($passed)) { 
    // echo $passed;   
    //write details to file
    write_dload_data($_REQUEST["name"], 
		     $_REQUEST["email"], 
		     $_REQUEST["affiliation"], 
		     //'downloads.txt');
		     //'/export/htdocs/website/research/projects/cosy/cast/downloads.txt');
		     '/home/staff/nah/public_html/bib-db/errorlog');
    
    //forward to download page
    header( 'Location: http://sourceforge.net/project/showfiles.php?group_id=199480' ) ;
    
    
  } 
  
 }
?> 

<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Strict//EN"
    "http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd">

<html xmlns="http://www.w3.org/1999/xhtml">


<head>
  <? require_once("include/functions.php") ?>
  <meta name="generator" content=
  "HTML Tidy for Linux/x86 (vers 1st March 2005), see www.w3.org" />
  <meta http-equiv="content-type" content=
  "text/html; charset=us-ascii" />
  <link rel="icon" href="images/icon.png" type="image/png" />

  <title>CAST &gt; Download</title>
<style type="text/css" media="all">
   /*<![CDATA[*/
   @import "css/cosy.css";
/*]]>*/
</style>
</head>

<body>
  <?php
      $string = get_include_contents('include/header.php');
      echo $string;
      ?>

  <div class="content">
    <h1>Download CAST</h1>

  <p>To allow us to provide support and contact you with news about
  updates to CAST please complete the following form before
  downloading.</p>


  
<?php
    // If there are errors, display them 
  if (isset($errors)) {
    echo "<p>There are <b>";
    echo $validator->showErr();
    echo "</b> error(s)"; 
    echo "<ul> ";
    echo $errors;
    echo "</ul></p> "; 
  }

?>

<form action="<?php $_SERVER['PHP_SELF']; ?>" method="post" id="form"> 
	
	<table>
<tr>  
<td>    
<label for="name"><b>Name: </b></label> 
</td>
<td>
<?php
echo "<input type=\"text\" id=\"name\" name=\"name\" value=\"";
if (isset($_REQUEST["submit"])) { 
  echo $validator->ifPost($_REQUEST["name"]);
 }
echo "\" />";
?>
</td>
</tr>

<tr>  
<td>    
<label for="email"><b>Email: </b></label> 
</td>
<td>
<?php
echo "<input type=\"text\" id=\"email\" name=\"email\" value=\"";
if (isset($_REQUEST["submit"])) { 
  echo $validator->ifPost($_REQUEST["email"]);
 }
echo "\" />";
?>
</td>
</tr>


<tr>  
<td>    
<label for="affiliation"><b>Affiliation: </b></label> 
</td>
<td>
<?php
echo "<input type=\"text\" id=\"affiliation\" name=\"affiliation\" value=\"";
if (isset($_REQUEST["submit"])) { 
  echo $validator->ifPost($_REQUEST["affiliation"]);
 }
echo "\" />";
?>
</td>
</tr>
  

<tr>
<td>
<input type="submit" id="submit" name="submit" value="Submit" /> 
  </td>
</tr>
</table>
</form> 
  

							
</div>
  




  <?php $string = get_include_contents('include/footer.php'); echo
        $string;

      $string = get_include_contents('include/menu.php');
      echo $string;
      ?>
</body>
</html>
