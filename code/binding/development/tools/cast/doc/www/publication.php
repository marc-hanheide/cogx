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

  <title>CAST &gt; Publications</title>
<style type="text/css" media="all">
   /*<![CDATA[*/
   @import "css/cosy.css";
   @import "css/bib.css";
/*]]>*/
</style>
</head>

<body>
  <?php require '/home/staff/nah/public_html/bib/bib_stuff.php'; ?><?php
  // get key from post/get
  $key = extract_key_from_post_get();
  if (!$key) return;
  // get item from database
  $item = bib_fetch_item($key);
  if (!$item) return;
  ?><?php
  $string = get_include_contents('include/header.php');
  echo $string;
  ?>

  <div class="content">
    <?php bib_display_item_detailed($item); ?>
  </div><?php
    $string = get_include_contents('include/footer.php');
  echo $string;

  $string = get_include_contents('include/menu.php');
  echo $string;
  ?>
</body>
</html>
