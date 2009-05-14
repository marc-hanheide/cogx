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
  $string = get_include_contents('include/header.php');
  echo $string;
  ?>

  <div class="content">
    <h3>Systems written with CAST</h3><?php

      bib_new_list(); 

            bib_new_section();

            bib_add("cat", "cast-systems");

    //bib_sort_by("year", "desc", "<h3>%s</h3>");
    //      bib_sort_by("month", "desc");

            bib_display_list_table("publication.php?key=%k");
            ?>
  </div>

  <div class="content">
    <h3>Papers on CAST</h3><?php

      bib_new_list(); 

            bib_new_section();

            bib_add("cat", "cast-theory");

    //bib_sort_by("year", "desc", "<h3>%s</h3>");
    //      bib_sort_by("month", "desc");

            bib_display_list_table("publication.php?key=%k");
            ?>
  </div><?php
    $string = get_include_contents('include/footer.php');
  echo $string;

  $string = get_include_contents('include/menu.php');
  echo $string;
  ?>
</body>
</html>
