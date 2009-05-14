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

  <title>CAST &gt; Home</title>
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
    <h1>CAST: The CoSy Architecture Schema Toolkit</h1>

    <p>As part of <a href=
    "http://www.cs.bham.ac.uk/research/projects/cosy/">the CoSy
    project</a> we have developed a software toolkit to support the
    developments of intelligent systems based on a space of
    possible architecture designs. We have developed the <em>CoSy
    Architecture Schema</em> (CAS) to describe this space of
    designs. The <em>CoSy Architecture Schema Toolkit</em> (CAST)
    is a software implementation of this schema designed to allow
    researchers (primarily in the fields of AI and robotics) to
    develop <em>instantiations</em> of the schema. The toolkit
    supports C++ and Java, and provides a communication framework
    for distributing an instantiation across a network. It's
    primary scientific purpose is to maintain a separation between
    a system's architecture and the content of it's architecture,
    allowing one to be varied independently of the other.</p>
  </div>

  <div class="content">
    <h2>Authors</h2>

    <p>The development of CAST is currently supported by the
    <a href="http://www.cognitivesystems.org">EU FP6 IST Cognitive
    Systems Integrated Project ``CoSy'' FP6-004250-IP</a>. The
    following researchers have contributed to the development of
    the software:</p>

    <ul>
      <li><a href="http://www.cs.bham.ac.uk/~nah/">Nick
      Hawes</a></li>

      <li><a href="http://www.cs.bham.ac.uk/~mxz/">Michael
      Zillich</a></li>

      <li><a href="http://www.dfki.de/~henrikj/">Henrik
      Jacobsson</a></li>

      <li><a href=
      "http://research.xlab.si/index.php?option=com_content&amp;task=view&amp;id=106&amp;Itemid=141">
      Gregor Berginc</a></li>
    </ul>

    <p>If you have any questions about CAST please contact <a href=
    "mailto:n.a.hawes@cs.bham.ac.uk">Nick Hawes</a>.</p>
  </div><?php $string = get_include_contents('include/footer.php'); echo
        $string;

      $string = get_include_contents('include/menu.php');
      echo $string;
      ?>
</body>
</html>
