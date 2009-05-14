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

  <title>CAST &gt; GUI</title>
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
    <h2>The CAST GUI</h2>

    <p>The GUI is currently a work in progress.</p>

    <p>This document describes the graphical user interface (GUI)
    for CAST and how you prepare components to make use of it.</p>

    <p>The basic ideas behind the user interface are:</p>

    <ul>
      <li>The GUI introspects the components. I.e. the GUI pulls
      information from the components.</li>

      <li>Each component can offer a 2D view, a 3D view and a text
      view of its contents.</li>

      <li>The user can choose which components to view.</li>
    </ul>

    <p>The first point means that the GUI is not intended to
    visualise whatever happens during processing, e.g. now we
    display extracted SIFT features, now we paint those in red
    which are outliers etc. But the GUI is meant to visualise the
    current state (content) of a component. This follows the
    model-view-controller user interface paradigm (where actually
    control is missing in our case). Moreover it is a rather
    pragmatic decision as it greatly simplifies the communication
    patterns between GUI and components. (If you do want to
    visualise stages of processing, make them part of the state of
    your component).</p>

    <p>The second point should give the component programmer enough
    flexibility to display whatever information might be of
    interest for debugging etc.</p>
  </div>

  <div class="content">
    <h2>Screenshots</h2>

    <p>First off, a few screenshots to give you an idea what this
    is all about. On the left side you have a list for selecting
    which component should be displayed. At the bottom you can
    choose between 2D view, 3D view or text view.</p>

    <div class="figure">
      <p><img alt="2d view screenshot" src="images/2d.png" /></p>

      <p>2D GUI View</p>
    </div>

    <div class="figure">
      <p><img alt="3d view screenshot" src="images/3d.png" /></p>

      <p>3D GUI View</p>
    </div>

    <div class="figure">
      <p><img alt="text view screenshot" src=
      "images/text.png" /></p>

      <p>Test GUI View</p>
    </div>
  </div>

  <div class="content">
    <h2>Overview</h2>

    <p>The following diagram shows the flow of communication
    between GUI and components.</p>

    <div class="figure">
      <p><img alt="GUI overview" src="images/overview.png" /></p>

      <p>Communication between components and the GUI</p>
    </div>

    <p>Whenever the GUI decides to do a redraw, it pulls from the
    currently selected component a batch of drawing primitives.
    I.e. all drawing primitives from a component come lumped
    together in one message. This reduces communication
    overhead.</p>

    <p>A redraw can become necessary if the GUI window is resized
    or is exposed after being hidden. Moreover the component can
    request a redraw, e.g. because it finished some processing and
    now wants to display the result. The component however does not
    just send a batch of drawing primitives. In that case the GUI
    would possibly be bombarded with draw batches from all over the
    system, where it actually only displays one component. So most
    drawing messages would go to nirvana. The component therefore
    only sends an empty message indicating its request to redraw.
    The GUI checks if that component is actually seleted for
    display and if so does the actual redraw by again pulling a
    draw batch.</p>
  </div>

  <div class="content">
    <h2>API</h2>

    <p>This API is available as the InspectableComponent class in
    <a href=
    "doxygen/html/classcast_1_1core_1_1components_1_1InspectableComponent.html">
    Java</a> and <a href=
    "doxygen/html/classcast_1_1InspectableComponent.html">C++</a>.
    By default all CAST components derive from this class.
    InspectableComponent class contains three abstract methods</p>

    <ul>
      <li><code>redrawGraphics2D()</code></li>

      <li><code>redrawGraphics3D()</code></li>

      <li><code>redrawGraphicsText()</code></li>
    </ul>

    <p>Override these and place any drawing commands inside. Note
    that you can be asked to redraw anytime. Moreover you have a
    method <code>redrawNow()</code> which sends a redraw request to
    the GUI. The GUI then might or might not issue a redraw,
    depending on whether your component is selected or not.</p>
  </div>

  <div class="content">
    <h2>2D Drawing Primitives</h2>

    <p>You can draw points, lines, rectangles, polygons and images.
    Colours are given as RGB with values 0..255. (More to come)</p>
  </div>

  <div class="content">
    <h2>3D Drawing Primitives</h2>

    <p>You can draw points and lines. (More to come)</p>
  </div>

  <div class="content">
    <h2>Text</h2>

    <p>You can print any number of lines, containing control
    characters such '\n' and '\t'. Note that the text display is
    not a log (as in printf-debugging on the console) but as with
    the 2D and 3D case a view of the current content of your
    component.</p>
  </div>

  <div class="content">
    <h2>Example</h2>

    <p>The best documentation is of course code. So there is an
    example component which shows the use of various drawing
    primitives. There is one for <a href=
    "doxygen/html/classcast_1_1ui_1_1inspectable_1_1GUIDemo.html">Java</a>
    and <a href="doxygen/html/GUIDemo_8cpp.html">C++</a>.</p>
  </div><?php
    $string = get_include_contents('include/footer.php');
  echo $string;

  $string = get_include_contents('include/menu.php');
  echo $string;
  ?>
</body>
</html>
