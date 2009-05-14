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

  <title>CAST &gt; Running an Instantiation</title>
<style type="text/css" media="all">
 
/*<![CDATA[*/ @import
"css/cosy.css"; 
/*]]>*/ 
</style>
</head>

<body>
  <?php 
      $string = get_include_contents('include/header.php'); 
      echo$string; 
      ?>

  <div class="content">
    <h2>Running a CAST Instantiation</h2>

    <p>To run a CAST instantiaion you must run an instance of
    <a href=
    "doxygen/classcast_1_1server_1_1CASTProcessServer.html">CASTProcessServer</a>
    on all but one of the machines that are to host components of
    the instantiation. On the remaining machine you must run the
    server with extra arguments to create and run the
    instantiation.</p>
  </div>

  <div class="content">
    <h2>Before You Start</h2>

    <p>Before you start running CAST servers you must start the
    Java nameserver on a machine on your network. This is done with
    the command:</p>
    <pre>
 tnameserv -ORBInitialPort 1050&amp; 
</pre>

    <p>All the machines that are running components must be able to
    connect to the machine running the nameserver on port 1050. We
    will refer to network name of the machine running the
    nameserver as $NAMESERVER.</p>
  </div>

  <div class="content">
    <h2>Standalone Servers</h2>

    <p>On each machine in the instantiation you must run an
    instance of CASTProcessServer. To do this you must run the Java
    CASTProcessServer class from the command line. This is done
    with the following command:</p>
    <pre>
   java -classpath $CLASSPATH:$OUTPUT_DIR/classes  cast.server.CASTProcessServer -h $NAMESERVER
  
</pre>

    <p>You may wish to add extra flags to the java virtual machine
    depending on your local setup. Add the following before
    <code>-classpath</code> if required</p>

    <ul>
      <li><code>-ea</code> - Enable assertions in the Java code run
      within this VM.</li>

      <li><code>-XstartOnFirstThread</code> - <strong>Only on Mac
      OS X</strong> - Must be used on the server that is running
      the <a href="gui.php">GUI</a>.</li>

      <li><code>-Djava.library.path=$OUTPUT_DIR/lib</code> -
      <strong>Only on Mac OS X</strong> - Use if Java has trouble
      finding the native libraries..</li>
    </ul>
  </div>

  <div class="content">
    <h2>Configuration Server</h2>

    <p>On one machine in the instantiation you must run an
    additional CASTProcessServer with arguments to specify the
    configuration of the instantiation. This is done with the
    following command:</p>
    <pre>
   java -classpath $CLASSPATH:$OUTPUT_DIR/classes  cast.server.CASTProcessServer -h $NAMESERVER -f CONFIG
  
</pre>

    <p>Where <code>CONFIG</code> is a path to the <a href=
    "config.php">configuration file</a> for the instantiation. You
    can also specify the following arguments to the process server
    after the class name.</p>

    <ul>
      <li><code>-g</code> - Use the <a href="gui.phg">CAST
      GUI</a>.</li>

      <li><code>-d</code> - Produce some extra debug information
      when connecting the processes.</li>

      <li><code>-r millis</code> - Run for a particular number of
      milliseconds. <strong>Deprecated for the time
      being.</strong></li>
    </ul>
  </div><?php
        $string = get_include_contents('include/footer.php');
      echo $string;

      $string = get_include_contents('include/menu.php');
      echo $string;
      ?>
  <!--  LocalWords:  Config dewey CASTProcessManager louie CPP StraightMan BALT
 -->
  <!--  LocalWords:  CASTDatatypeManager StringSeq wm AudienceWorkingMemory YAY
 -->
  <!--  LocalWords:  AssistantDirector AlwaysPositiveTaskManager AudienceMember
 -->
</body>
</html>
