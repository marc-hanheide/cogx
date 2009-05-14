<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Strict//EN"
    "http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd">

<html xmlns="http://www.w3.org/1999/xhtml">
<head>
  <? require_once("include/functions.php") ?>
  <meta name="generator" content=
  "HTML Tidy for Linux/x86 (vers 1 September 2005), see www.w3.org" />
  <meta http-equiv="content-type" content=
  "text/html; charset=us-ascii" />
  <link rel="icon" href="images/icon.png" type="image/png" />

  <title>CAST &gt; Installation</title>
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
    <h2>Platforms</h2>

    <p>CAST has been run successfully on the following
    platforms:</p>

    <ul>
      <li>Linux:</li>

      <li style="list-style: none">
        <ul>
          <li>Fedora Core 5 &amp; 6, i586 and x86_64, single core
          and smp.</li>

          <li>CentOS 4 &amp; 5, i586 and x86_64, single core and
          smp.</li>

          <li>Ubuntu 7</li>

          <li>Red Hat 7.3</li>
        </ul>
      </li>

      <li>Mac OS X:</li>

      <li style="list-style: none">
        <ul>
          <li>G4, Power PC</li>

          <li>Intel</li>
        </ul>
      </li>
    </ul>

    <p>CAST should run on any modern Linux distribution.</p>
  </div>

  <div class="content">
    <h2>Dependencies</h2>

    <p>To install CAST you need the following packages on your
    system</p>

    <ul>
      <li><a href="http://omniorb.sourceforge.net/">omniORB</a> 4.0
      or greater. This also requires Python &gt; 2.0 which most
      systems should have as standard.</li>

      <li><a href="http://www.boost.org/">Boost</a> 1.33 or
      greater.</li>

      <li><a href="http://java.sun.com">Java JDK</a> 1.5 or
      greater.</li>

      <li><a href="http://ant.apache.org/">Apache Ant</a> 1.6.2 or
      greater.</li>

      <li><a href="http://www.cmake.org">CMake</a> 2.4.3 or
      greater.</li>

      <li><a href=
      "http://http://pkg-config.freedesktop.org/wiki/">pkg-config</a>.</li>

      <li><a href="http://www.eclipse.org/swt/">SWT</a> 3.2 or
      greater.</li>

      <li><a href="https://jogl.dev.java.net/">JOGL</a> 1.0 or
      greater.</li>
    </ul>Before compiling CAST please make sure you set and/or test
    the following:

    <ul>
      <li>You have the environment variables JAVA_HOME and ANT_HOME
      pointing to your Java and Ant installations.</li>

      <li>That omniidl is setup correctly. E.g. the command
      `omniidl -V` should return something like "omniidl version
      1.0". If the command is not found ensure that omniidl is in
      your PATH. If you get an error like "omniidl: Could not open
      IDL compiler module _omniidlmodule." locate _omniidlmodule.so
      and add the containing directory to the PYTHONPATH, e.g. (in
      bash) `export
      PYTHONPATH=/usr/local/lib/_python_version_/site-packages`.</li>

      <li>That pkg-config can find omniORB. The command `pkg-config
      --cflags omniORB4` should return something like
      "-D__processor_unknown__ -D__linux__ -D__OSVERSION__=2
      -I/usr/local/include". If you get an error like "Package
      omniORB4 was not found in the pkg-config search path", locate
      omniORB4.pc and add the containing directory to the
      PKG_CONFIG_PATH, e.g. (in bash) `export
      PKG_CONFIG_PATH=/usr/local/lib/pkgconfig`.</li>

      <li>That Java is installed correctly. E.g. `java -version`
      should return something like "java version "1.5.0_07"".</li>

      <li>That Apache Ant is installed correctly. E.g. `ant
      -version` should return something like "Apache Ant version
      1.6.5 compiled on June 2 2005".</li>

      <li>That the jar files from the SWT and JOGL packages are
      contained in your java CLASSPATH.</li>
    </ul>
  </div>

  <div class="content">
    <h2>Installation</h2>

    <p>CAST is installed in two parts, the Java installation and
    the C++ installation. You must install both parts even if you
    are only interested in using one of the two languages. If you
    have any problems with these instructions, please consult the
    <a href="faq.php">FAQ</a>.</p>

    <ol>
      <li>Download a tarball of the latest release from <a href=
      "http://sourceforge.net/project/showfiles.php?group_id=199480">
      sourceforge</a>.</li>

      <li>Extract the tarball. This will produce a directory called
      cast-x.y.z. which we will refer to as $CAST_HOME for the
      remainder of these instructions.</li>

      <li>Change directory in to $CAST_HOME.</li>

      <li>Set the installation prefix for the java code. Edit the
      variable <code>output</code> on line 4 of build.xml to
      be the path that will contain all of the results of compiling
      CAST. We will refer to this as $OUTPUT_DIR. The default value
      is $CAST_HOME/output.</li>

      <li>Compile the java parts of cast by running `ant all`. This
      command actually executes 3 commands `ant configure` (which
      determines which directories to build based on the build.dirs
      file), `ant idl` which generates the CORBA parts of CAST from
      idl files, and `ant compile` which compiles all the required
      java sources.</li>

      <li>This will produce a directory $OUTPUT_DIR/classes
      containing the compiled class files.</li>

      <li>Create a directory in which to build the C++ code, for
      example $CAST_HOME/BUILD.</li>

      <li>Change in to this directory and run `ccmake -i
      $CAST_HOME` (e.g. `ccmake -i ..` in the above example).</li>

      <li>This brings up the cmake GUI. Press 'c' to configure the
      code, and press it repeatedly until the asterisks have been
      removed and the 'g' to generate option appears. If you
      receive any error messages at this point please correct the
      problems and double-check the tests listed above.</li>

      <li>On the GUI set the value OUTPUT to be <strong>the
      same</strong> as the $OUTPUT_DIR you used previously. If you
      did not change it previously then you don't need to change it
      here.</li>

      <li>Press 'g' to generate the build files.</li>

      <li>Run `make install` in your build directory. This will
      generate the necessary C++ code from IDL files and compile
      the C++ parts of CAST.</li>

      <li>This will produce directories called
      $CAST_HOME/output/lib and $CAST_HOME/output/include
      containing the shared library files and header files.</li>
    </ol>

    <p>Once you have got this far you should test your installation
    by running the <a href="comedian.php">example code</a> which
    was compiled along with CAST.</p>
  </div><?php
        $string = get_include_contents('include/footer.php');
      echo $string;

      $string = get_include_contents('include/menu.php');
      echo $string;
      ?>
</body>
</html>
