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

  <title>CAST &gt; Run-time Configuration</title>
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
    <h2>Writing a CAST Config File</h2>

    <p>Assuming that you have a subarchitecture or an architecture
    that you have compiled and now want to run, you can configure
    it, and the included processing components with a config file
    which is then passed to the CAST server. For the sake of
    consistency we always give the config files a
    <code>.cast</code> suffix and store them in a /config
    directory. In a CAST config file you can describe anything from
    a single subarchitecture (if you want to test it on its own) or
    to a full architecture with any number of subarchitecture. For
    each component you can specify the host it runs on and a list
    of command line arguments.</p>
  </div>

  <div class="content">
    <h2>The Format</h2>

    <h3>Comments</h3>

    <p>Comment lines start with #.</p>
  </div>

  <div class="content">
    <h3>The HOST Line</h3>

    <p>If you want to specify a host machine to be used as the
    default for all subsequent components, you can specify it using
    the keyword HOST. E.g. The entry...</p>
    <pre>
 HOST dewey 
</pre>

    <p>... will cause the configuration code to assign all
    subsequent components the hostname of dewey unless
    overridden.</p>
  </div>

  <div class="content">
    <h3>The SUBARCHITECTURE Section</h3>

    <p>The majority of a config file will probably be made up of
    subarchitecture descriptions. A subarchitecture description is
    started with the following line</p>
    <pre>
SUBARCHITECTURE &lt;subarchName&gt; &lt;opt:subarchHost&gt;
</pre>

    <p>This states that the following components make up the
    subarchitecture and run on the optionally specified host.
    E.g.</p>
    <pre>
SUBARCHITECTURE vision.sa dewey
</pre>

    <p>Describes a subarchitecture vision. with its components to
    run as default on the optionally specified host. The lines
    following this must describe a working memory component, and
    task manager component and zero or more processing
    components.</p>
  </div>

  <div class="content">
    <h3>The Working Memory Line</h3>

    <p>You describe a subarchitecture working memory with the
    following line...</p>
    <pre>
&lt;opt:host&gt; &lt;lang&gt; WM &lt;class&gt; &lt;opt:command line&gt;
</pre>

    <p>Where <code>&lt;opt:host&gt;</code> is an optional name of
    the host to run the component on, <code>&lt;lang&gt;</code>
    specifies the language the working memory was written in and is
    either <code>CPP</code> or <code>JAVA</code>, WM states that
    this component is a working memory, <code>&lt;class&gt;</code>
    is the name of the component class (that has been passed to the
    CASTProcessManager in C++ or is a fully qualified java class
    name), and <code>&lt;opt:command line&gt;</code> is a list
    command parameters which are passed to the components configure
    method. These are described in more detail here. E.g.</p>
    <pre>
JAVA WM org.cognitivesystems.planning.workingmemory.PlanningWorkingMemory --debug -v 2 -N
</pre>

    <p>... specified a java working memory component with some
    arguments.</p>
  </div>

  <div class="content">
    <h3>The Task Manager Line</h3>

    <p>This is the same format as the working memory line, but with
    TM instead of WM. E.g.</p>
    <pre>
louie JAVA TM cast.architecture.subarchitecture.AlwaysPositiveTaskManager 
</pre>
  </div>

  <div class="content">
    <h3>Unmanaged Component lines</h3>

    <p>Unmanaged components are added to the subarchitecture using
    a line of the following format...</p>
    <pre>
&lt;opt:host&gt; &lt;lang&gt; UM &lt;name&gt; &lt;class&gt; &lt;opt:command line&gt;
</pre>This is almost identical to preceding lines, with the
addition of the <code>&lt;name&gt;</code> argument which specifies
the unique identifier of the processing component (uniqueness is
not checked yet!). E.g.
    <pre>
CPP UM straight.man StraightMan -s "what's brown and sticky"
</pre>
  </div>

  <div class="content">
    <h3>Managed Component lines</h3>

    <p>The same as unmanaged components, but with MG instead of
    UM.</p>
  </div>

  <div class="content">
    <h2>BALT Components and Connections</h2>

    <p>You can also add BALT components and connections to a
    configuration file. This may be necessary if you have extra
    modules than run outside of CAST.</p>
  </div>

  <div class="content">
    <h3>BALT Components</h3>

    <p>Before you can connect up BALT components you must define
    them using the COMPONENT flag. This is done in a similar way to
    other components...</p>
    <pre>
COMPONENT &lt;opt:host&gt; &lt;lang&gt; &lt;name&gt; &lt;class&gt; &lt;opt:command line&gt;
</pre>

    <p>e.g.</p>
    <pre>
COMPONENT JAVA extra.comp comedyarch.subarchitectures.stage.StraightMan -s "what's brown and sticky"
</pre>

    <p>Would create an extra java component in the configuration
    extra..</p>
  </div>

  <div class="content">
    <h3>BALT Connections</h3>

    <p>You can connect BALT components and CAST components directly
    using the following line format:</p>
    <pre>
CONNECTION &lt;datatype&gt; &lt;sender list&gt; &lt;communication style&gt; &lt;receiver list&gt;
</pre>

    <p>Where <code>&lt;datatype&gt;</code> describes the class of
    data that the communication will use (which must be understood
    by CASTDatatypeManager), <code>&lt;sender list&gt;</code> is a
    space-separated list of <code>&lt;name&gt;</code>s of
    previously defined components, <code>&lt;communication
    style&gt;</code> is either <code>PUSH_TO</code> or
    <code>PULL_FROM</code> to state whether the senders push or
    pull to the receivers, and <code>&lt;receiver list&gt;</code>
    is a list of previously defined components. E.g.</p>
    <pre>
CONNECTION StringSeq extra.comp PUSH_TO funny.man
</pre>
  </div>

  <div class="content">
    <h2>"Command Line" Configuration</h2>

    <p>The command line-style arguments give to any process are
    parsed into key-value pairs and are placed into the map given
    as input to the components configure method. Any command line
    entry is treated as a key. Keys without attached values are
    given the . Strings are not broken up. E.g.</p>
    <pre>
--debug -v 2 -N -say "Hello World"
</pre>

    <p>would result in the key-value pairs</p>
    <pre>
--debug = true
-v = 2 
-N  = true
-say = Hello World
</pre>
  </div>

  <div class="content">
    <h2>Comedian Example Config File</h2>
    <pre>
HOST localhost

SUBARCHITECTURE stage.subarch
JAVA WM comedyarch.subarchitectures.stage.StageWorkingMemory #--log true --debug false
JAVA TM cast.architecture.subarchitecture.AlwaysPositiveTaskManager #--log
JAVA UM straight.man comedyarch.subarchitectures.stage.StraightMan -s "what's brown and sticky"
JAVA MG funny.man comedyarch.subarchitectures.stage.FunnyMan -p "a stick" --log


SUBARCHITECTURE director.subarch
#in this case another wm is not necessary
#it just needs the correct ontology
CPP WM AudienceWorkingMemory #--log true --debug false
#JAVA WM comedyarch.subarchitectures.stage.StageWorkingMemory #--log --debug
JAVA TM cast.architecture.subarchitecture.AlwaysPositiveTaskManager #--log
#JAVA MG ass.director comedyarch.subarchitectures.director.AssistantDirector --log #--debug
CPP MG ass.director AssistantDirector --log #--debug
#JAVA MG director comedyarch.subarchitectures.director.Director --log  #--debug
CPP MG director Director --log  #--debug


SUBARCHITECTURE audience.subarch
CPP WM AudienceWorkingMemory #--log --debug
CPP TM AlwaysPositiveTaskManager  #--log
CPP MG audience.member AudienceMember --reaction "YAY!" #--log #--debug
</pre>
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
