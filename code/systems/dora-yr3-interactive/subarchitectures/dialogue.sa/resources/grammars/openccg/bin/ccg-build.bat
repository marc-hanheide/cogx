@echo off

call ccg-env
set ANT_HOME=%OPENCCG_HOME%\lib
set PROPS=-Dant.home=%ANT_HOME% -Dopenccg.home=%OPENCCG_HOME%
set XALAN_JARS=%OPENCCG_HOME%\lib\xalan.jar;%OPENCCG_HOME%\lib\xercesImpl.jar;%OPENCCG_HOME%\lib\xml-apis.jar
set CP="%JAVA_HOME%\lib\tools.jar";%OPENCCG_HOME%\lib\ant.jar;%OPENCCG_HOME%\lib\ant-launcher.jar;%XALAN_JARS%
%JAVA% -classpath %CP% %PROPS% org.apache.tools.ant.Main %*

