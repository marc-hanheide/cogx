declare namespace log4j="http://jakarta.apache.org/log4j/";

let $d := doc("log.xml")
return fn:max($d/*/log4j:event[fn:starts-with(@logger,"StopWatch")]/@timestamp)-fn:min($d/*/log4j:event[fn:starts-with(@logger,"StopWatch")]/@timestamp)