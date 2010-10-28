declare namespace log4j="http://jakarta.apache.org/log4j/";

fn:max(/*/log4j:event[fn:starts-with(@logger,"StopWatch")]/@timestamp)-fn:min(/*/log4j:event[fn:starts-with(@logger,"StopWatch")]/@timestamp)