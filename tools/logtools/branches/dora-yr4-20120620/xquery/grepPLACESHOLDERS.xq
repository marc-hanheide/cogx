declare namespace log4j="http://jakarta.apache.org/log4j/";

  for $e  in /*/log4j:event
  let $msg := $e/log4j:message
  where fn:starts-with($e/@logger,"planner.sa.MotiveStatistics") and fn:starts-with(data($msg),"PLACEHOLDER")
  return fn:concat(data($e/@timestamp),fn:tokenize(data($msg),":")[2])
  
