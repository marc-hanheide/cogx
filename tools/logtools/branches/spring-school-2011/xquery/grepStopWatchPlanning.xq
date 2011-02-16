declare namespace log4j="http://jakarta.apache.org/log4j/";

  for $e  in /*/log4j:event
  let $msg := $e/log4j:message
  let $ts := data($e/@timestamp)
  let $tokenized := fn:tokenize(data($msg),": ")
  let $extrainfo := fn:tokenize($tokenized[1],"/")

  where fn:starts-with($e/@logger,"StopWatch.tictoc.PlanningTask") and fn:starts-with($extrainfo[2],"SUCCEEDED")

  return fn:concat($ts," ",$tokenized[2]," ",$extrainfo[1]," ",$extrainfo[4]," ",$extrainfo[6],"\n")
  
