declare namespace log4j="http://jakarta.apache.org/log4j/";

for $e  in /*/log4j:event[fn:starts-with(@logger,"WMLogger")]
  let $msg := $e/log4j:message
  let $op := $msg/WM_OP/WMC/cast.cdl.WorkingMemoryChange/operation
  let $id := $msg/WM_OP/WMC/cast.cdl.WorkingMemoryChange/address/id
  let $sa := $msg/WM_OP/WMC/cast.cdl.WorkingMemoryChange/address/subarchitecture
  let $type := $msg/WM_OP/WMC/cast.cdl.WorkingMemoryChange/type  

  (:where fn:starts-with($e/@logger,"WMLogger"):)
	return fn:concat(data($e/@timestamp),',',$type,',',$op,',',$sa,':',$id)
(:    
return fn:concat(data($e/@timestamp),fn:tokenize(data($msg),":")[2])
:)  
