declare namespace log4j="http://jakarta.apache.org/log4j/";
declare variable $minTS:=fn:min(/log4j:eventSet/log4j:event/@timestamp);
declare variable $loggers:=fn:distinct-values(/log4j:eventSet/log4j:event/@logger);


<ANNOTATION_DOCUMENT xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://www.mpi.nl/tools/elan/EAFv2.2.xsd" DATE="2005-09-24T11:27:35+01:00" AUTHOR="" VERSION="2.2" FORMAT="2.2">
	<HEADER MEDIA_FILE="" TIME_UNITS="milliseconds">
		<MEDIA_DESCRIPTOR MEDIA_URL="file:///tmp/none.mov" MIME_TYPE="video/quicktime"/>
	</HEADER>
(: time order here :)
	<TIME_ORDER>

{
for $x at $pos in fn:distinct-values(/log4j:eventSet/log4j:event/@timestamp)
order by $x
return <TIME_SLOT TIME_SLOT_ID="{$pos}" TIME_VALUE="{data($x)-$minTS}"/>
}
	</TIME_ORDER>


(: tiers here :)
{
for $x at $lpos in $loggers
return 
<TIER TIER_ID="{$x}" PARTICIPANT="hurga" LINGUISTIC_TYPE_REF="Default" DEFAULT_LOCALE="de">
  {
  for $e at $epos in /log4j:eventSet/log4j:event[@logger=$x]/log4j:message
  return <ANNOTATION><ALIGNABLE_ANNOTATION ANNOTATION_ID="a{$lpos}_{$epos}" TIME_SLOT_REF1="ts1" TIME_SLOT_REF2="ts2"><ANNOTATION_VALUE>{data($e)}</ANNOTATION_VALUE></ALIGNABLE_ANNOTATION></ANNOTATION>
  }
</TIER>
}


	<LINGUISTIC_TYPE LINGUISTIC_TYPE_ID="Default" TIME_ALIGNABLE="true" GRAPHIC_REFERENCES="false"/>
	<LOCALE LANGUAGE_CODE="de" COUNTRY_CODE="DE"/>
	<CONSTRAINT STEREOTYPE="Time_Subdivision" DESCRIPTION="Time subdivision of parent annotation&apos;s time interval, no time gaps allowed within this interval"/>
	<CONSTRAINT STEREOTYPE="Symbolic_Subdivision" DESCRIPTION="Symbolic subdivision of a parent annotation. Annotations refering to the same parent are ordered"/>
	<CONSTRAINT STEREOTYPE="Symbolic_Association" DESCRIPTION="1-1 association with a parent annotation"/>
</ANNOTATION_DOCUMENT>
