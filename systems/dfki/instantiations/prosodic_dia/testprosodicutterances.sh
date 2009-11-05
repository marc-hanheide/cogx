#!/bin/bash
if [ $# -ne 1 ]; then
        echo 1>&2 Usage: $0 InputFile [plain txt or prosodic]
        exit 127
fi
#echo "ProsodicTextToRawMARYXml takes as input: "
#echo "1) file containing prosodic strings."
#echo "2) MaryXMLheader."
#echo "3) location to save RawMaryXML and wav files ."
#echo "4) a flag indicating whether to save wav and xml files."
java -classpath $CLASSPATH:../../output/classes/ comsys.components.tts.ProsodicTextToRawMARYXml $1 ../../subarchitectures/comsys/src/java/comsys/components/tts/RAWMARYXMLhead.xml ./maryXMLs/  True
