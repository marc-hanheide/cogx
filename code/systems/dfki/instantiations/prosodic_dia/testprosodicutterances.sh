#!/bin/bash
if [ $# -ne 1 ]; then
        echo 1>&2 Usage: $0 InputFile [plain txt or prosodic]
        exit 127
fi

java -classpath $CLASSPATH:../../output/classes/ comsys.components.tts.ProsodicTextToRawMARYXml $1 ../../subarchitectures/comsys/src/java/comsys/components/tts/RAWMARYXMLhead.xml ./maryXMLs/ 
