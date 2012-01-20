#/bin/bash

cdsrImageGen () {
	java -classpath ../../output/classes cdsr.gui.LineMapImageGenerator data/$1.cdsr image-gen/$1.png > image-gen/$1.txt
}

cdsrImageGen classroom-1
cdsrImageGen classroom-2
cdsrImageGen classroom-3-real
cdsrImageGen classroom-3-sim
cdsrImageGen classroom-UG40-real
cdsrImageGen classroom-222-classroom-format-real
cdsrImageGen classroom-222-seminar-format-real
cdsrImageGen studio-mistry-sim
cdsrImageGen studio-klenk-sim
