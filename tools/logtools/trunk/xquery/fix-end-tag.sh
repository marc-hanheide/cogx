#!/bin/sh

sed 's/\!\[CDATA\[<//' | sed 's/>\]\]>/>/'
echo "</log4j:logsequence>"
