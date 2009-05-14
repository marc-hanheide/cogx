find . -name "*_msc.png" -exec rm {} \;
find . -name "*_msc" -exec mscgen -i {} -o {}.png -T png \;
find . -name "*_msc" -exec echo {} converted to {}.png \;
