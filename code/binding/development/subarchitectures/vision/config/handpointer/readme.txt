Instructions for configuration of HandPointer component

1. Prepare white paper in table-top scene. White paper should be the lightest object in table-top scene.

2. Run utility eg ./run-handpointer-utility.sh 

3. Adjust the segmented area of white paper with '+' or '-' until it appears completely black and surrounded with green line.

4. Place the hand over segmented area and adjust threshold with '+' or '-' to segment out only skin color. 

5. When skin color is extracted on black background press key 'p' to capture skin color pixels.

6. Repeat with step 5 three times with different hand positions, according to scene lighting.

7. When step 5 is repeated three times, the configuration file is generated from captured skin color pixels.
