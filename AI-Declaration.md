# Team 09 GenAI Declaration 

Declaration of Generative AI usage for Assignment #2 Part B (Task 3). 

## Acknowledge

We acknowledge the use of  for the following purposes:

* [x] for background research in the drafting of this assessment. 
* [x] to generate materials that were included within the final code submission in modified form.

## Describe
We used AI to come up with the best solution in terms of exploration and to find bugs within our code

## Evidence
Generative AI system: Claude
Prompt: Hey Claude we are makiing a robot go around an arena looking for a specific coloured beacon we need to come up with a way of exploring efficiently, what ideas do you have for this
Output:Here are some solid exploration strategies for your beacon-hunting robot:
Systematic Search Patterns

Spiral search — start from the centre and spiral outward (or vice versa). Great coverage, no repeated areas, works well in open arenas
Lawnmower/boustrophedon — sweep back and forth in rows. Very efficient but needs good odometry to stay aligned
Grid-based search — divide the arena into cells, visit each in sequence. Easy to track what's been covered

Reactive/Probabilistic Approaches

Random walk with wall-following — simple to implement, surprisingly decent coverage over time, but not optimal
Frontier-based exploration — build a map as you go and always drive toward the nearest unexplored "frontier" edge. Very efficient and adapts to obstacles
Probability map — maintain a grid where each cell has a likelihood of containing the beacon; move toward high-probability unexplored zones

Beacon-Specific Tricks

Rotate in place first — before moving, do a 360° scan from your current position. You might spot the beacon immediately without moving at all
Colour gradient following — if your camera can detect a faint or partial signal of the target colour, treat signal strength as a gradient and hill-climb toward it
Elevation advantage — if the robot can raise a sensor, scanning from height dramatically increases visible range

Practical Recommendations
For a competition/arena setting, I'd suggest combining a few:

Rotate-first scan at the start
Spiral or lawnmower as the primary movement pattern
Colour detection running continuously so you break out of the pattern the moment the beacon is spotted
Wall-following fallback if the robot gets stuck

A few questions that would help narrow this down — what sensors does the robot have (camera, lidar, IR?), and do you know the arena size and whether there are obstacles?
Prompt: Hey Claude we're having some issues with our robot taking pictures of beacons we think the rsv values are wrong can you suggest some new ones that might work better. Heres the current code. COLOUR_RANGES = {
    'yellow': ([20,  80,  80], [35, 255, 255]),
    'green':  ([36,  40,  40], [89, 255, 255]),
    'blue':   ([90,  40,  40], [130, 255, 255]),
    'red':    ([0,  150, 150], [10, 255, 255]),
}

RED_UPPER = ([165, 150, 150], [180, 255, 255])

MIN_CONTOUR_AREA = 2000
LOCK_WIDTH_FRAC  = 0.25
EDGE_MARGIN      = 30
Output: These are HSV ranges (not RSV — easy mix-up!). Here are some suggestions to try:
Suggested new ranges
python
COLOUR_RANGES = {
    'yellow': ([18,  60, 60], [38, 255, 255]),  
    'green':  ([36,   30,  30], [89, 255, 255]),   
    'blue':   ([85,   30,  30], [135, 255, 255]),  
    'red':    ([0,   100, 100], [10, 255, 255]),   
}

RED_UPPER = ([165, 150, 150], [180, 255, 255])     

## Declaration

* [x] We confirm that no content created by generative AI has been presented as our own work.
