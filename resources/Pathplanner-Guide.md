This document outlines the naming conventions of the Paths, Autos, adn locations used by FRC team 8032, SAASquatch. Some files may violate these conventions, but if this is intentional they will have clear names such as 'Example Auto.'

# Location Key

Locations:
- D(1-6) (C2): A Drop location, labelled clockwise around the reef. C2 indicates there are two corals dropped in the same area (only for Path names).
- P(1-2): A Pickup location, labelled one for the top or two for the bottom.
- Spawn Locations: Low, Mid, and Top. These are where the robot starts.

![Visual guide to understanding the various locations](/resources/Pathplanner-Guide-Images/Reefscape%20Diagram%20Annotated.png)

# Auto Names

Auto naming conventions are fairly long and complex. Each path consists of many segments. Names do not indicate wait times, but indicate most other relevant data.

Naming Outline:
- Spawn Location
- Sequence of Drop Locations (Separated by ‘,’)
- Direct or Indirect (see Path Names)
- Pickup Location (Top is 1, Bottom is 2 - Not on every path; just ones that are identical otherwise)
- Quantity of Drops (Mono, Double, Triple)
- Layer of the Reef that coral is placed upon
- The word ‘Auto’

An example of this is the following:

Low Spawn D1, D5 P2 Double L1 Auto

A visual representation of this is as follows:

![Low Spawn D1, D5 P2 Double L1 Auto Example](/resources/Pathplanner-Guide-Images/Low%20Spawn%20D1,%20D5%20P2%20Double%20L1%20Auto.png)

# Path Names

Path naming conventions are similar yet simpler. As each path consists of only one segment, they are named in an easy-to-understand format.

Naming Outline:
- Starting Position
- The word ‘to’
- Ending Position
- Optionally; the word ‘Indirect’ or ‘Direct’
  - This is used for cases where a most optimal path will likely collide with another robot.

An example of this is the following:

Top Spawn to D2 Direct

A visual representation of this is as follows:

![Top Spawn to D2 Direct Example](/resources/Pathplanner-Guide-Images/Top%20Spawn%20to%20D2%20Direct.png)