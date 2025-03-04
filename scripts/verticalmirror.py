import sys
import json
import os

# if (len(sys.argv) == 1):
#     print("Not enough arguments, specify a file path")
#     sys.exit()

input_file_path = "src\\main\\deploy\\pathplanner\\paths\\Coordinate Testing.path" #sys.argv[1]
file_stream = open(os.path.abspath(input_file_path), "r")
input_file_content = file_stream.read()
file_stream.close()

path = json.loads(input_file_content)

for waypoint in path["waypoints"]:
    # Step 1: make center of field 0
    # Try/Excepts are because sometimes these values are null
    waypoint["anchor"]["y"] -= 4
    try:
        waypoint["prevControl"]["y"] -= 4
    except:
        pass
    
    try:
        waypoint["nextControl"]["y"] -= 4
    except:
        pass

    # Step 2: mirror coordinates
    waypoint["anchor"]["y"] *= -1
    try:
        waypoint["prevControl"]["y"] *= -1
    except:
        print("Error mirroring prevControl")
    
    try:
        waypoint["nextControl"]["y"] *= -1
    except:
        print("Error mirroring nextControl")

    # Step 3: convert back to proper coordinate system
    waypoint["anchor"]["y"] += 4
    try:
        waypoint["prevControl"]["y"] += 4
    except:
        pass
    
    try:
        waypoint["nextControl"]["y"] += 4
    except:
        pass

path["idealStartingState"]["rotation"] = 180 - path["idealStartingState"]["rotation"]
path["goalEndState"]["rotation"] = 180 - path["goalEndState"]["rotation"]

output = json.dumps(path)

output_file_stream = open(os.path.abspath(input_file_path).replace(".path", " Mirrored.path"), "w")
output_file_stream.write(output)
output_file_stream.close()