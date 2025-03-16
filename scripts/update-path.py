import sys
import os
import json
import pathlib
import csv

if len(sys.argv) < 2:
	raise SystemExit("Usage: Python " + sys.argv[0] + " <path to auto file>")

file_path = pathlib.Path(sys.argv[1])
if not file_path.exists():
	raise SystemExit("Specified auto path does not exist!")

def waypointVals(name: str, entries):
   for entry in entries:
      if entry[2] == name:
         return (float(entry[3]), float(entry[4]), float(entry[5]))

def update(doc, entries):
  

  for waypoint in doc["waypoints"]:
    (xPos, yPos, theta) = waypointVals(waypoint["linkedName"], entries)
    dx = xPos - waypoint["anchor"]["x"]
    dy = yPos - waypoint["anchor"]["y"]
    waypoint["anchor"]["x"] = xPos
    waypoint["anchor"]["y"] = yPos
    
    if waypoint["prevControl"] is None:
      doc["idealStartingState"]["rotation"] = theta
    else:
      waypoint["prevControl"]["x"] += dx
      waypoint["prevControl"]["y"] += dy
    
    if waypoint["nextControl"] is None:
      doc["goalEndState"]["rotation"] = theta
    else:
      waypoint["nextControl"]["x"] += dx
      waypoint["nextControl"]["y"] += dy
  

try:
  waypointsFile = open(os.path.join(os.path.dirname(__file__),"waypoints.csv"), "r")
  csvFile = csv.reader(waypointsFile)
except:
   raise SystemExit("No waypoints found")

entries = []
for entry in csvFile:
  entries.append(entry)
filePath = os.path.join(os.path.dirname(__file__), sys.argv[1])
print(filePath)

file = open(filePath, "r")
doc = json.load(file)

update(doc, entries)
outfile = open(filePath, "w")
json.dump(doc, outfile, indent=2)