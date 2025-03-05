import sys
import json
import pathlib

locationMirror = {
  "S1": "S6",
  "S2": "S5",
  "S3": "S4",
  "S4": "S3",
  "S5": "S2",
  "S6": "S1",
  "D1": "D6",
  "D2": "D5",
  "D3": "D4",
  "D4": "D3",
  "D5": "D2",
  "D6": "D1",
  "P1": "P2",
  "P2": "P1"
}

if len(sys.argv) < 3:
	raise SystemExit("Usage: python[3] " + sys.argv[0] + " <path to auto file or directory> <path to write output>")

in_path = pathlib.Path(sys.argv[1])
if not in_path.exists():
	raise SystemExit("Specified auto path does not exist!")

out_path = pathlib.Path(sys.argv[2])

def mirror(inPath):
  file = open(inPath)
  
  doc = json.load(file)

  for waypoint in doc["waypoints"]:
    waypoint["anchor"]["y"] = 8 - waypoint["anchor"]["y"]
    if not waypoint["prevControl"] is None:
      waypoint["prevControl"]["y"] = 8 - waypoint["prevControl"]["y"]
    if not waypoint["nextControl"] is None:
      waypoint["nextControl"]["y"] = 8 - waypoint["nextControl"]["y"]
  
  doc["goalEndState"]["rotation"] *= -1

  doc["idealStartingState"]["rotation"] *= -1
  
  return doc 
  
try:
  outfile = open(out_path, "x")
except:
  raise SystemExit("Specified out path already exists")

json.dump(mirror(in_path), outfile, indent=2)