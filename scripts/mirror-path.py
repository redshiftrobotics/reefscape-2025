import sys
import json
import pathlib

if len(sys.argv) < 3:
	raise SystemExit("Usage: python[3] " + sys.argv[0] + " <path to auto file or directory> <path to write output>")

in_path = pathlib.Path(sys.argv[1])
if not in_path.exists():
	raise SystemExit("Specified auto path does not exist!")

out_path = pathlib.Path(sys.argv[2])

def mirror(inPath):
  file = open(inPath)
  
  doc = json.load(file)
  
  
  
try:
  outfile = open(out_path, "x")
except:
  raise SystemExit("Specified write path already exists")

json.dump(mirror(in_path), outfile)