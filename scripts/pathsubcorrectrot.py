import json
import sys
import pathlib

if len(sys.argv) < 2:
	raise SystemExit("Usage: python[3] " + sys.argv[0] + " <path to auto file>")

in_path = pathlib.Path(sys.argv[1])
if not in_path.exists():
	raise SystemExit("Specified path file does not exist!")

def do_file(in_path):
	with open(in_path, "r+") as file:
		doc = json.load(file)
		rot = doc["idealStartingState"]["rotation"]
		rot -= 180
		while rot < 0 or rot > 360:
			if rot < 0:
				rot += 360
			else:
				rot -= 360
		
		doc["idealStartingState"]["rotation"] = rot
		file.seek(0)
		file.truncate()
		json.dump(doc, file, indent=2)
		
	print("Wrote to \"" + file + "\"!")

if in_path.is_file():
	do_file(in_path)
elif in_path.is_dir():
	for child in in_path:
		if in_path.is_file():
			do_file(in_path)
	
else:
	raise SystemExit("Unknown type of path!")