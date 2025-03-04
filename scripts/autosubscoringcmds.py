import json
import sys
import pathlib

if len(sys.argv) < 2:
	raise SystemExit("Usage: python[3] " + sys.argv[0] + " <path to auto file>")

in_path = pathlib.Path(sys.argv[1])
if not in_path.exists():
	raise SystemExit("Specified auto file does not exist!")

with open(in_path, "r+") as file:
	doc = json.load(file)
	auto_cmds = doc["command"]["data"]["commands"]
	for cmd in auto_cmds:
		if cmd["type"] == "wait":
			cmd["type"] = "named"
			if cmd["data"]["waitTime"] == 1.5:
				cmd["data"]["name"] = "l4"
				del cmd["data"]["waitTime"]
			elif cmd["data"]["waitTime"] == 2.5:
				cmd["data"]["name"] = "intake"
				del cmd["data"]["waitTime"]
			elif cmd["data"]["waitTime"] == 4.0:
				cmd["data"]["name"] = "l4_algae"
				del cmd["data"]["waitTime"]
			
	file.seek(0)
	json.dump(doc, file, indent=2)	
	
print("Complete!")