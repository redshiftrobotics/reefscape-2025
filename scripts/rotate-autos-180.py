import json
import os

folder_path = "src\main\deploy\pathplanner\paths"
files = os.listdir(folder_path)

for f in files:
  file_path = folder_path + "\\" + f
  print(file_path)
  with open(file_path, "r") as file:
    data = json.load(file)

  data["goalEndState"]["rotation"] += 180
  if data["goalEndState"]["rotation"] > 180:
    data["goalEndState"]["rotation"] -= 360

  with open(file_path, "w") as file:
    json.dump(data, file, indent=4)