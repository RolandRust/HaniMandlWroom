

files = ["resources_de.h", "resources_en.h"]
variables = {}
x = []


#dic key schreiben
datei = open(files[0], 'r')
for line in datei:
  if line.strip().startswith("char"):
    variables = {**variables, **{line.split("=")[0].strip().split(" ")[1].strip().split("[")[0].strip():[line.split("=")[0].strip().split(" ")[1].strip().split("[")[0].strip()]}}
datei.close();

#alle sprachdateien durchachern
for i in range(len(files)):
  datei = open(files[i], 'r')
  for line in datei:
    if line.strip().startswith("char"):
      key = line.split("=")[0].strip().split(" ")[1].strip().split("[")[0].strip()
      if key == variables[key][0]:
        variables[key].append(line.strip().split("\"")[1].strip())
  datei.close();
  #kontrolieren ob alle variablen deviniert wurden im sprachekeil. ansonsten die vom default setting nehmen
  for key in variables.keys():
    if (len(variables[key])) != i + 2:
        variables[key].append(variables[key][1])

#file schreiben
datei = open("resources.h", 'w')
for key in variables.keys():
  line = "char %s[][30] = {" % variables[key][0]
  for i in range(len(variables[key])):
    if i >= 1 and i < len(variables[key]) - 1:
      line = line + "\"%s\"," % variables[key][i]
    elif i == len(variables[key]) - 1:
      line = line + "\"%s\"};\n" % variables[key][i]
  datei.write(line)

datei.close()