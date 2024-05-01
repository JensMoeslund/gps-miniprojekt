import os
import hashlib
import json
import subprocess
import platform

checksums = dict()
changes = dict()
old_data = dict()
paths = list()

proj_root = os.path.abspath(".")
thisfilepath = os.path.dirname(__file__)
def compile_figs(d: dict):
    global proj_root
    my_env = os.environ.copy()
    for file in d:
        head, tail = os.path.split(file)
        print(f"Running file:\t{tail}")
        filepath = os.path.join(proj_root, head)
        os.chdir(filepath)
        if platform.system() == "Windows":
            try:
                subprocess.call([thisfilepath+os.path.join("venv","Scripts","python."), os.path.join(filepath,tail)])
            except:
                subprocess.call([thisfilepath+os.path.join(".venv","Scripts","python"), os.path.join(filepath,tail)])
        else:
            try:
                subprocess.call([os.path.join(thisfilepath,"venv","bin","python"), os.path.join(filepath,tail)])
            except:
                subprocess.call([os.path.join(thisfilepath,".venv","bin","python"), os.path.join(filepath,tail)])            


for root, dirs, files in os.walk("chapters"):
    for file in files:
        if file.endswith(".py"):
            paths.append(os.path.join(root, file))

for path in paths:
    checksums[path] = hashlib.md5(open(path, "rb").read()).hexdigest()

data = json.dumps(checksums, indent=4, sort_keys=True)

try:
    f = open("py_check.json", "r")
    old_data = json.loads(f.read())
    f.close()
except (FileNotFoundError, json.decoder.JSONDecodeError) as error:
    compile_figs(checksums)
    os.chdir(proj_root)
    f = open("py_check.json", "w")
    f.write(data)
    f.close()
    exit()
except ... as error:
    print(error)
    exit()

for file in checksums:
    if file not in old_data.keys():
        changes[file] = checksums[file]
    elif old_data[file] != checksums[file]:
        print(f"{file} did change!")
        changes[file] = checksums[file]

compile_figs(changes)
os.chdir(proj_root)
f = open("py_check.json", "w")
f.write(data)
f.close()
