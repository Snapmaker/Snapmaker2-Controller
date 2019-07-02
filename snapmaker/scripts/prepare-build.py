import shutil
import sys
from os.path import isdir, isfile, join

Import("env")

platform = env.PioPlatform()
board = env.BoardConfig()

print "++++++++++++++++++++++++++++++Prepare env start++++++++++++++++++++++++++++++" 
print "Prepare env for " + env.get("BOARD")

frwk = env.get("PIOFRAMEWORK")
pkg = platform.frameworks[frwk[0]]["package"]

pkg_dir = platform.get_package_dir(pkg)

print "Package is " + pkg
print "Location: " + pkg_dir

mcu = board.get('build.mcu')

build_script = "platformio-build-%s.py" % mcu[0:7]
build_script_path = join(pkg_dir, "tools", build_script)
shutil.copy(join(sys.path[0], build_script), build_script_path)

"""
frwk_dir = join(pkg_dir, mcu[0:6].upper())
if isdir(frwk_dir):
  print "%s is exist." % frwk_dir
  # TODO: here need to add comparation between src and dst framework
else:
  print "%s is not exist." % frwk_dir
  print "now create it"
  shutil.copytree(join(sys.path[0], mcu[0:6].upper()), frwk_dir)
"""

print "++++++++++++++++++++++++++++++Prepare env end++++++++++++++++++++++++++++++" 