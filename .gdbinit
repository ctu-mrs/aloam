### EIGEN PRETTY PRINTER
python
from os.path import expanduser
home = expanduser("~")
import sys
sys.path.insert(0, home + "/.gdb/eigen")
from printers import register_eigen_printers
register_eigen_printers (None)
end

### OpenCV IMAGE SHOWER
source ~/.gdb/gdb-imshow/cv_imshow.py

### DASHBOARD (shows source, vars etc. nicely)
# uncomment the next two lines to activate gdb-dashboard
source ~/.gdb/gdb-dashboard/.gdbinit
dashboard -layout source expressions

### USER SETTINGS
# this line is necessary to enable breakpoints in a not-yet
# loaded program (which is needed for debugging roslaunched
# programs)
set breakpoint pending on
# you can specify breakpoints which should be loaded below in a format
# break filename.ext:line_number
# such as
# break utility_fcs.cpp:33
# break feature_extractor.cpp:285
