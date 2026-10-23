# Request from the mission node to the laser/servo payload controller
# (drone_control/laser_control), and the controller's echo of the actual state.
#
# The controller only switches a laser on when enable is true AND its own
# interlocks hold (hardware_enable parameter, vehicle armed and airborne,
# command fresh). A physical kill switch on the laser supply stays outside
# all software.

std_msgs/Header header
bool enable            # lasers requested on (mission) / actually on (state echo)
string pattern         # "triangle" or "line"; empty = keep the current arrangement
