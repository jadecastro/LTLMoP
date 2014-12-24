# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
rob1_pickup, 1
rob2_pickup, 1

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: False
synthesizer: jtlv
fastslow: True
decompose: True

CurrentConfigName:
two_robots_local_planner

Customs: # List of custom propositions
rob1_blocked_T
rob1_blocked_B
rob1_blocked_L
rob1_blocked_R
rob1_blocked_D

rob2_blocked_T
rob2_blocked_B
rob2_blocked_L
rob2_blocked_R
rob2_blocked_D

RegionFile: # Relative path of region description file
local_planner_deadlocks_no_DO.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
rob1_pickup_ac, 1
rob2_pickup_ac, 1
rob1_deadlock, 1
rob2_deadlock, 1
garbage, 1
door, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
B = p9
D = p8
L = p7
R = p6
T = p5

Spec: # Specification in structured English
visit T

