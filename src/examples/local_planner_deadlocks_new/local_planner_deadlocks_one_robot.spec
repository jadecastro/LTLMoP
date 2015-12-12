# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: False
synthesizer: jtlv
fastslow: True
decompose: True

CurrentConfigName:
one_robot_local_planner

Customs: # List of custom propositions
m_rob1_deadlock1
m_rob1_deadlock2
m_rob1_deadlock3

RegionFile: # Relative path of region description file
local_planner_deadlocks_one_robot.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
rob1_deadlock, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
B = p8
D = p7
L = p6
R = p5
T = p4
others = p1, p2, p9, p10

Spec: # Specification in structured English
visit T

