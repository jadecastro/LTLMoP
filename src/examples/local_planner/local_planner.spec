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
use_region_bit_encoding: True
synthesizer: False
fastslow: False
decompose: True

CurrentConfigName:
two_robots_local_planner

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
local_planner.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
garbage, 1


======== SPECIFICATION ========

Spec: # Specification in structured English
D

