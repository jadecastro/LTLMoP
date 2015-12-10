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
box_pushing_youbot

Customs: # List of custom propositions
pushingBox
m_rob1_B

RegionFile: # Relative path of region description file
box_pushing_new.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
moveToLeft, 1
moveToRight, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
A = p1
C = p3
B = p2
E = p5
D = p4
E_C = p7
E_D = p6

Spec: # Specification in structured English
# Please see box_pushing_new.structuredslugs for spec
visit A

