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
box_pushing_holonomic

Customs: # List of custom propositions
pushingBox
m_rob1_B

RegionFile: # Relative path of region description file
box_pushing_holonomic.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
moveToLeft, 1
moveToRight, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
A = p9
C = p7
B = p8
E = p4, p5, p12, p13, p14, p15, p16
D = p6
E_D = p4
E_C = p5
others = p10, p11, p17, p18

Spec: # Specification in structured English
# Please see box_pushing_holonomic.structuredslugs for spec
visit A

