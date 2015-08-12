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
light_painting

Customs: # List of custom propositions
m_rob1_F
m_rob1_D_H

RegionFile: # Relative path of region description file
light_painting.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
letterE, 1
letterH, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
D = p1
F = p2
H = p3
B = p4
I = p5
E = p6
G = p7
C = p8
A = p9

Spec: # Specification in structured English
visit A

