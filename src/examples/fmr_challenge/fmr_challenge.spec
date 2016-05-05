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
fmr_challenge

Customs: # List of custom propositions
eagent1_wait

RegionFile: # Relative path of region description file
fmr_challenge.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
A = p1
B = p2
C = p3
D = p4
E = p5
E_C = p7
E_D = p6

Spec: # Specification in structured English
visit s1a

