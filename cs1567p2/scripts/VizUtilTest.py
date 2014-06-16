from VizUtil import *


print("Expected: (0.0, 0.0)")
print("Actual: ", midpoint_2d([Point(-1, -1), Point(1, 1), Point(-1, 1), Point(1, -1)]))

print("Expected: True")
print("Actual: ", color_match(Color(0, 0, 0, 1), Color(19,19,19,1)))

print("Expected: False")
print("Actual: ", color_match(Color(0, 0, 0, 1), Color(21,21,21,1)))

# test other vizutil functions here