uint32    cloud_idx      # What is the index of the feature in the original cloud?
uint32    segment_idx    # What was the segment in which the feature originally lied in?

bool      selected       # Was the feature selected with feature selection?
bool      associated     # Was the feature associated into the map?
bool      standalone     # Was the feature standalone (note: for heuristic_gradient only)

float32   value          # What was the feature's value (e.g., gradient)?
float32[] eigenvalues    # What are the feature's eigenvalues w.r.t. xyzrpy in sensor frame?
