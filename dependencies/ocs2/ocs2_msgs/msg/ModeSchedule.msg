# MPC mode sequence

float64[] event_times          # event times: its size is equal to the size of mode_sequence minus one
int8[]    mode_sequence        # mode sequence: e.g., for a quadrupedal robot, it is in the set {0, 1,..., 15}
