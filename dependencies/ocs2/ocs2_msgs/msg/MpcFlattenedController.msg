# Flattened controller: A serialized controller

# Define controllerType Enum values
uint8 CONTROLLER_UNKNOWN = 0  # Safety mechanism: message initialization to zero
uint8 CONTROLLER_FEEDFORWARD = 1
uint8 CONTROLLER_LINEAR = 2

uint8                   controller_type          # What type of controller is this

MpcObservation         init_observation         # Plan initial observation

MpcTargetTrajectories plan_target_trajectories # Target trajectory in cost function
MpcState[]             state_trajectory         # Optimized state trajectory from planner
MpcInput[]             input_trajectory         # Optimized input trajectory from planner
float64[]               time_trajectory          # Time trajectory for state_trajectory and input_trajectory
uint16[]                post_event_indices       # Array of indices indicating the index of post-event time in the trajectories
ModeSchedule           mode_schedule            # Optimal/predefined MPC mode sequence and event times

ControllerData[]       data                     # The actual payload from flatten method: one vector of data per time step

MpcPerformanceIndices performance_indices      # Solver performance indices
