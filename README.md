Current features:
  - INS, using a Kalman filter (as found in a real system) to calculate position and velocity, including a simulation of system noise and uncertainty in measurements.  // largely complete; requires substantial bug-fixing and code cleanup
    - taking as input:
      - Angular velocity in pitch, roll and yaw, measured by three Ring Laser Gyroscopes.
      - Acceleration in x, y and z axes from three accelerometers
      - GNSS
      - Barometric altitude
      - IAS
 - Navigation Computer:  // WIP
  - interface between nav system and cockpit displays. If it is not possible to nodify existing solutions for TACAN/RSBN and ILS (e.g. from A-4 or A-29), will also calculate radio navigation data 
  - calculates CAS from IAS input
  - stores waypoints
  - stores current ILS frequency
  - stores current TACAN frequency
- Physical simulations of GNSS system, Laser Ring Gyroscopes and Accelerometers.  // largely complete, each class requires some minor refactoring to conform with improved Kalman filter implementation

- TODO:
  - implement Lua API to pass data back and forth with cockpit instruments
  - ascertain if it is possible to interface directly with EFM, avoiding the computational expense of passing position/orientation data via Lua environment
