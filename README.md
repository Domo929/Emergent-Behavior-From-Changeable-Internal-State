# Emergent Behavior From Internal States

## Dominic Cupo, Joshua Bloom

###Todo
* Experiment Analysis
  1. Fix Group Rotation to match Brown's implementation 
  2. Add Angular Momentum 
  3. Centroid calculation
  4. Average State Duration
     * Number for how long it was in State 0, same for State 1
     * Calculate average per robot, then average for the swarm, return swarm level
  5. Calculate 5 spacial measurements but for members of each state
     * This gives us 3 values per spatial measurement: total swarm, swarm state 0, swarm state 1
  6. Drop distance calculation (basically just Scatter calc)
   
* General fixes
  1. Remove `best_##.dat` calculations from main. We don't care
  2. Make sure to update names and outputs when the analysis stuff is added
  3. Add LEDs to describe internal state in Buzz
  
* Final Analysis 
  1. Implement Bootstrapping
  2. Improved data visualization    