# multi_mbes_sim
A ROS package to simulate data from a Multibeam Echosounder.

This package is under construction and not fully functional.

The multi_mbes_sim node uses a bathymetric grid in the form of a geotif to serve up "pings" of data to simulate that from a swath mapping multibeam echosounder. One can request a new MBES, and in doing so provide information for where the MBES can subscribe for vessel position and orientation information. Then an operator can send a "START_PINGING" command, after which the simulator will begin publishing of PointCloud2 messages with data popualted from the grid as one might expect from a MBES. 

There is much more to be done:

  * TF is not used in any way, so the simulation knows nothing of vessel pitch and roll.
  * There is no modeling for refraction.
  * The implementation has not been tested for performance with multiple MBES simulated at high ping rates.
  * The code desparately needs cleaning up. 


