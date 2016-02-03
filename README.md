# motion planning methods 

Documentation is available as a pdf. 


Examples of code running are available at http://www.cs.dartmouth.edu/~pritikav/datavis.html
*Note on harder environments, code may take a minute or more to find path.
This code assumes Java 8 and the ability to run java applications.*

To run, be sure to include the environment maps on the toplevel of directory. 

The problem set up in the driver is default to the PRM map. Other cases can be run by switching the following variables. 




To run the PRM:
    PlANAR_ROBOT in the driver to false
    Set the defaultSize to 1000

To run the RRT:
    PLANAR_ROBOT in the driver to true
    set the defaultSize to 20000

To run the modified RRT:
    Follow instructions for RRT
    Set the defaultSize to 50000
    Set the variable ALTER_RRT in RRTPlanner.java to true. 