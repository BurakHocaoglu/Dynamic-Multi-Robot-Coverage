README:
#######



Required packages:
------------------
- rospy
- numpy
- matplotlib
- scikit-geometry (May require CGAL to be built on the machine)
- scipy
- shapely



How to run:
-----------

- Create a ros workspace:
	- "mkdir -p ~/coverage_ws/src"
	- "cd ~/coverage_ws"
	- "catkin build --make-args -j6"


- After having built, do not forget:

	- "source ~/coverage_ws/devel/setup.bash"

	(Assuming coverage_ws is located at the HOME directory.)


- Make sure to have a 

	"params.yaml" 

  file under 

	"~/coverage_ws/src/coverage_control2/cfg"


- roslaunch coverage_control2 sync_coverage.launch count:=<Number of agents to run with>
	(After some initial setups, the processes should be spawned and the experiment should start.)
