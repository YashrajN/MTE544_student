from mapUtilities import *
from a_star import *
from probabilistic_road_map import *
import time
from utilities import Logger

POINT_PLANNER=0; TRAJECTORY_PLANNER=1; ASTAR_PLANNER=2; PRM_PLANNER=3

class planner:
    def __init__(self, type_, mapName="room", loggerName="planned_path.csv", loggerHeaders=["x", "y"]):

        self.type=type_
        self.mapName=mapName
        # Uses logger class to store planned path points in csv
        self.path_logger = Logger(loggerName, loggerHeaders)

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)

        self.costMap=None
        self.initTrajectoryPlanner()
        
        return self.trajectory_planner(startPose, endPose, self.type)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):
        
        #### If using the map, you can leverage on the code below originally implemented for A*
        self.m_utilities=mapManipulator(laser_sig=0.4)    
        self.costMap=self.m_utilities.make_likelihood_field()

        # List of obstacles to plot (in x and y coordiantes)
        self.obstaclesList = np.array(self.m_utilities.getAllObstacles())     

        # List of obstacles for PRM, in cell indices
        self.obstaclesListCell = np.array(self.m_utilities.getAllObstaclesCell())  

        
    def trajectory_planner(self, startPoseCart, endPoseCart, type):

        startPose=self.m_utilities.position_2_cell(startPoseCart)
        endPose=self.m_utilities.position_2_cell(endPoseCart)

        # [Part 3] TODO Use the PRM and search_PRM to generate the path
        # Hint: see the example of the ASTAR case below, there is no scaling factor for PRM
        if type == PRM_PLANNER:

            # Define robot's radius as per piazza
            robot_radius = 0.2  

            # Generate PRM graph using probabilistic roadmap
            sample_points, prm_roadmap = prm_graph(
                start=startPose,
                goal=endPose,
                obstacles_list=self.obstaclesListCell,
                robot_radius=robot_radius,
                rng=None,
                m_utilities=self.m_utilities
            )

            # Start the time to keep track of how long it takes to generate path using PRM
            start_time = time.time()
            
            # Search for a path in the PRM using search_PRM
            path_ = search_PRM(sample_points, prm_roadmap, startPose, endPose)

            # Ends time to see how long PRM took
            end_time = time.time()
            print(f"The time took for PRM pathfinding was {end_time - start_time}")

        elif type == ASTAR_PLANNER: # This is the same planner you should have implemented for Lab4
            scale_factor = 4 # Depending on resolution, this can be smaller or larger
            startPose = [int(i/scale_factor) for i in startPose]
            endPose   = [int(j/scale_factor) for j in endPose]
            start_time = time.time()

            path = search(self.costMap, startPose, endPose, scale_factor)

            end_time = time.time()


            print(f"the time took for a_star calculation was {end_time - start_time}")

            path_ = [[x*scale_factor, y*scale_factor] for x,y in path ]

        Path = np.array(list(map(self.m_utilities.cell_2_position, path_ )))

        # Log the generated path
        for point in Path:
            self.path_logger.log_values([point[0], point[1]])

        # Plot the generated path
        plt.plot(self.obstaclesList[:,0], self.obstaclesList[:,1], '.')
        plt.plot(Path[:,0], Path[:,1], '-*')
        plt.plot(startPoseCart[0],startPoseCart[1],'*')
        plt.plot(endPoseCart[0],
                 endPoseCart[1], '*')

        plt.show()
        
        return Path.tolist()
    

if __name__=="__main__":

    m_utilities=mapManipulator()
    
    map_likelihood=m_utilities.make_likelihood_field()
    
    # You can test your code here...
