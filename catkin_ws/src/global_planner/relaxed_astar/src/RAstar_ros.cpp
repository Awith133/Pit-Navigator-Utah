/* iPath: A C++ Library of Intelligent Global Path Planners for Mobile Robots with ROS Integration. 

 * Website: http://www.iroboapp.org/index.php?title=IPath
 * Contact: 
 *
 * Copyright (c) 2014
 * Owners: Al-Imam University/King AbdulAziz Center for Science and Technology (KACST)/Prince Sultan University
 * All rights reserved.
 *
 * License Type: GNU GPL
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>

#include <math.h>




#include "RAstar_ros.h"
#include <pluginlib/class_list_macros.h>
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(RAstar_planner::RAstarPlannerROS, nav_core::BaseGlobalPlanner)
int prevCell = -1;
int tackNum = 0;
bool tackLeft = true;
double sun_orientation[3] ={0.0,0.0,0.0};
int tackSize = 0;
//sliders
double solarCostSlider = 2;
double sunCostSlider = .5;
double tackCostSlider = .5;
double obstacleCostSlider = 2;





int value;
int mapSize;
double* OGM;
static const float INFINIT_COST = INT_MAX; //!< cost of non connected nodes
float infinity = std::numeric_limits< float >::infinity();
float tBreak;  // coefficient for breaking ties
ofstream MyExcelFile ("RA_result.xlsx", ios::trunc);

int clock_gettime(clockid_t clk_id, struct timespect *tp);

timespec diff(timespec start, timespec end)
{
  timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

inline vector <int> findFreeNeighborCell (int CellID);






namespace RAstar_planner
{

//Default Constructor
RAstarPlannerROS::RAstarPlannerROS()
{

}
RAstarPlannerROS::RAstarPlannerROS(ros::NodeHandle &nh)
{
  ROSNodeHandle = nh;

}

RAstarPlannerROS::RAstarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}


/** overriden method   */
void RAstarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{

  if (!initialized_)
  {
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    ros::NodeHandle private_nh("~/" + name);

    originX = costmap_->getOriginX();
    originY = costmap_->getOriginY();



	width = costmap_->getSizeInCellsX();
	height = costmap_->getSizeInCellsY();
	resolution = costmap_->getResolution();
	mapSize = width*height;
	tBreak = 1+1/(mapSize); 
	value =0;

  
	OGM = new double [mapSize]; 
    for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
    {
      for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
      {
        double cost = static_cast<double>(costmap_->getCost(ix, iy))/255.0;
        //cout<<cost;                                                              //RIGHT HERE I CHANGED THE FREE VALUE FROM 0 TO .5
        if (cost >= 0.95)
          OGM[iy*width+ix]=1;
        else
          OGM[iy*width+ix]=cost;
      }
    }


	MyExcelFile << "StartID\tStartX\tStartY\tGoalID\tGoalX\tGoalY\tPlannertime(ms)\tpathLength\tnumberOfCells\t" << endl;

    ROS_INFO("RAstar planner initialized successfully");
    initialized_ = true;
  }
  else
    ROS_WARN("This planner has already been initialized... doing nothing");
}




/** overrriden method   */
bool RAstarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan)
{
  
  if (!initialized_)
  {
    ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
    return false;
  }

  ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
            goal.pose.position.x, goal.pose.position.y);

  plan.clear();

  if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
  {
    ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
              costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
    return false;
  }

  tf::Stamped < tf::Pose > goal_tf;
  tf::Stamped < tf::Pose > start_tf;

  poseStampedMsgToTF(goal, goal_tf);
  poseStampedMsgToTF(start, start_tf);

  // convert the start and goal positions

  float startX = start.pose.position.x;
  float startY = start.pose.position.y;

  float goalX = goal.pose.position.x;
  float goalY = goal.pose.position.y;
  float goalOrientation[4] = {goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w};

  getCorrdinate(startX, startY);
  getCorrdinate(goalX, goalY);

  int startCell;
  int goalCell;

  if (isCellInsideMap(startX, startY) && isCellInsideMap(goalX, goalY))
  {
    startCell = convertToCellIndex(startX, startY);

    goalCell = convertToCellIndex(goalX, goalY);

MyExcelFile << startCell <<"\t"<< start.pose.position.x <<"\t"<< start.pose.position.y <<"\t"<< goalCell <<"\t"<< goal.pose.position.x <<"\t"<< goal.pose.position.y;

  }
  else
  {
    ROS_WARN("the start or goal is out of the map");
    return false;
  }

  /////////////////////////////////////////////////////////

  // call global planner

  if (isStartAndGoalCellsValid(startCell, goalCell)){

        vector<int> bestPath;
	bestPath.clear();

    bestPath = RAstarPlanner(startCell, goalCell); // calling the planner

//if the global planner find a path
    if ( bestPath.size()>0)
    {

// convert the path

      for (int i = 0; i < bestPath.size(); i++)
      {

        float x = 0.0;
        float y = 0.0;

        int index = bestPath[i];

        convertToCoordinate(index, x, y);

        geometry_msgs::PoseStamped pose = goal;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = goalOrientation[0];
        pose.pose.orientation.y = goalOrientation[1];
        pose.pose.orientation.z = goalOrientation[2];
        pose.pose.orientation.w = goalOrientation[3];

        plan.push_back(pose);                               //path is stored here?
      }


	float path_length = 0.0;
	
	std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
	
	geometry_msgs::PoseStamped last_pose;
	last_pose = *it;
	it++;
	for (; it!=plan.end(); ++it) {
	   path_length += hypot(  (*it).pose.position.x - last_pose.pose.position.x, 
		                 (*it).pose.position.y - last_pose.pose.position.y );
	   last_pose = *it;
	}
	cout <<"The global path length: "<< path_length<< " meters"<<endl;
	MyExcelFile << "\t" <<path_length <<"\t"<< plan.size() <<endl;
      //publish the plan

      return true;

    }

    else
    {
      ROS_WARN("The planner failed to find a path, choose other goal position");
      return false;
    }

  }

  else
  {
    ROS_WARN("Not valid start or goal");
    return false;
  }

}







void RAstarPlannerROS::getCorrdinate(float& x, float& y)
{

  x = x - originX;
  y = y - originY;

}

int RAstarPlannerROS::convertToCellIndex(float x, float y)
{

  int cellIndex;

  float newX = x / resolution;
  float newY = y / resolution;

  cellIndex = getCellIndex(newY, newX);

  return cellIndex;
}

void RAstarPlannerROS::convertToCoordinate(int index, float& x, float& y)
{

  x = getCellColID(index) * resolution;

  y = getCellRowID(index) * resolution;

  x = x + originX;
  y = y + originY;

}

bool RAstarPlannerROS::isCellInsideMap(float x, float y)
{
  bool valid = true;

  if (x > (width * resolution) || y > (height * resolution))
    valid = false;

  return valid;
}

void RAstarPlannerROS::mapToWorld(double mx, double my, double& wx, double& wy){
   costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    wx = costmap->getOriginX() + mx * resolution;
    wy = costmap->getOriginY() + my * resolution;
}

vector<int> RAstarPlannerROS::RAstarPlanner(int startCell, int goalCell){

   vector<int> bestPath;


//float g_score [mapSize][2];
float g_score [mapSize];

for (uint i=0; i<mapSize; i++)
	g_score[i]=infinity;

   timespec time1, time2;
  /* take current time here */
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

  bestPath=findPath(startCell, goalCell,  g_score);

   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);


   cout<<"time to generate best global path by Relaxed A* = " << (diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 << " microseconds" << endl;
   
   MyExcelFile <<"\t"<< (diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 ;

  return bestPath;

}


/*******************************************************************************/
//Function Name: findPath
//Inputs: the map layout, the start and the goal Cells and a boolean to indicate if we will use break ties or not
//Output: the best path
//Description: it is used to generate the robot free path
/*********************************************************************************/
vector<int> RAstarPlannerROS::findPath(int startCell, int goalCell, float g_score[])
{
	value++;
	vector<int> bestPath;
	vector<int> emptyPath;
	cells CP;

	multiset<cells> OPL;
	int currentCell;

	//calculate g_score and f_score of the start position
	g_score[startCell]=0;
	CP.currentCell=startCell;
	CP.fCost=g_score[startCell]+calculateHCost(startCell,goalCell);

	//add the start cell to the open list
	OPL.insert(CP);
	currentCell=startCell;

  solarCostReset();

	//while the open list is not empty continuie the search or g_score(goalCell) is equal to infinity
	while (!OPL.empty()&& g_score[goalCell]==infinity) 
	{
		//choose the cell that has the lowest cost fCost in the open set which is the begin of the multiset
		currentCell = OPL.begin()->currentCell;
		//remove the currentCell from the openList
		OPL.erase(OPL.begin());
		//search the neighbors of the current Cell
		vector <int> neighborCells; 

		neighborCells=findFreeNeighborCell(currentCell);
    //neighborCells=sunTacking(neighborCells,currentCell);
    //neighborCells=sunFilterNeighborCells(neighborCells,currentCell);

    solarCostUpdate();

		for(uint i=0; i<neighborCells.size(); i++) //for each neighbor v of current cell
		{
			// if the g_score of the neighbor is equal to INF: unvisited cell
			if(g_score[neighborCells[i]]==infinity)
			{
				g_score[neighborCells[i]]=g_score[currentCell]+getMoveCost(currentCell,neighborCells[i]);
				addNeighborCellToOpenList(OPL, currentCell, neighborCells[i], goalCell, g_score); 
			}//end if
		}//end for
	}//end while

	if(g_score[goalCell]!=infinity)  // if g_score(goalcell)==INF : construct path 
	{
		bestPath=constructPath(startCell, goalCell, g_score);
		return   bestPath; 
	}
	else
	{
		cout << "Failure to find a path !" << endl;
		return emptyPath;
	}
}

/*******************************************************************************/
//Function Name: constructPath
//Inputs: the start and the goal Cells
//Output: the best path
//Description: it is used to construct the robot path
/*********************************************************************************/
vector<int> RAstarPlannerROS::constructPath(int startCell, int goalCell,float g_score[])
{
	vector<int> bestPath;
	vector<int> path;

	path.insert(path.begin()+bestPath.size(), goalCell);
	int currentCell=goalCell;

	while(currentCell!=startCell)
	{ 
		vector <int> neighborCells;
		neighborCells=findFreeNeighborCell(currentCell);
    //neighborCells=sunTacking(neighborCells,currentCell);
    //neighborCells=sunFilterNeighborCells(neighborCells,currentCell);

		vector <float> gScoresNeighbors;
		for(uint i=0; i<neighborCells.size(); i++)
			gScoresNeighbors.push_back(g_score[neighborCells[i]]);
		
		int posMinGScore=distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()));
		currentCell=neighborCells[posMinGScore];

		//insert the neighbor in the path
		path.insert(path.begin()+path.size(), currentCell);
	}
	for(uint i=0; i<path.size(); i++)
		bestPath.insert(bestPath.begin()+bestPath.size(), path[path.size()-(i+1)]);

	return bestPath;
}

/*******************************************************************************/
//Function Name: calculateHCost
//Inputs:the cellID and the goalCell
//Output: the distance between the current cell and the goal cell
//Description: it is used to calculate the hCost 
/*********************************************************************************/
/*
float RAstarPlannerROS::calculateHCost(int cellID, int goalCell)
{    
  int x1=getCellRowID(goalCell);
  int y1=getCellColID(goalCell);
  int x2=getCellRowID(cellID);
  int y2=getCellColID(cellID);
  
  //if(getNeighborNumber()==4) 
    //The diagonal shortcut distance between two grid points (x1,y1) and (x2,y2) is:
    //  return min(abs(x1-x2),abs(y1-y2))*sqrt(2) + max(abs(x1-x2),abs(y1-y2))-min(abs(x1-x2),abs(y1-y2));
  
  //else
    //manhatten distance for 8 neighbor
    return abs(x1-x2)+abs(y1-y2);
}
*/
/*******************************************************************************/
//Function Name: addNeighborCellToOpenList
//Inputs: the open list, the neighbors Cell, the g_score matrix, the goal cell 
//Output: 
//Description: it is used to add a neighbor Cell to the open list
/*********************************************************************************/
void RAstarPlannerROS::addNeighborCellToOpenList(multiset<cells> & OPL,int currentCell, int neighborCell, int goalCell, float g_score[])
{
	cells CP;
	CP.currentCell=neighborCell; //insert the neighbor cell
	CP.fCost=g_score[neighborCell]+calculateHCost(neighborCell,goalCell);

  double solar_cost = solarCost(currentCell, neighborCell, goalCell);    //0-1
  double obstacle_cost = obstacleCost(neighborCell);                     //0-.95 | inf
  double scale = CP.fCost;
  //ROS_WARN("%s%f : %s%f : %s%f","SCost:",solar_cost*scale*solarCostSlider, "FCost:",CP.fCost, "OCost:",obstacle_cost*scale*obstacleCostSlider);
  //uncomment
  //CP.fCost=solar_cost*scale*solarCostSlider+CP.fCost+obstacle_cost*scale*obstacleCostSlider;
  
	
  
  OPL.insert(CP);
	//multiset<cells>::iterator it = OPL.lower_bound(CP);
	//multiset<cells>::iterator it = OPL.upper_bound(CP);
	//OPL.insert( it, CP  );
}

  /*******************************************************************************
 * Function Name: findFreeNeighborCell
 * Inputs: the row and columun of the current Cell
 * Output: a vector of free neighbor cells of the current cell
 * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
 * Check Status: Checked by Anis, Imen and Sahar
*********************************************************************************/

vector <int> RAstarPlannerROS::findFreeNeighborCell (int CellID){
 
  int rowID=getCellRowID(CellID);
  int colID=getCellColID(CellID);
  int neighborIndex;
  vector <int>  freeNeighborCells;
     for (int i=-1;i<=1;i++)
      for (int j=-1; j<=1;j++){
        //check whether the index is valid
      if ((rowID+i>=0)&&(rowID+i<height)&&(colID+j>=0)&&(colID+j<width)&& (!(i==0 && j==0))){
    neighborIndex = getCellIndex(rowID+i,colID+j);
          if(isFree(neighborIndex) )
        freeNeighborCells.push_back(neighborIndex);
    }
      }
      return  freeNeighborCells;
}

  /*******************************************************************************
 * Function Name: sunFilterNeighborCells
 * Inputs: the vector of neighboring cells, the row and columun of the current Cell
 * Output: a vector of free neighbor cells of the current cell
 * Description:it is used to filter the free neighbors Cells of a the current Cell in the grid based on the direction of the sun
 * Check Status: unused
*********************************************************************************/
vector <int> RAstarPlannerROS::sunFilterNeighborCells (vector <int> neighborCells, int CellID){
  //There's an external obstacle and should abort light following
  if (neighborCells.size() < 8){
    prevCell = -1;
    return neighborCells;
  }
  double sun_orientation[3] ={0.0,0.0,0.0};
  ros::param::get("/sun_orientation/x",sun_orientation[0]);
  ros::param::get("/sun_orientation/y",sun_orientation[1]);
  ros::param::get("/sun_orientation/z",sun_orientation[2]);
  if ((abs(sun_orientation[0]) > 0.7853982 && abs(sun_orientation[0]) <= 2.3561945) && (abs(sun_orientation[0]) > 0.7853982 && abs(sun_orientation[0]) <= 2.3561945)){
    ROS_WARN("HIGH SUN");
    prevCell = -1;
    return neighborCells;
  }
  else{
    static const double arr[] = { 2.3561945, 1.5707963, 0.7853982,  3.1415927, 0 , -2.3561945, -1.5707963, -0.7853982};
    vector <double> angles (arr, arr + sizeof(arr) / sizeof(arr[0]));
    vector <int> new_neighbors;
    for(int i = 0; i != neighborCells.size(); i++){
      if (!((angles[i]+0.3926991>sun_orientation[2]+1.5707963&&angles[i]-0.3926991<sun_orientation[2]+1.5707963) || (angles[i]+0.3926991>sun_orientation[2]-1.5707963&&angles[i]-0.3926991<sun_orientation[2]-1.5707963))){
        new_neighbors.push_back(neighborCells[i]);
        
      }
    }
    prevCell = CellID;
    return new_neighbors;
  }
}

/*******************************************************************************
 * Function Name: sunTacking
 * Inputs: the vector of neighboring cells, the row and columun of the current Cell
 * Output: a vector of free neighbor cells of the current cell
 * Description:it is used to filter the free neighbors Cells of a the current Cell in the grid based on the direction of the sun
 * Check Status: unused
*********************************************************************************/
vector <int> RAstarPlannerROS::sunTacking (vector <int> neighborCells, int CellID){
  //There's an external obstacle and should abort light following
  if (neighborCells.size() < 8){
    prevCell = -1;
    tackNum = 0;
    return neighborCells;
  }
  double sun_orientation[3] ={0.0,0.0,0.0};
  double tackSize=0;
  ros::param::get("/tackSize",tackSize);
  ros::param::get("/sun_orientation/x",sun_orientation[0]);
  ros::param::get("/sun_orientation/y",sun_orientation[1]);
  ros::param::get("/sun_orientation/z",sun_orientation[2]);
  if ((abs(sun_orientation[0]) > 0.7853982 && abs(sun_orientation[0]) <= 2.3561945) && (abs(sun_orientation[0]) > 0.7853982 && abs(sun_orientation[0]) <= 2.3561945)){
    ROS_WARN("HIGH SUN");
    prevCell = -1;
    tackNum = 0;
    return neighborCells;
  }
  else{
    static const double arr[] = { -2.3561945, -1.5707963, -0.7853982,  3.1415927, 0 , 2.3561945, 1.5707963, 0.7853982};
    vector <double> angles (arr, arr + sizeof(arr) / sizeof(arr[0]));
    vector <int> new_neighbors;
    vector <double> tackingAngles;
    int numbad = 0;
    if(tackNum < tackSize){
      for(int i = 0; i != neighborCells.size(); i++){
        if (neighborCells[i] == prevCell){
          vector <double> tempAngles;
          tempAngles.push_back(angles[i]+1.5707963);
          tempAngles.push_back(angles[i]-1.5707963);
          tempAngles.push_back(angles[i]+0.7853982);
          tempAngles.push_back(angles[i]-0.7853982);
          for (int j =0; j != tempAngles.size(); j++){
            if (tempAngles[j]>3.1415927)
              tackingAngles.push_back(tempAngles[j]-3.1415927*2);
            else if (tempAngles[j]<=-3.1415927)
              tackingAngles.push_back(tempAngles[j]+3.1415927*2);
            else
              tackingAngles.push_back(tempAngles[j]);
          }
          ROS_WARN("tacking angles: %2f, %2f, %2f, %2f",tackingAngles[0],tackingAngles[1],tackingAngles[2],tackingAngles[3]);
        }else{
          numbad+=1;
          if (numbad == 8){
            prevCell = -1;
            tackNum = 0;
            break;
          }
        }
      }
      tackNum += 1;
      ROS_WARN("Step: %2f / %2f", double(tackNum), tackSize);
    }else{
      prevCell = -1;
      tackNum = 0;
      ROS_WARN("Next tack");
    }
    
    for(int i = 0; i != neighborCells.size(); i++){
      if (prevCell != -1){
        if ((abs(angles[i] - tackingAngles[0]) <= .001 || abs(angles[i] - tackingAngles[1])<= .001 || abs(angles[i] - tackingAngles[2])<= .001 || abs(angles[i] - tackingAngles[3])<= .001)){
          ROS_WARN("Removed tacking angle: %2f",angles[i]);
          continue; // remove addition to perpendicular section
        }
      }
      if (((angles[i]+0.3926991>sun_orientation[2]+1.5707963&&angles[i]-0.3926991<sun_orientation[2]+1.5707963) || (angles[i]+0.3926991>sun_orientation[2]-1.5707963&&angles[i]-0.3926991<sun_orientation[2]-1.5707963))){
        ROS_WARN("Removed Sun angle: %2f",angles[i]);
        continue; // remove perpendicular direction
      }
      if (prevCell == neighborCells[i]){
        ROS_WARN("Removed backwards angle: %2f",angles[i]);
        continue; // remove backwards
      }

      new_neighbors.push_back(neighborCells[i]);
    }
    prevCell = CellID;
    return new_neighbors;
  }
}

/*******************************************************************************
 * Function Name: solarCost
 * Inputs: current cell id, neighbor cell id
 * Output: cost of moving that direction relative to the sun
 * Description:it is used to filter the free neighbors Cells of a the current Cell in the grid based on the direction of the sun
 * Check Status: Unchecked
*********************************************************************************/
double RAstarPlannerROS::solarCost(int CellID1, int CellID2, int goalCell)
{
  if ((abs(sun_orientation[0]) > 0.7853982 && abs(sun_orientation[0]) <= 2.3561945) && (abs(sun_orientation[0]) > 0.7853982 && abs(sun_orientation[0]) <= 2.3561945)){
    ROS_WARN("HIGH SUN");
    return 0;
  }
  int i1=0,i2=0,j1=0,j2=0,i3=0,j3=0;
  i1=getCellRowID(CellID1);
  j1=getCellColID(CellID1);
  i2=getCellRowID(CellID2);
  j2=getCellColID(CellID2);
  i3=getCellRowID(goalCell);
  j3=getCellColID(goalCell);
  bool dirToGoalThres = (abs(sun_orientation[2]-atan2(i3-i1,j3-j1)) <= 2.3561945 && abs(sun_orientation[2]-atan2(i3-i1,j3-j1)) >= 0.7853982);
  if (!dirToGoalThres){
    return 0;
  }
  // ROS_WARN("tackNum: %i",tackNum);
  if (tackLeft){
  //   if (tackNum > 0){
  //     tackNum-=1;    
      double sun = 1-abs(cos(sun_orientation[2]-atan2(i2-i1,j2-j1))); //phaseshift
      double tack = 1-abs(cos(sun_orientation[2]-atan2(i2-i1,j2-j1)   -0.7853982));
      return (sunCostSlider*sun+tackCostSlider*tack);
  //   }
  }else{
  //   if (tackNum < tackSize){
  //     tackNum+=1;  
      double sun = 1-abs(cos(sun_orientation[2]-atan2(i2-i1,j2-j1))); //phaseshift
      double tack = 1-abs(cos(sun_orientation[2]-atan2(i2-i1,j2-j1)   +0.7853982));
      return (sunCostSlider*sun+tackCostSlider*tack);
  //   }
  }
  // tackLeft = !tackLeft;
  return 1-abs(cos(sun_orientation[2]-atan2(i2-i1,j2-j1))); // remember that the id goes from top left not bottom left
}


void RAstarPlannerROS::solarCostUpdate()
{
    // get sun for each step
    ros::param::get("/tackSize",tackSize);
    ros::param::get("/sun_orientation/x",sun_orientation[0]);
    ros::param::get("/sun_orientation/y",sun_orientation[1]);
    ros::param::get("/sun_orientation/z",sun_orientation[2]);
    return;
}

void RAstarPlannerROS::solarCostReset(){
//   int tackNum = 0;
//  bool tackLeft = false;
  return;
}

/*******************************************************************************
 * Function Name: obstacleCost
 * Inputs: cell id
 * Output: cost of moving that direction relative to the sun
 * Description:it is used to filter the free neighbors Cells of a the current Cell in the grid based on the direction of the sun
 * Check Status: Unchecked
*********************************************************************************/
double RAstarPlannerROS::obstacleCost(int CellID1)
{
  double cost = OGM[CellID1];
  if (cost>=0.95){
    return infinity;
  }
  return cost;
}


/*******************************************************************************/
//Function Name: isStartAndGoalCellsValid
//Inputs: the start and Goal cells
//Output: true if the start and the goal cells are valid
//Description: check if the start and goal cells are valid
/*********************************************************************************/
bool RAstarPlannerROS::isStartAndGoalCellsValid(int startCell,int goalCell)
{ 
 bool isvalid=true;
 bool isFreeStartCell=isFree(startCell);
 bool isFreeGoalCell=isFree(goalCell);
    if (startCell==goalCell)
    {
    //cout << "The Start and the Goal cells are the same..." << endl; 
    isvalid = false;
    }
   else
   {
      if (!isFreeStartCell && !isFreeGoalCell)
      {
	//cout << "The start and the goal cells are obstacle positions..." << endl;
        isvalid = false;
      }
      else
      {
	if (!isFreeStartCell)
	{
	  //cout << "The start is an obstacle..." << endl;
	  isvalid = false;
	}
	else
	{
	    if(!isFreeGoalCell)
	    {
	      //cout << "The goal cell is an obstacle..." << endl;
	      isvalid = false;
	    }
	    else
	    {
	      if (findFreeNeighborCell(goalCell).size()==0)
	      {
		//cout << "The goal cell is encountred by obstacles... "<< endl;
		isvalid = false;
	      }
	      else
	      {
		if(findFreeNeighborCell(startCell).size()==0)
		{
		  //cout << "The start cell is encountred by obstacles... "<< endl;
		  isvalid = false;
		}
	      }
	    }
	}
      }
  }
 return isvalid;
}


 float  RAstarPlannerROS::getMoveCost(int i1, int j1, int i2, int j2){
   float moveCost=INFINIT_COST;//start cost with maximum value. Change it to real cost of cells are connected
   //if cell2(i2,j2) exists in the diagonal of cell1(i1,j1)
   
   if((j2==j1+1 && i2==i1+1)||(i2==i1-1 && j2==j1+1) ||(i2==i1-1 && j2==j1-1)||(j2==j1-1 && i2==i1+1)){
     //moveCost = DIAGONAL_MOVE_COST;
     moveCost = 1.4;
   }
    //if cell 2(i2,j2) exists in the horizontal or vertical line with cell1(i1,j1)
   else{
     if ((j2==j1 && i2==i1-1)||(i2==i1 && j2==j1-1)||(i2==i1+1 && j2==j1) ||(i1==i2 && j2==j1+1)){
       //moveCost = MOVE_COST;
       moveCost = 1;
     }
   }
   return moveCost;
 } 
 
  float  RAstarPlannerROS::getMoveCost(int CellID1, int CellID2){
   int i1=0,i2=0,j1=0,j2=0;
    
   i1=getCellRowID(CellID1);
   j1=getCellColID(CellID1);
   i2=getCellRowID(CellID2);
   j2=getCellColID(CellID2);
    
    return getMoveCost(i1, j1, i2, j2);
 } 


 //verify if the cell(i,j) is free
 bool  RAstarPlannerROS::isFree(int i, int j){
   int CellID = getCellIndex(i, j);
 return OGM[CellID]<=0.95;

 } 

  //verify if the cell(i,j) is free
 bool  RAstarPlannerROS::isFree(int CellID){
 return OGM[CellID]<=0.95;
 } 
}
;

bool operator<(cells const &c1, cells const &c2) { return c1.fCost < c2.fCost; }
