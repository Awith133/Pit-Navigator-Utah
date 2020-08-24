#include "rasmEval.h"
#include <stdio.h>
#include <assert.h>
#include <string.h> // memset
#include <util.h>
#include <config.h>

bool g_debug_planner = false;


/*
 * These are global so they can be overridden, for example in main().
 */
unsigned int TMAP::numAxleChecks = 10; /* check this many points between the wheels for clearance */
float TMAP::wheelDist = 0.0;   // read in config file
float TMAP::axleHeight = 0.0;  // read in config file
float TMAP::roverLength = 0.0; // read in config file

/* don't allow roll/pitch over this amount (in radians) */
float TMAP::dangerousRoll = 0*M_PI/180.0;  // read in config file
float TMAP::dangerousPitch = 0*M_PI/180.0; // read in config file

/* roll/pitch under this amount are treated as flat (in radians) */
float TMAP::safeRoll = 0*M_PI/180.0;  // read in config file
float TMAP::safePitch = 0*M_PI/180.0; // read in config file

/* arc evaluation = arc cost * arcToPathFactor + path cost */
float TMAP::arcToPathFactor = 1.5;

/* whether or not intersection tests should be verbose (for debugging) */
bool TMAP::verboseIntersection = false;


// double g_pose_timeout_sec = 0.0;
// double g_sensor_data_timeout_sec = 0.0;

/* Path generation parameters */
double RASM::g_path_length_meters=1.5; //3.5;
double RASM::g_path_resolution_meters=0.05;

//tweaks to path evaluation
double RASM::g_reverse_path_penalty = 1.5;
double RASM::g_last_path_bonus = 1.0;//0.99;

double RASM::g_path_max_heading_change_rad = 20.0*(M_PI/180);
double RASM::g_path_heading_resolution_rad =  1.0*(M_PI/180);

#if ENABLE_DYNAMIC_OBSTACLES
double RASM::g_dynamic_obstacle_time_sec = 3.0;/* predict motion for this long (sec) */
#endif

double RASM::g_max_speed_meters_per_sec = 0.0;
double RASM::g_path_duration_sec = 2.0;
double RASM::g_reverse_path_speed_penalty = 0.75;

// bool   RASM::g_execute_point_to_goal_maneuver = false;
// double RASM::g_point_to_goal_radius_meters = 0.0;
// double RASM::g_point_to_goal_speed_meters_per_sec = 0.0;
// double RASM::g_point_to_goal_drive_cmd_timeout_sec = 0.0;
// double RASM::g_point_to_goal_timeout_sec = 0.0;
// double RASM::g_pointing_threshold_rad = 0.0;

double RASM::g_far_weight = 1.0; // > 1.0 to enhance effect of far-eval cost



/* given point p in triangle tri
 * if moving directly to a neighbor is better,
 * then adjust the path and to center costs appropriately
 * if either tri or the existing pathCost is invalid, do nothing
 * returns tri if no change was made
 * if an improvement is made, returns the index of the neighboring triangle
 */
static int adjustFirstStep(const RASM::point3d &p, int tri,
			   TMAP::tmap &model,
			   const TMAP::obstacleMAP &obstacles,
			   float &pathCost, float &toCenterCost){
  if((tri < 0) || (pathCost < 0.0))return tri;
  RASM::triangle t = model.getTriangle(tri);

  /* get the cost of the path and the cost of moving to the start
   * (for each valid neighbor)
   */
  float neighborPath[3] = {-1.0, -1.0, -1.0};
  float toCenter[3] = {-1.0, -1.0, -1.0};
  for(int i=0;i<3;i++){
    int neighborTri = t.neighbors[i];
    if(neighborTri<0)continue;
    neighborPath[i] = model.readpathcost(neighborTri, obstacles);
    if(neighborPath[i] < 0)continue;
    toCenter[i] = model.costToTriangleCenter(p, (unsigned int)neighborTri);
  }

  /* make improvements if possible */
  int ret = tri;
  for(int i=0;i<3;i++){
    if(neighborPath[i] < 0 || toCenter[i] < 0)continue;
    if(pathCost + toCenterCost <= neighborPath[i] + toCenter[i])continue;
    pathCost = neighborPath[i];
    toCenterCost = toCenter[i];
    ret = t.neighbors[i];
  }
  return ret;
}


/* helper gives the distance (in meters) to the goal */
double RASM::eval::dist_to_goal_meters() const
{
  //TODO: Should this compute distance to nearest bound of the goal region?
  return RASM::dist2d(m_goal.m_position, 
		      RASM::point2d(m_rover_pose.getPosition()));
}


/* helper gives the angle (in radians) to the goal */
double RASM::eval::yaw_to_goal_rad() const
{
  RASM::point2d delta = m_goal.m_position - RASM::point2d(m_rover_pose.getPosition());
  return atan2(RASM_TO_METERS(delta.Y()), RASM_TO_METERS(delta.X())) - M_PI/2.0;
}


/* helper prints goal information to stdout */
void RASM::eval::print_goal_info() const
{
  double gx = RASM_TO_METERS(m_goal.m_position.X());
  double gy = RASM_TO_METERS(m_goal.m_position.Y());
  double rx = RASM_TO_METERS(RASM::point2d(m_rover_pose.getPosition()).X());
  double ry = RASM_TO_METERS(RASM::point2d(m_rover_pose.getPosition()).Y());
	
  printf("[%s:%d %f] Goal at (%3.2f,%3.2f) radius (%3.2f[%3.2f]) orientation %3.2f degrees\n", 
	 __FILE__, __LINE__, now(),
	 gx, gy,
	 m_goal.m_semimajor_axis,
	 m_goal.m_semiminor_axis,
	 m_goal.m_orientation_radians * 180.0/M_PI);
	
  printf("[%s:%d %f] Rover at (%3.2f,%3.2f) yaw %3.2f degrees\n", 
	 __FILE__, __LINE__, now(),
	 rx, 
	 ry,
	 m_rover_pose.getOrientation().yaw * 180.0/M_PI);
	
  printf("[%s:%d %f] Need to travel %0.1f E and %0.1f N (%0.1f m) and turn %0.1f degrees\n",
	 __FILE__, __LINE__, now(),
	 gx - rx, 
	 gy - ry, 
	 dist_to_goal_meters(), 
	 yaw_to_goal_rad() * 180.0/M_PI);
  //TODO Distance to goal ellipse depends upon orientation
}



/* 
 * eval::at_goal ()
 *
 * Checks if the vehicle has reached the goal
 * Sends an atGoal and returns true if the rover is within the goal ellipse
 *
 */

bool RASM::eval::at_goal()
{
  RASM::point3d pos = m_rover_pose.getPosition();

  bool within_goal_region = m_goal.contains(pos);

  print_goal_info();
	
  if(within_goal_region)
    {	
      m_interface->send_at_goal_notification(true);
      m_interface->send_stop_cmd();
    }
	
  return within_goal_region;
} // at_goal

/* checks for a new world model and updates it
 * aborts the goal and returns false if the world model is stale
 */
bool RASM::eval::update_world_model()
{
  assert(NULL != m_interface);

  printf("[%s:%d %f] %d %f %d %f %d %f\n",
	 __FILE__, __LINE__, now(),
	 m_interface->m_have_new_map[RASM::MapType::WORLD],
	 m_interface->m_map_timestamps[RASM::MapType::WORLD],
	 m_interface->m_have_new_map[RASM::MapType::ARC],
	 m_interface->m_map_timestamps[RASM::MapType::ARC],
	 m_interface->m_have_new_map[RASM::MapType::PATH],
	 m_interface->m_map_timestamps[RASM::MapType::PATH]);

  if(m_interface->have_new_map(RASM::MapType::WORLD))
    {		
      printf("[%s:%d %f] Got new world model with time %f (age %f sec)\n",
	     __FILE__, __LINE__, now(),
	     m_interface->m_map_timestamps[RASM::MapType::WORLD],
	     now() - m_interface->m_map_timestamps[RASM::MapType::WORLD]);

      /* get new data
       * (this clears old data and trusts the triangulation
       * from the incoming message)
       */
      RASM::mesh recvd_mesh;
      m_interface->get_map(RASM::MapType::WORLD, 
			   m_world_sequence_number,
			   recvd_mesh,
			   m_world_model_timestamp);
      m_world_model.importMesh(recvd_mesh, 
			       /*do_rebuild=*/true);
		
      if(-1 == m_world_model.findTriangle2d(m_goal.m_position))
	{
	  printf("updateWorldModel(): goal not in world model...\n");
	  printf("[%s:%d %f] goal not in world model...stopping robot\n",
		 __FILE__, __LINE__, now());	  
	  m_interface->send_stop_cmd();
	  return false; // well, it's not really a catastrophic error, world model still OK
	  //	  add_goal_to_world_model();
	}
		
      /* TBD: it may not be a good idea to simply set the world-
       * model's goal equal to this->m_goal_pos, since on init it
       * could be (0,0). but for the time being do it.
       */
      m_world_model.setGoal(m_goal.m_position);
		
      return true;
    }

  return false;
}


/* waits for a goal to arrive and accepts it
 */
void RASM::eval::get_first_goal()
{
  assert(NULL != m_interface);
	
  printf("[%s:%d %f] Waiting for goal...\n",
	 __FILE__, __LINE__, now());
	
  m_interface->wait_for_goal();
	
  if(!m_interface->update_goal(m_goal))
    {
      printf("[%s:%d %f] Error, updateGoal() failed on first goal\n",
	     __FILE__, __LINE__, now());
      assert(0); // TBD: why this assert? 
    }
	
  m_interface->update_pose(m_rover_pose);
  m_interface->accept_goal(/*goal_is_valid=*/true);
  print_goal_info();
}


/* checks if a new goal has been issued and accepts it
 * returns true if the goal has changed
 */
bool RASM::eval::check_for_new_goal()
{
  assert(NULL != m_interface);
	
  //TODO Need major and minor axes
  if(!m_interface->update_goal(m_goal)) return false;
	
  printf("[%s:%d %f] Warning, got a new goal while running... clearing old one\n",
	 __FILE__, __LINE__, now());
	
  m_interface->send_at_goal_notification(/*success=*/false);
	
  m_interface->update_pose(m_rover_pose);
  m_interface->accept_goal(/*goal_is_valid=*/true);
	
  print_goal_info();
	
  return true;
}


double evaluate_far(TMAP::tmap& terrain,
		    PathModel* path_model,
		    unsigned int path_index,
		    TMAP::obstacleMAP& obstacles,
		    RASM::goal& g)
{
  assert(NULL != path_model);
  assert(path_index <= path_model->m_path_set_size);
  assert(path_model->m_num_path_points[path_index] > 0);

  if(g_debug_planner)
    {
      printf("[%s:%d %f] Evaluating far path index %d\n", __FILE__, __LINE__, now(), path_index); fflush(stdout);
    }

  uint32_t end_of_path = path_model->m_num_path_points[path_index]-1;

  double traversability =  (RASM::dist2d(g.m_position, RASM::point2d(path_model->m_front_left_wheel_paths[path_index][end_of_path])) + 
			    RASM::dist2d(g.m_position, RASM::point2d(path_model->m_front_right_wheel_paths[path_index][end_of_path])) + 
			    RASM::dist2d(g.m_position, RASM::point2d(path_model->m_rear_left_wheel_paths[path_index][end_of_path])) + 
			    RASM::dist2d(g.m_position, RASM::point2d(path_model->m_rear_right_wheel_paths[path_index][end_of_path]))) / 4.0;
  
  
#if 0
    {

      float pathA = -1, pathB = -1, toCenterA = -1, toCenterB = -1, pathC = -1, pathD = -1, toCenterC = -1, toCenterD = -1;
      /* get the cost from the wheel to the center of its triangle */

      int triA = terrain.getContainingTriangle(path_model->m_front_left_wheel_paths[path_index][end_of_path],  toCenterA);
      int triB = terrain.getContainingTriangle(path_model->m_front_right_wheel_paths[path_index][end_of_path], toCenterB);
      int triC = terrain.getContainingTriangle(path_model->m_rear_left_wheel_paths[path_index][end_of_path],   toCenterC);
      int triD = terrain.getContainingTriangle(path_model->m_rear_right_wheel_paths[path_index][end_of_path],  toCenterD);
      /* get the cost from that triangle to the goal */
      if(triA >= 0) pathA = terrain.readpathcost(triA, obstacles);
      if(triB >= 0) pathB = terrain.readpathcost(triB, obstacles);
      if(triC >= 0) pathC = terrain.readpathcost(triC, obstacles);
      if(triD >= 0) pathD = terrain.readpathcost(triD, obstacles);

      /* go straight to the neighbor if that's cheaper */
      triA = adjustFirstStep(path_model->m_front_left_wheel_paths[path_index][end_of_path], triA,
			     terrain, obstacles,
			     pathA, toCenterA);
      triB = adjustFirstStep(path_model->m_front_right_wheel_paths[path_index][end_of_path], triB,
			     terrain, obstacles,
			     pathB, toCenterB);
      triC = adjustFirstStep(path_model->m_rear_left_wheel_paths[path_index][end_of_path], triC,
			     terrain, obstacles,
			     pathC, toCenterC);
      triD = adjustFirstStep(path_model->m_rear_right_wheel_paths[path_index][end_of_path], triD,
			     terrain, obstacles,
			     pathD, toCenterD);

      /* one side unreachable */
      if((pathA<0) || (pathB<0) || (pathC<0) || (pathD<0))return -1;
  
      double traversability = (toCenterA + toCenterB + pathA + pathB + toCenterC + toCenterD + pathC + pathD) / 4.0;
    }
#endif //0 

  if(g_debug_planner)
    {
      printf("[%s:%d %f] path index %d has far traversability score %f\n", 
	     __FILE__, __LINE__, now(),
	     path_index, traversability);
    }

  /* average the costs */
  return traversability;

} // evaluate_far()


/*
 * evaluate_near performs traversability simulation over a path.
 * returns a floating-point cost of the path (cost units are meaningless).
 */
double evaluate_near(TMAP::tmap& terrain,
		     PathModel* path_model,
		     unsigned int path_index,
		     TMAP::obstacleMAP& obstacles,
		     RASM::goal& g,
		     bool& path_reaches_goal)
{
  assert(NULL != path_model);
  assert(path_index <= path_model->m_path_set_size);
  assert(path_model->m_num_path_points[path_index] > 0);

  path_reaches_goal = false;

  if(g_debug_planner)
    {
      printf("[%s:%d %f] Evaluating near path index %d\n", __FILE__, __LINE__, now(), path_index); fflush(stdout);
    }

  /*
   * Iterate through each element of the wheel paths and calculate
   * traversability cost.
   */
  double traversability = 0;

  RASM::point3d frontA, frontB, rearA, rearB;
  int a_tri=0, b_tri=0, tailA_tri=0, tailB_tri=0;
  
  for(uint32_t i=0; i < path_model->m_num_path_points[path_index]; i++)
    {
      /*
       * First, check to see if this path enters the goal region. 
       * If so, break out of the traversability-analysis loop. This will
       * lower the path's cost, thus making it more likely to be chosen.
       */
      if(g.contains(path_model->m_front_left_wheel_paths[path_index][i])  &&
	 g.contains(path_model->m_front_right_wheel_paths[path_index][i]) &&
	 g.contains(path_model->m_rear_left_wheel_paths[path_index][i])   &&
	 g.contains(path_model->m_rear_right_wheel_paths[path_index][i]))
	{
	  if(g_debug_planner) printf("[%s:%d %f] all four wheel paths enter goal region at index %u\n", __FILE__, __LINE__, now(), i);
	  path_reaches_goal = true;
	  break;
	}


      // find wheels on the model
      assert((-1) != terrain.findSurfaceAt(path_model->m_front_left_wheel_paths[path_index][i],
					   frontA, a_tri));
      assert((-1) != terrain.findSurfaceAt(path_model->m_front_right_wheel_paths[path_index][i],
					   frontB, b_tri));
      assert((-1) != terrain.findSurfaceAt(path_model->m_rear_left_wheel_paths[path_index][i],
					   rearA, tailA_tri));
      assert((-1) != terrain.findSurfaceAt(path_model->m_rear_right_wheel_paths[path_index][i],
					   rearB, tailB_tri));
      
      // save the 3D locations of the wheel points
      path_model->m_front_left_wheel_paths[path_index][i].coord3d[2]  = frontA.Z();
      path_model->m_front_right_wheel_paths[path_index][i].coord3d[2] = frontB.Z();
      path_model->m_rear_left_wheel_paths[path_index][i].coord3d[2]   = rearA.Z();
      path_model->m_rear_right_wheel_paths[path_index][i].coord3d[2]  = rearB.Z();

      // save copies of the triangle indices for use below
      int lrIndex = tailA_tri;
      int rrIndex = tailB_tri;
      frontA.coord3d[2] += METERS_TO_RASM(TMAP::axleHeight);
      frontB.coord3d[2] += METERS_TO_RASM(TMAP::axleHeight);
      rearA.coord3d[2]  += METERS_TO_RASM(TMAP::axleHeight);
      rearB.coord3d[2]  += METERS_TO_RASM(TMAP::axleHeight);

      // get the front and rear axle pivots 
      RASM::point3d frontPivot = frontA + frontB;
      RASM::point3d rearPivot  = rearA + rearB;
      frontPivot /= 2;
      rearPivot  /= 2;
      
      // get the roll angle 
      RASM::point3d frontD = frontA - frontB;
      float rise = RASM_TO_METERS(frontD.Z());
      float run = sqrt(RASM_TO_METERS(frontD.X())*RASM_TO_METERS(frontD.X()) + 
		       RASM_TO_METERS(frontD.Y())*RASM_TO_METERS(frontD.Y()));
      float rollAngle = atan(rise/run);
      
      if(fabs(rollAngle) > TMAP::dangerousRoll)
	{
	  if(g_debug_planner)
	    {
	      printf("[%s:%d %f] Angle %f deg exceeds max roll %f deg with axle between ", 
		     __FILE__, __LINE__, now(),
		     rollAngle*180.0/M_PI, TMAP::dangerousRoll*180.0/M_PI);
	      frontA.print(stdout);
	      printf(" and ");
	      frontB.print(stdout);
	      printf("\n");
	    }
	  obstacles.insert(frontA, frontB);
	  return OBST;
	}
  
      // get the pitch angle 
      rise = RASM_TO_METERS(rearPivot.Z()) - RASM_TO_METERS(frontPivot.Z());
      run = sqrt((RASM_TO_METERS(rearPivot.X()) - RASM_TO_METERS(frontPivot.X())) * (RASM_TO_METERS(rearPivot.X()) - RASM_TO_METERS(frontPivot.X())) + 
		 (RASM_TO_METERS(rearPivot.Y()) - RASM_TO_METERS(frontPivot.Y())) * (RASM_TO_METERS(rearPivot.Y()) - RASM_TO_METERS(frontPivot.Y())));
      float pitchAngle = atan(rise/run);
      
      if(fabs(pitchAngle) > TMAP::dangerousPitch)
	{
	  if(g_debug_planner)
	    {
	      printf("[%s:%d %f] Angle %f deg exceeds max pitch %f deg with front pivot at ", 
		     __FILE__, __LINE__, now(),
		     pitchAngle*180.0/M_PI, TMAP::dangerousPitch*180.0/M_PI);
	      frontPivot.print(stdout);
	      printf(" and rear pivot at ");
	      rearPivot.print(stdout);
	      printf("\n");
	    }
	  obstacles.insert(frontPivot, rearPivot);
	  return OBST;
	}
    
      // check the front axle clearance at a few points
      int triangleIndex = a_tri;
      RASM_UNITS minclearance = METERS_TO_RASM(TMAP::axleHeight);
      RASM::point3d minAxlePoint(0,0,0);
      for(uint32_t j=0; j < TMAP::numAxleChecks; j++)
	{
	  float percent = (((float)j)+1.0) / (((float)TMAP::numAxleChecks)+1.0);
	
	  RASM::point3d axlePoint = frontD;
	  for(uint32_t k=0; k < 3; k++)
	    {
	      axlePoint.coord3d[k] = (RASM_UNITS)(percent*((float)axlePoint.coord3d[k]));
	    }
	  
	  axlePoint += frontB;
	  
	  RASM::point3d modelPoint;
	  triangleIndex = -1;
	  
	  if(-1 == terrain.findSurfaceAt(axlePoint, modelPoint, triangleIndex))
	    {
	      continue;
	    }
	  
	  RASM_UNITS clearance = axlePoint.Z() - modelPoint.Z();
	  if(clearance < minclearance)
	    {
	      minclearance = clearance;
	      minAxlePoint = axlePoint;
	    }	  
	  
	} // for each axle check

      if(minclearance <= 0)
	{
	  if(g_debug_planner)
	    {
	      printf("[%s:%d %f] Front axle collision between ",
		     __FILE__, __LINE__, now());
	      frontA.print(stdout);
	      printf(" and ");
	      frontB.print(stdout);
	      printf("\n");
	    }
	  obstacles.insert(frontA, frontB);
	  return OBST; // used to be -1.0, which indicated that the path "could not be evaluated"
	}

      /* check the side clearance at the middle of the body
	 this the midpoint of the line between the wheels
      */
      RASM::point3d bodyL = frontA + rearA;
      RASM::point3d bodyR = frontB + rearB;
      bodyL /= 2.0;
      bodyR /= 2.0;
      
      RASM::point3d modelLPoint, modelRPoint;
      assert((-1) != terrain.findSurfaceAt(bodyL, modelLPoint, lrIndex));
      assert((-1) != terrain.findSurfaceAt(bodyR, modelRPoint, rrIndex));

      RASM_UNITS clearanceL = bodyL.Z() - modelLPoint.Z();
      RASM_UNITS clearanceR = bodyR.Z() - modelRPoint.Z();
      RASM_UNITS bodyClearance = clearanceL;
      if(clearanceR < clearanceL)
	{
	  bodyClearance = clearanceR;
	}

      if(bodyClearance <= 0)
	{
	  if(g_debug_planner)
	    {
	      printf("[%s:%d %f] Body collision, left clearance %f, right clearance %f\n",
		     __FILE__, __LINE__, now(),
		     RASM_TO_METERS(clearanceL), RASM_TO_METERS(clearanceR));
	    }
	  obstacles.insert(bodyL, bodyR);
	  return OBST; // used to be 1
	}

      if(g_debug_planner)
	{
	  printf("[%s:%d %f] point %d (%0.3f %0.3f %0.3f): axle clearance %0.2fm Body clearance %0.2f roll: %0.1fdeg (safe is %0.3f unsafe is %0.3f) pitch: %0.1f deg (safe is %0.3f unsafe is %0.3f)\n",
		 __FILE__, __LINE__, now(),
		 i,
		 RASM_TO_METERS(frontPivot.X()), RASM_TO_METERS(frontPivot.Y()), RASM_TO_METERS(frontPivot.Z()),
		 RASM_TO_METERS(minclearance),
		 RASM_TO_METERS(bodyClearance),
		 rollAngle*180.0/M_PI, 
		 TMAP::safeRoll*180.0/M_PI, 
		 TMAP::dangerousRoll*180.0/M_PI, 
		 pitchAngle*180.0/M_PI, 
		 TMAP::safePitch*180.0/M_PI, 
		 TMAP::dangerousPitch*180.0/M_PI);
	}

      /* 1 = safe, 0 = dangerous */

      rollAngle  = fabs(rollAngle);
      pitchAngle = fabs(pitchAngle);

      rollAngle   = ((rollAngle >TMAP::safeRoll )?(rollAngle  -TMAP::safeRoll ):0.0);
      pitchAngle  = ((pitchAngle>TMAP::safePitch)?(pitchAngle -TMAP::safePitch):0.0);

      /* if the clearance is half the axle height or more, then we're good
       * if the clearance is zero or less, then we'll hit something
       * otherwise scale in between
       */
      float safetyClearance = RASM_TO_METERS(minclearance);
      if(safetyClearance > (TMAP::axleHeight/2.0))
	{
	  safetyClearance=(TMAP::axleHeight/2.0);
	}

      float safetyClearanceBody = RASM_TO_METERS(bodyClearance);
      if(safetyClearanceBody > (TMAP::axleHeight/2.0))
	{
	  safetyClearanceBody=(TMAP::axleHeight/2.0);
	}

      float danger[4] = {
	/* how safe is the roll? */
	1.0 - (rollAngle/(TMAP::dangerousRoll-TMAP::safeRoll)),
	/* how safe is the pitch? */
	1.0 - (pitchAngle/(TMAP::dangerousPitch-TMAP::safePitch)),
	/* how safe is the axle clearance? */
	safetyClearance/(TMAP::axleHeight/2.0),
	/* how safe is the body clearance? */
	safetyClearanceBody/(TMAP::axleHeight/2.0)
      };

      /* how dangerous is it overall?  1 = safe, 0 = dangerous */
      float percent = danger[0] * danger[1] * danger[2] * danger[3];

      if(g_debug_planner)
	{
	  printf("[%s:%d %f] point %d overall: %0.3f (1 = good, 0 = bad)", 
		 __FILE__, __LINE__, now(), i, percent);
	  printf(" roll %0.3f", danger[0]);
	  printf(" pitch %0.3f", danger[1]);
	  printf(" axle clearance %0.3f", danger[2]);
	  printf(" body clearance %0.3f\n", danger[3]);
	}

      assert((percent >= 0.0) && (percent <= 1.0));

      traversability += FREE*(percent) + OBST*(1.0 - percent);

    } // for each point on this path

  if(g_debug_planner)
    {
      printf("[%s:%d %f] path index %d has near traversability score %f\n", 
	     __FILE__, __LINE__, now(),
	     path_index, traversability);
    }

  return traversability;

} // evaluate_near()


/* waits for a goal and executes it until reached or an abort
 * if a new goal is sent, it replaces the old one automatically
 * the goal can be aborted by a message or stale data
 * after finishing, it sends any response messages and stops the vehicle.
 * returns true if goal was reached correctly, false on an error
 */
bool RASM::eval::execute_goal(PathModel *path_model, 
			      bool save_models,
			      bool debug_output)
{
  assert(NULL != m_interface);
	
  get_first_goal();
  // if(RASM::g_execute_point_to_goal_maneuver)
  //   {
  //     m_interface->point_to_goal(RASM::g_point_to_goal_radius_meters,
  // 				 RASM::g_point_to_goal_speed_meters_per_sec,
  // 				 RASM::g_point_to_goal_drive_cmd_timeout_sec,
  // 				 RASM::g_point_to_goal_timeout_sec,
  // 				 RASM::g_pointing_threshold_rad);
  //   }
	
  bool publish_paths = true;
	
  m_last_path_index = -1;

  /*
   * Starting out, don't allow reverse paths
   */
  path_model->update_path_set(/*use_reverse_paths=*/false);
	
  while(1)
    {
      printf("[%s:%d %f] start of loop, sequence number %d\n", 
	     __FILE__, __LINE__, now(),
	     m_world_sequence_number);
		
      m_interface->pause(1/*msec*/);
		
      /* check if the goal changed */
      if(check_for_new_goal()) 
	{
	  m_interface->pause(100);
	  // if(RASM::g_execute_point_to_goal_maneuver)
	  //   {
	  //     m_interface->point_to_goal(RASM::g_point_to_goal_radius_meters,
	  // 				 RASM::g_point_to_goal_speed_meters_per_sec,
	  // 				 RASM::g_point_to_goal_drive_cmd_timeout_sec,
	  // 				 RASM::g_point_to_goal_timeout_sec,
	  // 				 RASM::g_pointing_threshold_rad);
	  //   }
	  continue;
	}
		
      /* check if the goal was removed,
       * if so then go back to the start and wait for a goal
       */
      if(m_interface->should_abort_goal())
	{
	  m_interface->send_at_goal_notification(/*success=*/false);
	  printf("[%s:%d %f] goal cleared\n",
		 __FILE__, __LINE__, now());

	  /*
	   * When navigation stops due to reaching or aborting goal,
	   * or due to an internal error, don't allow reverse paths.
	   */
	  path_model->update_path_set(/*use_reverse_paths=*/false);
	  break;
	}
		
      /* check if the goal was reached */
      if(!m_interface->update_pose(m_rover_pose))
	{
	  printf("[%s:%d %f] error retrieving pose\n",
		 __FILE__, __LINE__, now());

	  /*
	   * When navigation stops due to reaching or aborting goal,
	   * or due to an internal error, don't allow reverse paths.
	   */
	  path_model->update_path_set(/*use_reverse_paths=*/false);
	  break;
	}
      RASM::point3d pos; float r = 0; float p = 0; float y = 0;
      m_rover_pose.get(pos, r, p, y);
      printf("[%s:%d %f] pose is ( x %0.3f m, y %0.3f m, z %0.3f m, roll %0.3f deg, pitch %0.3f deg, yaw %0.3f deg, t: %f )\n",
	     __FILE__, __LINE__, now(),
	     RASM_TO_METERS(pos.X()),
	     RASM_TO_METERS(pos.Y()),
	     RASM_TO_METERS(pos.Z()),
	     r*180.0/M_PI,
	     p*180.0/M_PI,
	     y*180.0/M_PI,
	     m_rover_pose.getTime());
		
      if(at_goal())
	{
	  printf("[%s:%d %f] at goal\n",
		 __FILE__, __LINE__, now());

	  /*
	   * When navigation stops due to reaching or aborting goal,
	   * or due to an internal error, don't allow reverse paths.
	   */
	  path_model->update_path_set(/*use_reverse_paths=*/false);
	  break;
	}

      update_world_model();

      /* 
       * Check safety gate
       */ 
      if(RASM::SafetyGate::OK != m_interface->m_safety_gate->get_state())
      	{
      	  // printf("[%s:%d %f] ERROR: haven't gotten pose in %0.1fs!\n", 
      	  // 	 __FILE__, __LINE__, now(),
      	  // 	 g_pose_timeout_sec);
      	  // printf("Current time: %0.3f\n", now());
      	  // printf("   Pose time: %0.3f\n", m_interface->most_recent_pose_timestamp());
	  printf("[%s:%d %f] safety gate reports a problem\n", __FILE__, __LINE__, now());
			
      	  m_interface->send_stop_cmd();
	  m_interface->delete_goal();

      	  /*
      	   * When navigation stops due to reaching or aborting goal,
      	   * or due to an internal error, don't allow reverse paths.
      	   */
      	  path_model->update_path_set(/*use_reverse_paths=*/false);

      	  m_interface->pause(100);
      	  continue;
      	}
		
		
      /* check to see if the world model includes both us and the
       * goal region. if not, then go back to the beginning of the
       * loop. 
       *
       * TBD: if any of these tests fail should we command a stop?
       */
      if(0 == m_world_model.numTriangles())
	{
	  printf("[%s:%d %f] world model has no triangles (but it has %d points)...\n",
		 __FILE__, __LINE__, now(),
		 m_world_model.numPoints());

	  /*
	   * When navigation stops due to reaching or aborting goal,
	   * or due to an internal error, don't allow reverse paths.
	   */
	  path_model->update_path_set(/*use_reverse_paths=*/false);

	  m_interface->pause(100);
	  continue;
	}
      RASM::point2d rover = (RASM::point2d)m_rover_pose.getPosition();
      if(-1 == m_world_model.findTriangle2d(rover))
	{
	  printf("[%s:%d %f] rover pose (%0.3f %0.3f) not in world model...\n",
		 __FILE__, __LINE__, now(),
		 RASM_TO_METERS(rover.X()), RASM_TO_METERS(rover.Y()));
	  m_interface->send_stop_cmd();

	  /*
	   * When navigation stops due to reaching or aborting goal,
	   * or due to an internal error, don't allow reverse paths.
	   */
	  path_model->update_path_set(/*use_reverse_paths=*/false);

	  m_interface->pause(100);
	  continue;
	  //	  add_rover_pose_to_world_model();			
	}
      if(-1 == m_world_model.findTriangle2d(m_goal.m_position))
	{
	  printf("[%s:%d %f] execute_goal(): goal not in world model...\n",
		 __FILE__, __LINE__, now());
	  m_interface->send_stop_cmd();

	  /*
	   * When navigation stops due to reaching or aborting goal,
	   * or due to an internal error, don't allow reverse paths.
	   */
	  path_model->update_path_set(/*use_reverse_paths=*/false);

	  m_interface->pause(100);
	  continue;
	  //	  add_goal_to_world_model();
	}
		
      printf("[%s:%d %f] seq %d evaluating paths on world model...\n",
	     __FILE__, __LINE__, now(),
	     m_world_sequence_number);
				
      m_interface->update_pose(m_rover_pose);
		
      /* TBD: extrapolate forward in time
       * guess where the rover *will* be when the path is sent
       */		
      RASM::point3d rover_pos = m_rover_pose.getPosition();
      printf("[%s:%d %f] Transforming pathset to rover's current pose (%0.3f %0.3f %0.3f, %0.3f deg)\n",
	     __FILE__, __LINE__, now(),
	     RASM_TO_METERS(rover_pos.X()),
	     RASM_TO_METERS(rover_pos.Y()),
	     RASM_TO_METERS(rover_pos.Z()),
	     m_rover_pose.getOrientation().yaw * 180.0 * M_PI);
      path_model->translateAndRotate(m_rover_pose);
		
      if(!m_world_model.haveGoal())
	{
	  printf("[%s:%d %f] Warning, world model did not have goal, setting it now!\n",
		 __FILE__, __LINE__, now());
	  m_world_model.setGoal(m_goal.m_position);
	}
		
      printf("[%s:%d %f] Evaluating pathset\n",
	     __FILE__, __LINE__, now());
      if(!evaluate_paths(path_model,
			 publish_paths, 
			 m_world_sequence_number, 
			 save_models, 
			 debug_output))
	{
	  printf("[%s:%d %f] Cannot find a safe path, stopping robot\n",
		 __FILE__, __LINE__, now());
	  m_interface->send_stop_cmd();

	  /*
	   * Can't find a safe path forward, so try reverse paths
	   */
	  path_model->update_path_set(/*use_reverse_paths=*/true);
	} 
      else
	{
	  printf("[%s:%d %f] commanding path index %d\n", __FILE__, __LINE__, now(), m_last_path_index);
	  fflush(stdout);
	  m_interface->send_path_cmd(m_last_path_index);

	  /*
	   * Now that we're safely moving forward, don't allow reverse paths
	   */
	  path_model->update_path_set(/*use_reverse_paths=*/false); 
	  
	  /*
	   * Send the selected path
	   */
	  RASM::point3d origin(0,0,0);
	  RASM::mesh mesh_to_send;

	  mesh_to_send.m_vertices = new RASM::point3d[path_model->m_num_path_points[m_last_path_index] * 4];

	  for(uint32_t k=0; k < path_model->m_num_path_points[m_last_path_index]; k++)
	    {
	      mesh_to_send.m_vertices[k + 0 * path_model->m_num_path_points[m_last_path_index]] = path_model->m_front_left_wheel_paths [m_last_path_index][k];
	      mesh_to_send.m_vertices[k + 1 * path_model->m_num_path_points[m_last_path_index]] = path_model->m_front_right_wheel_paths [m_last_path_index][k];
	      mesh_to_send.m_vertices[k + 2 * path_model->m_num_path_points[m_last_path_index]] = path_model->m_rear_left_wheel_paths [m_last_path_index][k];
	      mesh_to_send.m_vertices[k + 3 * path_model->m_num_path_points[m_last_path_index]] = path_model->m_rear_right_wheel_paths [m_last_path_index][k];
	    }

	  mesh_to_send.m_num_vertices = path_model->m_num_path_points[m_last_path_index] * 4;
	  mesh_to_send.m_faces = NULL;
	  mesh_to_send.m_num_faces = 0;
	  m_interface->publish_map(RASM::MapType::PATH, 
				   m_world_sequence_number, 
				   mesh_to_send,
				   origin,
				   now());

	  if(save_models)
	    {
	      char path_file[256];
	      sprintf(path_file, "path.%04d.obj", m_world_sequence_number);
	      FILE* f = fopen(path_file, "w");

	      for(uint32_t k=0; k < mesh_to_send.m_num_vertices; k++)
		{
		  fprintf(f, "v %0.3f %0.3f %0.3f\n",
			  RASM_TO_METERS(mesh_to_send.m_vertices[k].coord3d[0]),
			  RASM_TO_METERS(mesh_to_send.m_vertices[k].coord3d[1]),
			  RASM_TO_METERS(mesh_to_send.m_vertices[k].coord3d[2]));
		}

	      fclose(f);
	    }

	  delete[] mesh_to_send.m_vertices;

	}
		
      /* pause for up to half a second waiting for new data */
      printf("[%s:%d %f] Waiting for additional data\n",
	     __FILE__, __LINE__, now());
		
      m_interface->pause(0/*msec*/);
		
      for(int i=0; i < 50 && !m_interface->have_new_map(RASM::MapType::WORLD); i++)
	{
	  m_interface->pause(10/*msec*/);
	}
		
      if(!m_interface->have_new_map(RASM::MapType::WORLD))
	{
	  printf("[%s:%d %f] Haven't gotten any new tmapData yet\n",
		 __FILE__, __LINE__, now());
	}
      else
	{
	  printf("[%s:%d %f] Got new tmapData\n",
		 __FILE__, __LINE__, now());
	}

    }/* end of loop to goal */
  
  m_interface->send_stop_cmd();

  /*
   * When navigation stops due to reaching or aborting goal,
   * or due to an internal error, don't allow reverse paths.
   */
  path_model->update_path_set(/*use_reverse_paths=*/false);

  /*
   * We haven't yet defined a case where this function will return an error.
   * Pose and sensor-data watchdogs tripping are not necessarily due to errors,
   * and generally the user will want the planner to resume operation when pose
   * or sensor data resume.
   */
  return true;
} /// execute_goal()

/// Constructor
RASM::eval::eval(RASM::CommsInterface* _interface, std::map<std::string, std::string>& config)
  : m_interface(_interface),
    m_world_sequence_number(0),
    m_world_model_timestamp(-1.0),
    m_last_path_index(-1)
{
  assert(NULL != m_interface);
  initConfigParameters(config);
}


bool RASM::eval::stale_world_model(double max_age_sec) const 
{
  printf("[%s:%d %f] checking world model timestamp %f against pose timestamp %f and max age %f\n",
	 __FILE__, __LINE__, now(),
	 m_world_model_timestamp,
	 m_rover_pose.getTime(),
	 max_age_sec);
	
  /* check if any data is invalid */
  if((m_world_model_timestamp <= 0.0) || !m_rover_pose.isValid()) return true;
	
  /* check the time */
  return (fabs(m_world_model_timestamp - m_rover_pose.getTime()) > max_age_sec);
}


bool RASM::eval::evaluate_paths(PathModel* path_model,
				bool publish_path,
				unsigned int sequence_number,
				bool save_models,
				bool debug_output)
{
  assert(NULL != path_model);
  assert(path_model->m_path_set_size > 0);
  assert(NULL != path_model->m_path_risk_factor);
  assert(NULL != m_interface);
	
  /* store the near, far and final/aggregate path costs
   */
  double *near_cost  = (double *)alloca(path_model->m_path_set_size*sizeof(double));
  double *far_cost   = (double *)alloca(path_model->m_path_set_size*sizeof(double));
  double *total_cost = (double *)alloca(path_model->m_path_set_size*sizeof(double));
	
  assert(NULL != near_cost);
  assert(NULL != far_cost);
  assert(NULL != total_cost);

  /* start with the near evaluation
   * this checks for dynamic obstacle collision if needed
   * and simulates motion along the terrain
   *
   * motion simulation may find an impasseable obstacle,
   * in that case the obstacle is added to the list
   * and will also be avoided in the far evaluation
   */
  printf("[%s:%d %f] starting near evaluation\n",
	 __FILE__, __LINE__, now());

  for(unsigned int i=0; i < path_model->m_path_set_size; i++)
    {
      bool path_reaches_goal = false;

      near_cost[i] = evaluate_near(m_world_model,
				   path_model, i,
				   m_obstacles,
				   m_goal,
				   path_reaches_goal);

      if(path_reaches_goal)
	{
	  far_cost[i] = 0;
	} 
      else
	{
	  far_cost[i] = evaluate_far(m_world_model,
				     path_model, i,
				     m_obstacles,
				     m_goal);
	}

      total_cost[i] = (near_cost[i] + RASM::g_far_weight * far_cost[i]) * path_model->m_path_risk_factor[i];
    }
	
  printf("[%s:%d %f] Completed path evaluation\n",
	 __FILE__, __LINE__, now());

  RASM::point3d pos;
  float roll = 0, pitch = 0, yaw = 0;
  m_rover_pose.get(pos, roll, pitch, yaw);
  printf("[%s:%d %f] rover at (%0.3f %0.3f, %0.3f deg), goal at (%0.3f %0.3f, %0.3f maj %0.3f min)\n",
	 __FILE__, __LINE__, now(),
	 RASM_TO_METERS(pos.X()),
	 RASM_TO_METERS(pos.Y()),
	 yaw * 180.0 / M_PI,
	 RASM_TO_METERS(m_goal.m_position.X()),
	 RASM_TO_METERS(m_goal.m_position.Y()),
	 m_goal.m_semimajor_axis,
	 m_goal.m_semiminor_axis);
	
  if(save_models)
    {
      char world_file[256];
      sprintf(world_file,"world.%04d.obj",m_world_sequence_number);
      m_world_model.writeToFile(world_file);
    }
  /*** done evaluating paths ***/
	
  /*
   * Update obstacle data 
   */
  if(save_models)
    {
      m_obstacles.writeToFile("obstacles.obj");
    }
  m_obstacles.prune();
	
  /* 
   * Apply a hysteresis bonus to the last path (if it is passable)
   */
  if( ( m_last_path_index >= 0 ) && 
      ( total_cost[m_last_path_index] >= 0 ) )
    {
      total_cost[m_last_path_index] *= RASM::g_last_path_bonus;
    }
	
  /* 
   * Pick the best path
   */
  bool found_path = false;
  unsigned int best_path_index = 0;
  double best_cost = FLT_MAX;
  for(unsigned int i=0; i < path_model->m_path_set_size; i++)
    {
      if(g_debug_planner)
	{
	  printf("[%s:%d %f] pick the best path: %d %f %d %d, %f %f %f %f, %f\n",
		 __FILE__, __LINE__, now(),
		 i, 
		 path_model->m_path_risk_factor[i],
		 found_path,
		 best_path_index,
		 near_cost[i],
		 far_cost[i], RASM::g_far_weight * far_cost[i],
		 total_cost[i],
		 best_cost);
	}

      if(( total_cost[i] >= 0.0) &&
	 ( total_cost[i] < best_cost ) )
	{
	  best_path_index = i;
	  best_cost = total_cost[i];
	  found_path = true;
	}
    }

  /* check for no path */
  if(!found_path)
    {
      printf("[%s:%d %f] Error, no paths!\n",
	     __FILE__, __LINE__, now());
      m_last_path_index = -1;
      return false;
    }
	
  printf("[%s:%d %f] Selected path %d\n",
	 __FILE__, __LINE__, now(),
	 best_path_index);
	
  m_last_path_index = best_path_index;
	
  /* return true if an path was selected */
  return true;

} // evaluate_paths()


///////////////////////////////////////////////////
//  Loading configuration parameters from ini file
///////////////////////////////////////////////////
bool RASM::eval::initConfigParameters(std::map<std::string, std::string>& config)
{
  // if(config.find("pose_timeout_sec") != config.end()) g_pose_timeout_sec = atof(config["pose_timeout_sec"].c_str());
  // if(config.find("sensor_data_timeout_sec") != config.end()) g_sensor_data_timeout_sec = atof(config["sensor_data_timeout_sec"].c_str());
  // assert(g_pose_timeout_sec > 0);
  // assert(g_sensor_data_timeout_sec > 0);

  // printf("[%s:%d] Pose-watchdog timeout is %0.3f sec, data-watchdog timeout is %0.3f sec\n",
  // 	 __FILE__, __LINE__,
  // 	 g_pose_timeout_sec,
  // 	 g_sensor_data_timeout_sec);

  if(config.find("path_length_meters") != config.end()) RASM::g_path_length_meters = atof(config["path_length_meters"].c_str());
  if(config.find("path_resolution_meters") != config.end()) RASM::g_path_resolution_meters = atof(config["path_resolution_meters"].c_str());
  assert(RASM::g_path_length_meters > 0);
  assert(RASM::g_path_resolution_meters > 0);

  if(config.find("max_heading_change_deg") != config.end()) RASM::g_path_max_heading_change_rad = atof(config["max_heading_change_deg"].c_str()) * M_PI/180.0;
  if(config.find("heading_resolution_deg") != config.end()) RASM::g_path_heading_resolution_rad = atof(config["heading_resolution_deg"].c_str()) * M_PI/180.0;
  assert(RASM::g_path_max_heading_change_rad > 0);
  assert(RASM::g_path_heading_resolution_rad > 0);

  if(config.find("reverse_path_penalty") != config.end()) RASM::g_reverse_path_penalty = atof(config["reverse_path_penalty"].c_str());
  if(config.find("last_path_bonus") != config.end()) RASM::g_last_path_bonus = atof(config["last_path_bonus"].c_str());
  assert(RASM::g_last_path_bonus <= 2.0);
  assert(RASM::g_last_path_bonus > 0);

  if(config.find("max_speed_meters_per_sec") != config.end()) RASM::g_max_speed_meters_per_sec = atof(config["max_speed_meters_per_sec"].c_str());
  if(config.find("path_duration_sec") != config.end()) RASM::g_path_duration_sec = atof(config["path_duration_sec"].c_str());
  if(config.find("reverse_path_speed_penalty") != config.end()) RASM::g_reverse_path_speed_penalty = atof(config["reverse_path_speed_penalty"].c_str());
  assert(RASM::g_max_speed_meters_per_sec > 0);
  assert(RASM::g_path_duration_sec > 0);

  //  if(config.find("execute_point_to_goal_maneuver") != config.end()) RASM::g_execute_point_to_goal_maneuver = atob(config["execute_point_to_goal_maneuver"].c_str());
  //  if(config.find("point_to_goal_radius_meters") != config.end()) RASM::g_point_to_goal_radius_meters = atof(config["point_to_goal_radius_meters"].c_str());
  //  if(config.find("point_to_goal_speed_meters_per_sec") != config.end()) RASM::g_point_to_goal_speed_meters_per_sec = atof(config["point_to_goal_speed_meters_per_sec"].c_str());
  //  if(config.find("point_to_goal_drive_cmd_timeout_sec") != config.end()) RASM::g_point_to_goal_drive_cmd_timeout_sec = atof(config["point_to_goal_drive_cmd_timeout_sec"].c_str());
  //  if(config.find("point_to_goal_timeout_sec") != config.end()) RASM::g_point_to_goal_timeout_sec = atof(config["point_to_goal_timeout_sec"].c_str());
  //  if(config.find("pointing_threshold_deg") != config.end()) RASM::g_pointing_threshold_rad = atof(config["pointing_threshold_deg"].c_str()) * M_PI/180.0;
  // if(RASM::g_execute_point_to_goal_maneuver)
  //   {
  //     assert(RASM::g_point_to_goal_radius_meters > 0);
  //     assert(RASM::g_point_to_goal_speed_meters_per_sec > 0);
  //     assert(RASM::g_point_to_goal_drive_cmd_timeout_sec > 0);
  //     assert(RASM::g_point_to_goal_timeout_sec > 0);
  //     assert(RASM::g_pointing_threshold_rad > 0);
  //   }

  /*
   * used in tmap_astar
   */
  if(config.find("use_heuristic") != config.end()) TMAP::USE_HEURISTIC = atob(config["use_heuristic"].c_str());
  if(config.find("no_path_smoothing") != config.end()) TMAP::NO_PATH_SMOOTHING = atob(config["no_path_smoothing"].c_str());
  if(config.find("astar_z_min") != config.end()) TMAP::astarZmin = atof(config["astar_z_min"].c_str());
  if(config.find("astar_z_max") != config.end()) TMAP::astarZmax = atof(config["astar_z_max"].c_str());
  if(config.find("min_altitude") != config.end()) TMAP::minAltitude = atof(config["min_altitude"].c_str());
  if(config.find("max_altitude") != config.end()) TMAP::maxAltitude = atof(config["max_altitude"].c_str());
  if(config.find("min_peak_trough") != config.end()) TMAP::minPeakTrough = atof(config["min_peak_trough"].c_str());
  if(config.find("max_peak_trough") != config.end()) TMAP::maxPeakTrough = atof(config["max_peak_trough"].c_str());

  if(config.find("far_weight") != config.end()) RASM::g_far_weight = atof(config["far_weight"].c_str());

  // TMAP::USE_HEURISTIC = ini.GetBoolValue("astar","useHeuristic",true);
  // TMAP::NO_PATH_SMOOTHING  = ini.GetBoolValue("astar","noPathSmoothing",false);
  // TMAP::astarZmin = ini.GetDoubleValue("astar","astarZmin", 0.7);
  // TMAP::astarZmax = ini.GetDoubleValue("astar","astarZmax", 0.95);
  // TMAP::minAltitude = ini.GetDoubleValue("astar","minAltitude", 1.0);
  // TMAP::maxAltitude = ini.GetDoubleValue("astar","maxAltitude", 5.0);
  // TMAP::minPeakTrough = ini.GetDoubleValue("astar","minPeakTrough", 0.1);
  // TMAP::maxPeakTrough = ini.GetDoubleValue("astar","maxPeakTrough", 0.3);
	
  /*
   * used in tmap_arc
   */
  if(config.find("num_axle_checks") != config.end()) TMAP::numAxleChecks = atof(config["num_axle_checks"].c_str());
  if(config.find("wheel_distance_meters") != config.end()) TMAP::wheelDist = atof(config["wheel_distance_meters"].c_str());
  if(config.find("axle_height_meters") != config.end()) TMAP::axleHeight = atof(config["axle_height_meters"].c_str());
  //  if(config.find("rover_length_meters") != config.end()) TMAP::roverLength = atof(config["rover_length_meters"].c_str());
  if(config.find("dangerous_roll_deg") != config.end()) TMAP::dangerousRoll = atof(config["dangerous_roll_deg"].c_str()) * M_PI/180.0;
  if(config.find("dangerous_pitch_deg") != config.end()) TMAP::dangerousPitch = atof(config["dangerous_pitch_deg"].c_str()) * M_PI/180.0;
  if(config.find("safe_roll_deg") != config.end()) TMAP::safeRoll = atof(config["safe_roll_deg"].c_str()) * M_PI/180.0;
  if(config.find("safe_pitch_deg") != config.end()) TMAP::safePitch = atof(config["safe_pitch_deg"].c_str()) * M_PI/180.0;
  if(config.find("arc_to_path_factor") != config.end()) TMAP::arcToPathFactor = atof(config["arc_to_path_factor"].c_str());
  if(config.find("verbose_intersection") != config.end()) TMAP::verboseIntersection = atob(config["verbose_intersection"].c_str());
  assert(TMAP::numAxleChecks > 0);
  assert(TMAP::wheelDist > 0);
  assert(TMAP::axleHeight >= 0);
  //  assert(TMAP::roverLength > 0);
  assert(TMAP::dangerousRoll > 0);
  assert(TMAP::dangerousPitch > 0);
  assert(TMAP::safeRoll >= 0);
  assert(TMAP::safePitch >= 0);

  if(config.find("debug_planner") != config.end()) g_debug_planner = atob(config["debug_planner"].c_str());

  return true;
}
