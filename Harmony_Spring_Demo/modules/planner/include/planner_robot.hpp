/* +---------------------------------------------------------------------------+
 /*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   planner.hpp
 * Author: Dibyendu Ghosh
 * Organization : Intel Research Lab bangalore
 * Created on 12 February, 2016, 11:50 AM
 */
// ------------------------------------------------------*/
#include <iostream>
#include <vector>
#include <mrpt/poses.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/nav/planners/PlannerRRT_SE2_TPS.h>
using namespace std;
using namespace mrpt;
using namespace mrpt::nav;
#ifndef PLANNER_ROBOT_HPP
#define PLANNER_ROBOT_HPP

class Planner_robot {
public:

mrpt::poses::CPose2D Path_Planner(mrpt::poses::CPose2D current_pose, mrpt::poses::CPose2D local_goal, mrpt::maps::CSimplePointsMap obs_map);

void planner_ValueIterative(const mrpt::maps::COccupancyGridMap2D &theMap,const mrpt::poses::CPose2D &current_pose, const mrpt::poses::CPose2D &local_goal,std::deque<mrpt::math::TPoint2D> &path, bool &notFound,
			float maxSearchPathLength);
//mrpt::math::TPose2D motion_command(static mrpt::nav::TListPTGs ptgs_num, mrpt::nav::TMoveTreeSE2_TP::NODE_TYPE nodes_planner,static mrpt::math::TPolygon2D robot_size);

/** The maximum occupancy probability to consider a cell as an obstacle, default=0.5  */
float	occupancyThreshold;
/** The minimum distance between points in the returned found path (default=0.4); Notice
  *  that full grid resolution is used in path finding, this is only a way to reduce the
  *  amount of redundant information to be returned.
  */
float	minStepInReturnedPath;

mrpt::nav::PlannerRRT_SE2_TPS  planner_m;

float	robotRadius;  //!< The aproximate robot radius used in the planification. Default is 0.35m

private:

};

#endif /* PLANNER_HPP */
