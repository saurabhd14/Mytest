/* +---------------------------------------------------------------------------+
 /*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   planner.cpp
 * Author: Dibyendu Ghosh
 * Organization : Intel Research Lab bangalore
 * Created on 12 February, 2016, 11:50 AM
 */
// ------------------------------------------------------*/
#include <iostream>
#include <mrpt/poses.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/nav/planners/PlannerRRT_SE2_TPS.h>
#include <mrpt/nav/tpspace/motion_planning_utils.h>
#include <mrpt/utils/CConfigFile.h>
#include <../include/planner_robot.hpp>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::nav;
using namespace std;
#define max_wheelang M_PI/4   /* maximum wheel angles */

mrpt::math::TPose2D motion_command(mrpt::nav::TListPTGPtr ptgs,	mrpt::nav::TMoveTreeSE2_TP::NODE_TYPE nodes_planner, mrpt::math::TPolygon2D robot_size) {

	float lvel = 0.0; //linear velocity
	float avel = 0.0; // angular velocity
	std::vector<float> ptg_lvel, ptg_avel;
	for (size_t t = 0; t < ptgs.size(); t++) {
		ptgs[t]->directionToMotionCommand(nodes_planner.edge_to_parent->ptg_K, lvel, avel);
		ptg_lvel.push_back(lvel);
		ptg_avel.push_back(avel);
	}

	/* minimum turning radius of robot*/
	std::vector<double> robot_xm, robot_ym;
	robot_size.getPlotData(robot_xm, robot_ym);
	double robot_l = *std::max_element(robot_xm.begin(), robot_xm.end()) * 2;
	double robot_w = *std::max_element(robot_ym.begin(), robot_ym.end()) * 2;

	double Rmin = robot_l * tan((M_PI / 2) - max_wheelang) + robot_w / 2;

	/* linear velocity and angular velocity slection */
	std::vector<mrpt::math::TPoint3D> vw_selection;

	for (size_t t = 0; t < ptg_lvel.size(); t++) {
		double ratio = abs(ptg_lvel.at(t) / ptg_avel.at(t));
		if (ratio >= Rmin) {
			vw_selection.push_back(
					mrpt::math::TPoint3D(ratio, ptg_lvel.at(t),
							ptg_avel.at(t)));
		}
	}

	/*if (vw_selection.size() != 0) {
		std::sort(vw_selection.begin(), vw_selection.end());

		if (vw_selection.size() == 1) {
			lvel = vw_selection.at(0).y;
			avel = vw_selection.at(0).z;
		}
		if (vw_selection.size() != 1 && vw_selection.size() % 2 == 0) {
			lvel = vw_selection.at(vw_selection.size() / 2).y;
			avel = vw_selection.at(vw_selection.size() / 2).z;
		}

		if (vw_selection.size() != 1 && vw_selection.size() % 2 != 0) {
			lvel = vw_selection.at(vw_selection.size() + 1 / 2).y;
			avel = vw_selection.at(vw_selection.size() + 1 / 2).z;
		}
	} else {
		lvel = 0.0;
		avel = 0.0;
	}*/
	lvel = 0.0;
	avel = 0.0;
	ptgs[1]->directionToMotionCommand(nodes_planner.edge_to_parent->ptg_K, lvel, avel);
	
	return mrpt::math::TPose2D(lvel, 0.0, avel);
}


mrpt::poses::CPose2D Planner_robot::Path_Planner(mrpt::poses::CPose2D current_pose, mrpt::poses::CPose2D local_goal,mrpt::maps::CSimplePointsMap obs_map) {

	mrpt::nav::PlannerRRT_SE2_TPS::TPlannerInput input;
	mrpt::nav::PlannerRRT_SE2_TPS::TPlannerResult result;

	static mrpt::nav::TListPTGPtr num_ptg = planner_m.getPTGs();
	static mrpt::math::TPolygon2D robot_size = planner_m.params.robot_shape;

	input.start_pose = current_pose;
	input.goal_pose = local_goal;
	input.obstacles_points = obs_map;
	mrpt::math::TPoint3D bbox_min, bbox_max;
	input.obstacles_points.boundingBox(bbox_min, bbox_max);
       std::cout<< " bbbox: " << bbox_min << " : " << bbox_max <<std::endl;

// Workspace bounding box:
	input.world_bbox_min = mrpt::math::TPoint2D(bbox_min.x, bbox_min.y);
	input.world_bbox_max = mrpt::math::TPoint2D(bbox_max.x, bbox_max.y);
	planner_m.solve(input, result);

	if (!result.success) {
		std::cout << "Path not Found " << std::endl;
		return mrpt::poses::CPose2D(0.0,0.0,0.0);
	} else {
		std::vector<mrpt::math::TPose2D> path;
		mrpt::nav::TMoveTreeSE2_TP::NODE_TYPE nodeall =(result.move_tree.getAllNodes().end())->second;
		//mrpt::nav::TMoveTreeSE2_TP::NODE_TYPE nodeall_vw =(result.move_tree.getAllNodes().end())->second;
		const mrpt::nav::TMoveTreeSE2_TP::node_map_t &lstnode =	result.move_tree.getAllNodes();
		for (mrpt::nav::TMoveTreeSE2_TP::node_map_t::const_iterator itnode =
				lstnode.begin(); itnode != lstnode.end(); ++itnode) {
			nodeall = itnode->second;
			std::cout <<"Path : " << nodeall.state <<std::endl;
			path.push_back(nodeall.state);
		}
		mrpt::math::TPose2D vw_calculation = motion_command(num_ptg, nodeall,
				robot_size);
		return mrpt::poses::CPose2D(vw_calculation.x, 0, vw_calculation.phi);
	}
}


/* value Iterative algorithm for circular robot */

void Planner_robot::planner_ValueIterative(const COccupancyGridMap2D &theMap, const CPose2D &origin_, const CPose2D &target_,std::deque<math::TPoint2D> &path, bool &notFound, float maxSearchPathLength) {

#define	CELL_ORIGIN		0x0000
#define	CELL_EMPTY		0x8000
#define	CELL_OBSTACLE	0xFFFF
#define	CELL_TARGET		0xFFFE

	const TPoint2D origin = TPoint2D(origin_);
	const TPoint2D target = TPoint2D(target_);

	std::vector<uint16_t> grid;
	int size_x, size_y, i, n, m;
	int x, y;
	bool searching;
	uint16_t minNeigh = CELL_EMPTY, maxNeigh = CELL_EMPTY, v = 0, c;
	int passCellFound_x = -1, passCellFound_y = -1;
	std::vector<uint16_t> pathcells_x, pathcells_y;

	// Check that origin and target falls inside the grid theMap!!
	// -----------------------------------------------------------
	ASSERT_(
			origin.x > theMap.getXMin() && origin.x < theMap.getXMax()
					&& origin.y > theMap.getYMin()
					&& origin.y < theMap.getYMax());
	ASSERT_(
			target.x > theMap.getXMin() && target.x < theMap.getXMax()
					&& target.y > theMap.getYMin()
					&& target.y < theMap.getYMax());

	// Check for the special case of origin and target in the same cell:
	// -----------------------------------------------------------------
	if (theMap.x2idx(origin.x) == theMap.x2idx(target.x)
			&& theMap.y2idx(origin.y) == theMap.y2idx(target.y)) {
		path.clear();
		path.push_back(TPoint2D(target.x, target.y));
		notFound = false;
		return;
	}

	// Get the grid size:
	// -----------------------------------------------------------
	size_x = theMap.getSizeX();
	size_y = theMap.getSizeY();

	// Fill the grid content with free-space and obstacles:
	// -----------------------------------------------------------
	grid.resize(size_x * size_y);
	for (y = 0; y < size_y; y++) {
		int row = y * size_x;
		for (x = 0; x < size_x; x++) {
			grid[x + row] =
					(theMap.getCell(x, y) > occupancyThreshold) ?
							CELL_EMPTY : CELL_OBSTACLE;
		}
	}

	// Enlarge obstacles with the robot radius:
	// -----------------------------------------------------------
	int obsEnlargement = (int) (ceil(robotRadius / theMap.getResolution()));
	for (int nEnlargements = 0; nEnlargements < obsEnlargement;
			nEnlargements++) {
//		int					size_y_1 = size_y-1;
//		int					size_x_1 = size_x-1;

		// For all cells(x,y)=EMPTY:
		// -----------------------------
		for (y = 2; y < size_y - 2; y++) {
			int row = y * size_x;
			int row_1 = (y + 1) * size_x;
			int row__1 = (y - 1) * size_x;

			for (x = 2; x < size_x - 2; x++) {
				uint16_t val = ( CELL_OBSTACLE - nEnlargements);

				//  A cell near an obstacle found??
				// -----------------------------------------------------
				if (grid[x - 1 + row__1] >= val || grid[x + row__1] >= val
						|| grid[x + 1 + row__1] >= val
						|| grid[x - 1 + row] >= val || grid[x + 1 + row] >= val
						|| grid[x - 1 + row_1] >= val || grid[x + row_1] >= val
						|| grid[x + 1 + row_1] >= val) {
					grid[x + row] = std::max((uint16_t) grid[x + row],
							(uint16_t) (val - 1));
				}
			}
		}
	}

	// Definitevely set new obstacles as obstacles
	for (y = 1; y < size_y - 1; y++) {
		int row = y * size_x;
		for (x = 1; x < size_x - 1; x++) {
			if (grid[x + row] > CELL_EMPTY)
				grid[x + row] = CELL_OBSTACLE;
		}
	}

	// Put the special cell codes for the origin and target:
	// -----------------------------------------------------------
	grid[theMap.x2idx(origin.x) + size_x * theMap.y2idx(origin.y)] =
			CELL_ORIGIN;
	grid[theMap.x2idx(target.x) + size_x * theMap.y2idx(target.y)] =
			CELL_TARGET;

	// The main path search loop:
	// -----------------------------------------------------------
	searching = true;	// Will become false on path found.
	notFound = false;// Will be set true inside the loop if a path is not found.

	int range_x_min = min(theMap.x2idx(origin.x) - 1,
			theMap.x2idx(target.x) - 1);
	int range_x_max = max(theMap.x2idx(origin.x) + 1,
			theMap.x2idx(target.x) + 1);
	int range_y_min = min(theMap.y2idx(origin.y) - 1,
			theMap.y2idx(target.y) - 1);
	int range_y_max = max(theMap.y2idx(origin.y) + 1,
			theMap.y2idx(target.y) + 1);

	do {
		notFound = true;
		bool wave1Found = false, wave2Found = false;
		int size_y_1 = size_y - 1;
		int size_x_1 = size_x - 1;
		int longestPathInCellsMetric = 0;

		range_x_min = max(1, range_x_min - 1);
		range_x_max = min(size_x_1, range_x_max + 1);
		range_y_min = max(1, range_y_min - 1);
		range_y_max = min(size_y_1, range_y_max + 1);

		// For all cells(x,y)=EMPTY:
		// -----------------------------
		for (y = range_y_min; y < range_y_max && passCellFound_x == -1; y++) {
			int row = y * size_x;
			int row_1 = (y + 1) * size_x;
			int row__1 = (y - 1) * size_x;
			char metric;// =2 horz.vert, =3 diagonal <-- Since 3/2 ~= sqrt(2)

			for (x = range_x_min; x < range_x_max; x++) {
				if (grid[x + row] == CELL_EMPTY) {
					//  Look in the neighboorhood:
					// -----------------------------
					minNeigh = maxNeigh = CELL_EMPTY;
					metric = 2;
					v = grid[x + row__1];
					if (v + 2 < minNeigh)
						minNeigh = v + 2;
					if (v - 2 > maxNeigh && v != CELL_OBSTACLE)
						maxNeigh = v - 2;
					v = grid[x - 1 + row];
					if (v + 2 < minNeigh)
						minNeigh = v + 2;
					if (v - 2 > maxNeigh && v != CELL_OBSTACLE)
						maxNeigh = v - 2;
					v = grid[x + 1 + row];
					if (v + 2 < minNeigh)
						minNeigh = v + 2;
					if (v - 2 > maxNeigh && v != CELL_OBSTACLE)
						maxNeigh = v - 2;
					v = grid[x + row_1];
					if (v + 2 < minNeigh)
						minNeigh = v + 2;
					if (v - 2 > maxNeigh && v != CELL_OBSTACLE)
						maxNeigh = v - 2;

					v = grid[x - 1 + row__1];
					if ((v + 3) < minNeigh) {
						metric = 3;
						minNeigh = (v + 3);
					}
					if ((v - 3) > maxNeigh && v != CELL_OBSTACLE) {
						metric = 3;
						maxNeigh = v - 3;
					}
					v = grid[x + 1 + row__1];
					if ((v + 3) < minNeigh) {
						metric = 3;
						minNeigh = (v + 3);
					}
					if ((v - 3) > maxNeigh && v != CELL_OBSTACLE) {
						metric = 3;
						maxNeigh = v - 3;
					}
					v = grid[x - 1 + row_1];
					if ((v + 3) < minNeigh) {
						metric = 3;
						minNeigh = (v + 3);
					}
					if ((v - 3) > maxNeigh && v != CELL_OBSTACLE) {
						metric = 3;
						maxNeigh = v - 3;
					}
					v = grid[x + 1 + row_1];
					if ((v + 3) < minNeigh) {
						metric = 3;
						minNeigh = (v + 3);
					}
					if ((v - 3) > maxNeigh && v != CELL_OBSTACLE) {
						metric = 3;
						maxNeigh = v - 3;
					}

					//  Convergence cell found? = The shortest path found
					// -----------------------------------------------------
					if (minNeigh < CELL_EMPTY && maxNeigh > CELL_EMPTY) {
						// Stop the search:
						passCellFound_x = x;
						passCellFound_y = y;
						searching = false;
						longestPathInCellsMetric = 0;
						break;
					} else if (minNeigh < CELL_EMPTY) {
						wave1Found = true;

						// Cell in the expansion-wave from origin
						grid[x + row] = minNeigh + metric;
						ASSERT_(minNeigh+metric<CELL_EMPTY);

						longestPathInCellsMetric = max(longestPathInCellsMetric,
								CELL_EMPTY - minNeigh);
					} else if (maxNeigh > CELL_EMPTY) {
						wave2Found = true;

						// Cell in the expansion-wave from the target
						grid[x + row] = maxNeigh - metric;
						ASSERT_(maxNeigh-metric>CELL_EMPTY);

						longestPathInCellsMetric = max(longestPathInCellsMetric,
								maxNeigh - CELL_EMPTY);
					} else { // Nothing to do: A free cell inside of all also free cells.
					}
				} // if cell empty
			} // end for x
		} // end for y

		notFound = !wave1Found && !wave2Found;

		// Exceeded the max. desired search length??
		if (maxSearchPathLength > 0)
			if (theMap.getResolution() * longestPathInCellsMetric * 0.5f
					> 1.5f * maxSearchPathLength) {
//				printf_debug("[PlannerSimple2D::computePath] Path exceeded desired length! (length=%f m)\n", theMap.getResolution() * longestPathInCellsMetric * 0.5f);
				notFound = true;
			}

	} while (!notFound && searching);

	// Path not found:
	if (notFound)
		return;

	// Rebuild the optimal path from the two-waves convergence cell
	// ----------------------------------------------------------------

	// STEP 1: Trace-back to origin
	//-------------------------------------
	pathcells_x.reserve((minNeigh + 1) + (CELL_TARGET - maxNeigh));	// An (exact?) estimation of the final vector size:
	pathcells_y.reserve((minNeigh + 1) + (CELL_TARGET - maxNeigh));
	x = passCellFound_x;
	y = passCellFound_y;

	while ((v = grid[x + size_x * y]) != CELL_ORIGIN) {
		// Add cell to the path (in inverse order, now we go backward!!) Later is will be reversed
		pathcells_x.push_back(x);
		pathcells_y.push_back(y);

		// Follow the "negative gradient" toward the origin:
		static char dx = 0, dy = 0;
		if ((c = grid[x - 1 + size_x * y]) < v) {
			v = c;
			dx = -1;
			dy = 0;
		}
		if ((c = grid[x + 1 + size_x * y]) < v) {
			v = c;
			dx = 1;
			dy = 0;
		}
		if ((c = grid[x + size_x * (y - 1)]) < v) {
			v = c;
			dx = 0;
			dy = -1;
		}
		if ((c = grid[x + size_x * (y + 1)]) < v) {
			v = c;
			dx = 0;
			dy = 1;
		}

		if ((c = grid[x - 1 + size_x * (y - 1)]) < v) {
			v = c;
			dx = -1;
			dy = -1;
		}
		if ((c = grid[x + 1 + size_x * (y - 1)]) < v) {
			v = c;
			dx = 1;
			dy = -1;
		}
		if ((c = grid[x - 1 + size_x * (y + 1)]) < v) {
			v = c;
			dx = -1;
			dy = 1;
		}
		if ((c = grid[x + 1 + size_x * (y + 1)]) < v) {
			v = c;
			dx = 1;
			dy = 1;
		}

		ASSERT_(dx != 0 || dy != 0);
		x += dx;
		y += dy;
	}

	// STEP 2: Reverse the path, since we want it from the origin
	//   toward the convergence cell
	//--------------------------------------------------------------
	n = pathcells_x.size();
	m = n / 2;
	for (i = 0; i < m; i++) {
		v = pathcells_x[i];
		pathcells_x[i] = pathcells_x[n - 1 - i];
		pathcells_x[n - 1 - i] = v;

		v = pathcells_y[i];
		pathcells_y[i] = pathcells_y[n - 1 - i];
		pathcells_y[n - 1 - i] = v;
	}

	// STEP 3: Trace-foward toward the target
	//-------------------------------------
	x = passCellFound_x;
	y = passCellFound_y;

	while ((v = grid[x + size_x * y]) != CELL_TARGET) {
		// Add cell to the path
		pathcells_x.push_back(x);
		pathcells_y.push_back(y);

		// Follow the "positive gradient" toward the target:
		static char dx = 0, dy = 0;
		c = grid[x - 1 + size_x * y];
		if (c > v && c != CELL_OBSTACLE) {
			v = c;
			dx = -1;
			dy = 0;
		}
		c = grid[x + 1 + size_x * y];
		if (c > v && c != CELL_OBSTACLE) {
			v = c;
			dx = 1;
			dy = 0;
		}
		c = grid[x + size_x * (y - 1)];
		if (c > v && c != CELL_OBSTACLE) {
			v = c;
			dx = 0;
			dy = -1;
		}
		c = grid[x + size_x * (y + 1)];
		if (c > v && c != CELL_OBSTACLE) {
			v = c;
			dx = 0;
			dy = 1;
		}

		c = grid[x - 1 + size_x * (y - 1)];
		if (c > v && c != CELL_OBSTACLE) {
			v = c;
			dx = -1;
			dy = -1;
		}
		c = grid[x + 1 + size_x * (y - 1)];
		if (c > v && c != CELL_OBSTACLE) {
			v = c;
			dx = 1;
			dy = -1;
		}
		c = grid[x - 1 + size_x * (y + 1)];
		if (c > v && c != CELL_OBSTACLE) {
			v = c;
			dx = -1;
			dy = 1;
		}
		c = grid[x + 1 + size_x * (y + 1)];
		if (c > v && c != CELL_OBSTACLE) {
			v = c;
			dx = 1;
			dy = 1;
		}

		ASSERT_(dx != 0 || dy != 0);
		x += dx;
		y += dy;
	}

	// STEP 4: Translate the path-of-cells to a path-of-2d-points with subsampling
	//-------------------------------------------------------------------------------
	path.clear();
	n = pathcells_x.size();
	float xx, yy;
	float last_xx = origin.x, last_yy = origin.y;
	for (i = 0; i < n; i++) {
		// Get cell coordinates:
		xx = theMap.idx2x(pathcells_x[i]);
		yy = theMap.idx2y(pathcells_y[i]);

		// Enough distance??
		if (sqrt(square(xx - last_xx) + square(yy - last_yy))
				> minStepInReturnedPath) {
			// Add to the path:
			path.push_back(CPoint2D(xx, yy));

			// For the next iteration:
			last_xx = xx;
			last_yy = yy;
		}

	}

	// Add the target point:
	path.push_back(CPoint2D(target.x, target.y));

}
