/*
 * maps_hs.hpp
 *
 *  Created on: 11-Mar-2016
 *      Author: dibyendu ghosh
 *  Organization : Intel Research Lab bangalore
 */
#include<mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <istream>
#ifndef MAPS_HS_HPP
#define MAPS_HS_HPP

class Maps_Hs
{
	public:
	mrpt::maps::CSimplePointsMap maps_obs(mrpt::obs::CObservation2DRangeScan obs_data);


};



#endif /* MAPS_HS_HPP */
