/********************************************************
  Autonomous Driving Software
  Copyright (c) 2017 MSC Lab, UC Berkeley 
  All rights reserved.

 ********************************************************/

#include "path_follower/latlongtoUTM.h"
using namespace std;

UTM LatLon2Utm(double Lat, double Lon)
{
	const double pi = 3.1415926535897;

	//WGS84 Parameters
	double WGS84_A = 6378137.0; // major axis
	double WGS84_E = 0.0818191908; // first eccentricity

	//UTM Parameters
	double UTM_K0 = 0.9996; // scale factor
	double UTM_E2 = (WGS84_E * WGS84_E); // e^2

	//Make sure the longitude is between -180 and 179.9
	double LongTemp = (Lon + 180) - floor((Lon + 180) / 360) * 360 -180;

	double LatRad = pi / 180 * Lat;
	double LongRad = pi / 180 * LongTemp;

	double zone = floor((LongTemp + 180) / 6) + 1;

	if (Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
	{
		zone = 32;
	}

	// Special zones for Svalbard
	if (Lat >= 72.0 && Lat < 84.0)
	{
		if (LongTemp >= 0.0 && LongTemp < 9.0)
		{
			zone = 31;
		}
		else if (LongTemp >= 9.0 && LongTemp < 21.0)
        {
        	zone = 33;
        }
    	else if (LongTemp >= 21.0 && LongTemp < 33.0)
        {
        	zone = 35;
        }
    	else if (LongTemp >= 33.0 && LongTemp < 42.0)
        {
        	zone = 37;
        }
	}

	// +3 puts origin in middle of zone
	double LongOrigin = (zone - 1) * 6 - 180 + 3;
	double LongOriginRad = pi / 180 * LongOrigin;

	// compute the UTM Zone from the latitude and longitude
	double eccPrimeSquared = UTM_E2 / (1 - UTM_E2);
	double N = WGS84_A / sqrt(1 - UTM_E2 * sin(LatRad) * sin(LatRad));
	double T = tan(LatRad) * tan(LatRad);
	double C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
	double A = cos(LatRad) * (LongRad - LongOriginRad);
	double M = WGS84_A * ((1 - UTM_E2 / 4 - 3 * UTM_E2 * UTM_E2 / 64 - 5 * UTM_E2 * UTM_E2 * UTM_E2 / 256) * LatRad 
		- (3 * UTM_E2 / 8 + 3 * UTM_E2 * UTM_E2 / 32 + 45 * UTM_E2 * UTM_E2 * UTM_E2 / 1024) * sin(2 * LatRad)
        + (15 * UTM_E2 * UTM_E2 / 256 + 45 * UTM_E2 * UTM_E2 * UTM_E2 / 1024) * sin(4 * LatRad)
        - (35 * UTM_E2 * UTM_E2 * UTM_E2 / 3072) * sin(6 * LatRad));

    double UTMEasting = UTM_K0 * N * (A + (1 - T + C) * A * A * A / 6 
    	+ (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A * A * A * A / 120) + 500000.0;

    double UTMNorthing = UTM_K0 * (M + N * tan(LatRad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24
        + (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A * A * A * A * A * A / 720));

    bool hemi = 0;

    if (Lat < 0)
    {
    	UTMNorthing = UTMNorthing + 10000000.0; //offset for southern hemisphere
    	hemi = 1;
    }
    else
    {
    	hemi = 0;
    }

    UTM current_UTM = {hemi,zone,UTMEasting,UTMNorthing};
    return current_UTM;
}
