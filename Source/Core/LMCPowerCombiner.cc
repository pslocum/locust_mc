/*
 * LMCPowerCombiner.cc
 *
 *  Created on: Jan 27, 2019
 *      Author: penny
 */

#include "LMCPowerCombiner.hh"
#include <iostream>


namespace locust
{

    PowerCombiner::PowerCombiner()
    {
    }


    PowerCombiner::~PowerCombiner()
    {
    }



    double PowerCombiner::GetVoltageDamping(int njunctions)
    {
    	// test:  clobber voltage with 0.6 at each junction.
      return pow(1.0, njunctions);
    }

    double PowerCombiner::GetLinePhaseCorr(unsigned z_index, double DopplerFrequency)
    {
        int njunctions = z_index;  // series feed
        double D = 0.0068; // m.  18.6 keV 90 degree electron, lambda in kapton.
        double c_n = LMCConst::C()/1.704;  // speed of light in Kapton.
        double lambda = c_n/(DopplerFrequency/(2.*LMCConst::Pi())); // any pitch angle
        double dphi = 2.*LMCConst::Pi() * D * njunctions / lambda;
        return dphi;
    }

  double PowerCombiner::GetCenterFedLinePhaseCorr(int NPatchesPerStrip, unsigned z_index, double DopplerFrequency, double PatchSpacing)
    {
        int njunctions = fabs((double)z_index - (double)NPatchesPerStrip/2.);
        if (z_index >= NPatchesPerStrip/2) njunctions += 1;
        
        double D = PatchSpacing; // m.  18.0 keV 90 degree electron, lambda in kapton.
        double c_n = LMCConst::C()/1.704;  // speed of light in Kapton.
        double lambda = c_n/DopplerFrequency;
        double dphi = 2.*LMCConst::Pi() * D * njunctions / lambda;
	//	printf("2PI*D/lambda is %g\n", 2.*LMCConst::Pi()*D/lambda); getchar();
        return dphi;
    }





}


