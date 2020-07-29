/*
 * LMCPlanarArrayPositioner.cc
 *
 *  Created on: July 23, 2020
 *      Author: atelles and pslocum
 */

#include "LMCPlanarArrayPositioner.hh"
using std::string;


namespace locust
{

	LOGGER( lmclog, "PlanarArrayPositioner" );

    PlanarArrayPositioner::PlanarArrayPositioner()
    {
    }

    PlanarArrayPositioner::~PlanarArrayPositioner()
    {
    }

    bool PlanarArrayPositioner::Configure( const scarab::param_node& aParam )
    {

    	if( !AntennaElementPositioner::Configure(aParam))
    	{
    		LERROR(lmclog,"Error configuring AntennaElementPositioner class from PlanarArrayPositioner subclass");
    	}

    	return true;
    }
    using AntennaElementPositioner::GetPositionZ;
    double PlanarArrayPositioner::GetPositionZ(double zShiftArray, int channelIndex, int nChannels,
            int nSubarrays, int nReceivers, double elementSpacingZ, int receiverIndex, int nPlanarArrayRows)
    {
        double zPosition = zShiftArray + 
                        (int(channelIndex/(nChannels/nSubarrays))-((nSubarrays -1.)/2.))*(nReceivers/2.)*elementSpacingZ +
                        ((receiverIndex % (nReceivers/nPlanarArrayRows)) - ((nReceivers/nPlanarArrayRows) - 1.) /2.) * elementSpacingZ;
                    
        return zPosition;
    }

    double PlanarArrayPositioner::GetTheta(int channelIndex, int dThetaArray, int receiverIndex, int nReceivers, int nPlanarArrayRows, double planarRowSpacing, double elementRadius)
    {
        double tTheta = channelIndex * dThetaArray;
        double thetaAdjust = atan(((int(receiverIndex/(nReceivers/nPlanarArrayRows)) - ((nPlanarArrayRows - 1.)/2.))*planarRowSpacing)/elementRadius);
        tTheta += thetaAdjust;
        return tTheta;
    }





} /* namespace locust */
