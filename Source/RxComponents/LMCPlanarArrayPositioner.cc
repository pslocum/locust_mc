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

    PlanarArrayPositioner::PlanarArrayPositioner():
        fNPlanarArrayRows( 2 ),
        fPlanarArraySpacing( 0.009636 ), // distance between adjacent planar array rows.
        fThetaAdjustment( 0.0 )
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

        if( aParam.has( "planar-array-spacing" ) )
        {
            fPlanarArraySpacing = aParam["planar-array-spacing"]().as_double();
        }

        if( aParam.has( "n-planar-array-rows" ) )
        {
            fNPlanarArrayRows = aParam["n-planar-array-rows"]().as_int();
        }

    	return true;
    }

    bool PlanarArrayPositioner::IsPlanarArray()
    {

        return true;
    }

    double PlanarArrayPositioner::GetPositionZ(double zShiftArray, int channelIndex, int nChannels,
            int nSubarrays, int nReceivers, double elementSpacingZ, int receiverIndex)
    {
        double zPosition = zShiftArray + 
                        (int(channelIndex/(nChannels/nSubarrays))-((nSubarrays -1.)/2.))*(nReceivers/2.)*elementSpacingZ +
                        ((receiverIndex % (nReceivers/fNPlanarArrayRows)) - ((nReceivers/fNPlanarArrayRows) - 1.) /2.) * elementSpacingZ;
                    
        return zPosition;
    }

    double PlanarArrayPositioner::GetTheta(int channelIndex, int dThetaArray, int receiverIndex, int nReceivers, double elementRadius)
    {
        double tTheta = channelIndex * dThetaArray;
        SetThetaAdjustment(receiverIndex, nReceivers, elementRadius);
        tTheta += fThetaAdjustment;
        return tTheta;
    }

    void PlanarArrayPositioner::SetThetaAdjustment(int receiverIndex, int nReceivers, double elementRadius)
    {
        fThetaAdjustment = atan(((int(receiverIndex/(nReceivers/fNPlanarArrayRows)) - ((fNPlanarArrayRows - 1.)/2.))*fPlanarArraySpacing)/elementRadius);
    }

    void PlanarArrayPositioner::PlaceElement(Receiver &modelElement, double elementRadius, double theta, double zPosition)
    {
        double centralTheta = theta - fThetaAdjustment;
        modelElement.SetCenterPosition({elementRadius * cos(theta) , elementRadius * sin(theta) , zPosition });
        modelElement.SetPolarizationDirection({sin(centralTheta), -cos(centralTheta), 0.0});
        modelElement.SetCrossPolarizationDirection({0.0, 0.0, 1.0});  // longitudinal axis of array.
        modelElement.SetNormalDirection({-cos(centralTheta), -sin(centralTheta), 0.0}); //Say normals point inwards
    }


} /* namespace locust */
