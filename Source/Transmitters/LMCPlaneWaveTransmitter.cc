/*
 * LMCPlaneWaveTransmitter.cc
 *
 *  Created on: May 4, 2019
 *      Author: pslocum
 */

#include "LMCPlaneWaveTransmitter.hh"
#include "logger.hh"

using std::string;


namespace locust
{
    LOGGER( lmclog, "PlaneWaveTransmitter" );

    PlaneWaveTransmitter::PlaneWaveTransmitter():
		fAOI( 0.),
		fAmplitude( 0.0 ),
		fRF_Frequency( 0. ),
        fPatternType( 0 )
    {
    }

    PlaneWaveTransmitter::~PlaneWaveTransmitter()
    {
    }

    bool PlaneWaveTransmitter::Configure( const scarab::param_node& aParam )
    {

		if( aParam.has( "transmitter-frequency" ) )
		{
            fRF_Frequency= aParam["transmitter-frequency"]().as_double();
		}
		if( aParam.has( "AOI" ) )
		{
            fAOI= aParam["AOI"]().as_double()*2.*LMCConst::Pi()/360.;

		}
		if( aParam.has( "planewave-amplitude" ) )
		{
            fAmplitude= aParam["planewave-amplitude"]().as_double();
		}

        if( aParam.has( "pattern-plane" ) )
        {
            if(aParam["pattern-plane"]().as_string() == "xz-plane")
            {
                fPatternType = 0;
            }
            if(aParam["pattern-plane"]().as_string() == "xy-plane")
            {
                fPatternType = 1;
            }
        }


        return true;
    }

    void PlaneWaveTransmitter::AddPropagationPhaseDelay(LMCThreeVector pointOfInterest)
    {
        // Take the end of the array as the first place that the phase front of the planewave touches.
        double planewaveStart;
        double distanceFromCenter = 0.0;

        if(fPatternType == 0)
        {
            planewaveStart = GetFieldPoint(0).GetZ();
            distanceFromCenter = pointOfInterest.GetZ() - planewaveStart;
        }
        if(fPatternType == 1)
        {
            planewaveStart = GetFieldPoint(0).GetY();
            distanceFromCenter = pointOfInterest.GetY() - planewaveStart;
        }
        double phaseDelay = 2*LMCConst::Pi()*distanceFromCenter*sin(fAOI)*fRF_Frequency/LMCConst::C(); 
	   Transmitter::AddPropagationPhaseDelay(phaseDelay);
       printf("phaseDelay for location (%f, %f, %f) is %f\n", pointOfInterest.GetX(), pointOfInterest.GetY(), pointOfInterest.GetZ(), phaseDelay);
    }

    void PlaneWaveTransmitter::AddIncidentKVector(LMCThreeVector pointOfInterest)
    {
        LMCThreeVector incidentKVector(1.0, 0.0, 0.0);

        if(fPatternType == 0)
        {
            incidentKVector.SetComponents(cos(fAOI), 0.0, sin(fAOI));
        }
        if(fPatternType  == 1)
        {
            incidentKVector.SetComponents(cos(fAOI), sin(fAOI), 0.0);
        }
	
	Transmitter::AddIncidentKVector(incidentKVector);
    }

    double* PlaneWaveTransmitter::GetEFieldCoPol(int fieldPointIndex, double dt)
    {
    	double initialPhaseDelay = GetPropagationPhaseDelay(fieldPointIndex); 
		double fieldAmp = fAmplitude;

		if (fieldPointIndex==0) fPhaseDelay += 2. * LMCConst::Pi() * fRF_Frequency * dt;
		//if ( (zIndex == fieldPointIndex0) && (channelIndex == 0) ) fPhaseDelay += 2. * LMCConst::Pi() * fRF_Frequency * dt;
		double fieldValue = fieldAmp*cos(fPhaseDelay + initialPhaseDelay);
		//AddIncidentKVector(pointOfInterest);

        double* fieldSolution = new double[2];
        fieldSolution[0] = fieldValue;
        fieldSolution[1] = 2. * LMCConst::Pi() * fRF_Frequency;  // rad/s

        return fieldSolution;
    }



} /* namespace locust */
