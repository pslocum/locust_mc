/*
 * LMCPlaneWaveTransmitter.hh
 *
 *  Created on: Jan 27, 2020
 *      Author: pslocum
 */

#ifndef LMCPLANEWAVETRANSMITTER_HH_
#define LMCPLANEWAVETRANSMITTER_HH_

#include "LMCTransmitter.hh"
#include "LMCConst.hh"
#include "param.hh"
#include "LMCThreeVector.hh"

namespace locust
{

    /*!
     @class PlaneWaveTransmitter
     @author P. L. Slocum Jan. 27 2020

     @brief Class to generate tranmistter from plane wave

     @details
     Operates in time space

     Configuration name: "plane-wave"

     Available configuration options:
     - "transmitter-frequency": double -- frequency of plane wave in GHz
     - "AOI": double -- angle of incidence between the plane wave normal and the antenna normal. Measured in degrees.
     - "planewave-amplitude": double -- amplitude of plane wave in V/m
     - "pattern-plane": int -- the plane of the resulting pattern of running AOI -89 to 89: in x-z plane or in x-y plane.

     */
    class PlaneWaveTransmitter : public Transmitter
    {
    public:

        PlaneWaveTransmitter();
        virtual ~PlaneWaveTransmitter();

        bool Configure( const scarab::param_node& aNode );

        virtual double* GetEFieldCoPol(int fieldIndexPoint, double dt);


    private:

        double fAOI; // from json file, in degrees.
        double fAmplitude;
        double fRF_Frequency;  // typically defined by a parameter in json file.
        double fPhaseDelay=0.0; //Delay in the phase that changes for each time sample
        int fPatternType; // the plane in which the antenna pattern will be made
        LMCThreeVector fIncidentKVector;  // vector pointing from plane wave to requested point of interest.
        void AddIncidentKVector(LMCThreeVector pointOfInterest);
        void AddPropagationPhaseDelay(LMCThreeVector pointOfInterest);

    };


} /* namespace locust */

#endif /* LMCPLANEWAVETRANSMITTER_HH_ */
