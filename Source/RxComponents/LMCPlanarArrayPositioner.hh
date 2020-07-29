/*
 * LMCPlanarArrayPositioner.hh
 *
 *  Created on: July 23, 2020
 *      Author: atelles and pslocum
 */

#ifndef LMCPLANARARRAYPOSITIONER_HH_
#define LMCPLANARARRAYPOSITIONER_HH_

#include "LMCAntennaElementPositioner.hh"
#include "param.hh"
#include "logger.hh"
#include <iostream>

namespace locust
{
 /*!
 @class PlanarArrayPositioner
 @author A. Telles and P. Slocum
 @brief Derived class describing the positioning of elements in a planar array antenna.
 @details
 Available configuration options:
 No input parameters
 */


    class PlanarArrayPositioner: public AntennaElementPositioner
    {

        public:
            PlanarArrayPositioner();
            virtual ~PlanarArrayPositioner();
            virtual bool Configure( const scarab::param_node& aNode );
            using AntennaElementPositioner::GetPositionZ;
            virtual double GetPositionZ(double zShiftArray, int channelIndex, int nChannels,
            		int nSubarrays, int nReceivers, double elementSpacingZ, int receiverIndex, int nPlanarArrayRows);
          
            virtual double GetTheta(int channelIndex, int dThetaArray, int receiverIndex, int nReceivers, int nPlanarArrayRows, double planarRowSpacing, double elementRadius);

    };


} /* namespace locust */

#endif /* LMCPLANARARRAYPOSITIONER_HH_ */
