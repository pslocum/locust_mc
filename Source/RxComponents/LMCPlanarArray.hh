/*
 * LMCPlanarArray.hh
 *
 *  Created on: Jul 13, 2020
 *      Author: A. B. Telles
 */

#ifndef LMCPLANARARRAY_HH_
#define LMCPLANARARRAY_HH_

#include "LMCPowerCombiner.hh"
#include "param.hh"
#include "logger.hh"
#include <iostream>

namespace locust
{
 /*!
 @class PlanarArray
 @author A. B. Telles
 @brief Derived class describing the slotted planar array
 @details
 Available configuration options:
 No input parameters
 */


    class PlanarArray: public PowerCombiner
    {

        public:
            PlanarArray();
            virtual ~PlanarArray();
            virtual bool Configure( const scarab::param_node& aNode );

        	virtual bool SetVoltageDampingFactors();
            virtual Receiver* ChooseElement();


        private:
        	double fImpedanceTransformation;  // != 1.0 either in Locust or in HFSS, but not both.


    };


} /* namespace locust */

#endif /* LMCPLANARARRAY_HH_ */
