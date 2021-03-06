/*
 * LMCGaussianDistribution.hh
 *
 *  Created on: Mar 12, 2020
 *      Author: nbuzinsky
 */

#ifndef LMCGAUSSIANDISTRIBUTION_HH_
#define LMCGAUSSIANDISTRIBUTION_HH_

#include "LMCBaseDistribution.hh"

namespace locust
{
 /*!
 @class GaussianDistribution
 @author N. Buzinsky
 @brief Derived class for Locust gaussian distribution called with [mean, std-dev]. Type can be double or int.
 @details
 Available configuration options:
 No input parameters
 */
    //template<class T>
    class GaussianDistribution : public BaseDistribution
    {

        public:
            GaussianDistribution(const scarab::param_node &aParam);
            GaussianDistribution(const double &aMean, const double &aStdDev);
            double Generate();

        private:
            std::normal_distribution<double> fDistribution;
            double fMean;
            double fStdDev;
};


} /* namespace locust */

#endif /* LMCGAUSSIANDISTRBUTION_HH_ */
