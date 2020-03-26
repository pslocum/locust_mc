/*
 * LMCExponentialDistribution.hh
 *
 *  Created on: Mar 23, 2020
 *      Author: nbuzinsky
 */

#ifndef LMCEXPONENTIALDISTRIBUTION_HH_
#define LMCEXPONENTIALDISTRIBUTION_HH_

#include "LMCBaseDistribution.hh"

namespace locust
{
 /*!
 @class ExponentialDistribution
 @author N. Buzinsky
 @brief Derived class for Locust exponential distribution of form p(x) = -lambda * x
 @details
 Available configuration options:
 No input parameters
 */
    //template<class T>
    class ExponentialDistribution : public BaseDistribution
    {

        public:
            ExponentialDistribution(const scarab::param_node &aParam);
            double Generate();

        private:
            std::exponential_distribution<double> fDistribution;
            double fLambda;
};


} /* namespace locust */

#endif /* LMCEXPONENTIALDISTRBUTION_HH_ */
