/*
 * LMCExponentialDistribution.cc
 *
 *  Created on: Mar 23, 2020
 *      Author: nbuzinsky
 */

#include "LMCExponentialDistribution.hh"

namespace locust
{
    LOGGER( lmclog, "LMCExponentialDistribution" );

    ExponentialDistribution::ExponentialDistribution(const scarab::param_node &aParam) :
        fLambda( 1. )
    {
        if(aParam.has("lambda"))
            fLambda = aParam.get_value< double >( "lambda", fLambda );

        fDistribution = std::exponential_distribution<double>(fLambda);
        LDEBUG( lmclog, "Created exponential distribution. lambda: " <<fLambda);
    }

    ExponentialDistribution::ExponentialDistribution(const double &aLambda) :
        fLambda( aLambda ),
        fDistribution( aLambda )
    {
    }

    double ExponentialDistribution::Generate()
    {
        return fDistribution(*fRNEngine);
    }

} /* namespace locust */
