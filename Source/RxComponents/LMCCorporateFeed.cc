/*
 * LMCCorporateFeed.cc
 *
 *  Created on: Feb 25, 2020
 *      Author: pslocum
 */

#include "LMCCorporateFeed.hh"
using std::string;


namespace locust
{

	LOGGER( lmclog, "CorporateFeed" );

    CorporateFeed::CorporateFeed()
    {
    }

    CorporateFeed::~CorporateFeed()
    {
    }

    bool CorporateFeed::Configure( const scarab::param_node& aParam )
    {

    	if( !PowerCombiner::Configure(aParam))
    	{
    		LERROR(lmclog,"Error configuring PowerCombiner class from CorporateFeed subclass");
    		return false;
    	}

    	if (GetNElementsPerStrip()%2 != 0)
    	{
    		LERROR(lmclog,"Center-fed power combining config expects even number of patches per strip.");
    		return false;
    	}

		SetAmplifierLoss(0.425); // hard coded active S-matrix for 2 patches
    	SetVoltageDampingFactors();
    	return true;
    }



	bool CorporateFeed::SetVoltageDampingFactors()
	{

		for (unsigned z_index=0; z_index<GetNElementsPerStrip(); z_index++)
		{
			double aFactor = GetAmplifierLoss();
			SetDampingFactor(z_index, aFactor);
		}

		return true;

	}




} /* namespace locust */
