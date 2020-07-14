/*
 * LMCPlanarArray.cc
 *
 *  Created on: Jul 13, 2020
 *      Author: A. B. Telles
 */

#include "LMCPlanarArray.hh"
using std::string;


namespace locust
{

	LOGGER( lmclog, "PlanarArray" );

    PlanarArray::PlanarArray():
    		fImpedanceTransformation( 0.339 ) // sqrt(50./435.) unless already included in HFSS TF.
    {
    }

    PlanarArray::~PlanarArray()
    {
    }

    bool PlanarArray::Configure( const scarab::param_node& aParam )
    {

    	if( !PowerCombiner::Configure(aParam))
    	{
    		LERROR(lmclog,"Error configuring PowerCombiner class from PlanarArray subclass");
    		return false;
    	}
/*
        if( aParam.has( "impedance-transformation" ) )
        {
            //fImpedanceTransformation = aParam["impedance-transformation"]().as_double();
            temp = aParam["impedance-transformation"]().as_double();
        }
*/
    	SetVoltageDampingFactors();
    	return true;
    }



	bool PlanarArray::SetVoltageDampingFactors()
	{

		for (unsigned z_index=0; z_index<GetNElementsPerStrip(); z_index++)
		{
			double aFactor = fImpedanceTransformation;
			//Adhoc scaling in case the slotted waveguide has fewer/more slots than 10
			aFactor = aFactor/sqrt(GetNElementsPerStrip()/10.);
			SetDampingFactor(z_index, aFactor);
		}

		return true;

	}

    Receiver* PlanarArray::ChooseElement()
    {
    	SlotAntenna* aSlot = new SlotAntenna;
    	return aSlot;
    }



} /* namespace locust */
