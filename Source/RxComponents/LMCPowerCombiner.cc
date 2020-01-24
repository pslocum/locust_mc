/*
 * LMCPowerCombiner.cc
 *
 *  Created on: Jan 27, 2019
 *      Author: p. l. slocum
 */

#include "LMCPowerCombiner.hh"
#include <iostream>
#include "logger.hh"



namespace locust
{

	LOGGER( lmclog, "PowerCombiner" );


	PowerCombiner::PowerCombiner():
	fpowerCombiner( 0 ),
	fnPatchesPerStrip( 0 ),
	fnSlots( 0 ),
	fjunctionLoss( 0 ),
	fpatchLoss( 0 ),
	famplifierLoss( 0 ),
	fendPatchLoss( 0 ),
	fdampingFactors( 0 ),
	fjunctionResistance( 0 )
	{
	}


	PowerCombiner::~PowerCombiner()
	{
	}

	bool PowerCombiner::Configure(const scarab::param_node& aParam)
	{

		if( aParam.has( "power-combining-feed" ) )
		{
			SetPowerCombiner(aParam["power-combining-feed"]().as_string());
			return true;
		}

		else
        {
            LWARN( lmclog, "LMCPowerCombiner has been configured without a feed type.  Defaulting to corporate feed.");
            return true;
        }

	}

    bool PowerCombiner::SetPowerCombiner( std::string feed )
    {
    	if (feed == "corporate") fpowerCombiner = 0;  // default
    	else if (feed == "series") fpowerCombiner = 1;
        else if (feed == "one-quarter") fpowerCombiner = 2;
        else if (feed == "seven-eighths") fpowerCombiner = 3;
        else if (feed == "nine-sixteenths") fpowerCombiner = 4;
        else if (feed == "voltage-divider") fpowerCombiner = 5;
        else if (feed == "s-matrix") fpowerCombiner = 6;
        else if (feed == "single-patch") fpowerCombiner = 7;
        else if (feed == "slotted-waveguide") fpowerCombiner = 8;
        else fpowerCombiner = 0;  // default
    	return true;
    }

    int PowerCombiner::GetPowerCombiner()
    {
    	return fpowerCombiner;
    }

    Receiver* PowerCombiner::ChooseElement()
    {
    	PatchAntenna* currentPatch = new PatchAntenna;
    	SlotAntenna* currentSlot = new SlotAntenna;
    	if (fpowerCombiner == 8)
    	{
    		delete currentPatch;
    		return currentSlot;
    	}
    	else
    	{
    		delete currentSlot;
    		return currentPatch;
    	}
    }



	bool PowerCombiner::AddOneVoltageToStripSum(Signal* aSignal, double VoltageFIRSample, double phi_LO, unsigned z_index, unsigned sampleIndex)
	{

		VoltageFIRSample *= fdampingFactors[z_index];
		aSignal->LongSignalTimeComplex()[sampleIndex][0] += 2.*VoltageFIRSample * sin(phi_LO);
		aSignal->LongSignalTimeComplex()[sampleIndex][1] += 2.*VoltageFIRSample * cos(phi_LO);

		return true;
	}





	double PowerCombiner::GetVoltageDividerWeight(double RJunction, double R0, double RGround, unsigned z_index)
	{
		int NPAIRS = fabs((double)z_index - (double)fnPatchesPerStrip/2.);
		if (z_index >= fnPatchesPerStrip/2) NPAIRS += 1; // compensate for patches to the right of amp.
		std::vector<double> D = GetPartialGains(RJunction, R0, RGround, NPAIRS);  // calculate new vector of gains.
		//double dampingfactor = 0.6*0.66;  // patch loss * T-junction loss.
		double dampingfactor = 0.425; // "active S-matrix" for the 2 patch case
		return dampingfactor * D[NPAIRS-1];
	}

	std::vector<double> PowerCombiner::GetResistances(double RJunction, double R0, double RGround, int NPAIRS)
	{
		int NRESISTORS = NPAIRS + 1;
		std::vector<double> R;
		R.resize(NRESISTORS);

		for (unsigned i=0; i<NPAIRS; i++)
		{
			R[i] = R0 + i*RJunction;
		}
		R[NRESISTORS-1] = RGround;

		return R;
	}

	std::vector<double> PowerCombiner::GetPartialGains(double RJunction, double R0, double RGround, int NPAIRS)
	{
		int NRESISTORS = NPAIRS + 1;
		std::vector<double> D;
		D.resize(NRESISTORS);
		std::vector<double> R = GetResistances(RJunction, R0, RGround, NPAIRS);
		double Dtotal = 0.;

		for (int i=0; i<NRESISTORS; i++)
		{
			double ParallelResistance = GetParallelResistance(R, NRESISTORS, i);
			D[i] = ParallelResistance/(ParallelResistance + R[i]);
			Dtotal += D[i];
		}

		return D;
	}
	double PowerCombiner::GetParallelResistance(std::vector<double> R, int NRESISTORS, int resistorindex)
	{
		double OneOverR = 0.;
		for (unsigned i=0; i<NRESISTORS; i++)
		{
			if (i != resistorindex) OneOverR += 1./R[i];
		}
		return 1./OneOverR;
	}

	double PowerCombiner::GetSeriesPhaseDelay(unsigned z_index, double DopplerFrequency, double PatchSpacing)
	{
		int njunctions = z_index;
		double D = PatchSpacing; // m.  18.6 keV 90 degree electron, lambda in kapton.
		double c_n = LMCConst::C()/1.704;  // speed of light in Kapton.
		double lambda = c_n/DopplerFrequency;
		double dphi = 2.*LMCConst::Pi() * D * njunctions / lambda;
		return dphi;
	}


	bool PowerCombiner::SetVoltageDividerDampingFactors()
	{

        for (unsigned z_index=0; z_index<fnPatchesPerStrip; z_index++)
        {
        	int NPAIRS = fabs((double)z_index - (double)fnPatchesPerStrip/2.);
        	if (z_index >= fnPatchesPerStrip/2) NPAIRS += 1; // compensate for patches to the right of amp.
        	std::vector<double> D = GetPartialGains(fjunctionResistance, 1.0, 10.e6, NPAIRS);  // calculate new vector of gains.
        	fdampingFactors[z_index] = fpatchLoss*famplifierLoss * D[NPAIRS-1];  // patch loss * T-junction loss
        }
		
		return true;
	}


	bool PowerCombiner::SetSeriesFedDampingFactors()
	{
		for (unsigned z_index=0; z_index<fnPatchesPerStrip; z_index++)
		{
			int njunctions = z_index;
			fdampingFactors[z_index] = fpatchLoss*famplifierLoss*pow(fjunctionLoss, njunctions);
		}
		return true;
	}

	double PowerCombiner::GetCenterFedPhaseDelay(unsigned z_index, double DopplerFrequency, double PatchSpacing)
	{
		int njunctions = fabs((double)z_index - (double)fnPatchesPerStrip/2.) - 1;
		if (z_index >= fnPatchesPerStrip/2) njunctions += 1; // compensate for patches to the right of amp.
		double D = PatchSpacing; // m.  18.0 keV 90 degree electron, lambda in kapton.
		double c_n = LMCConst::C()/1.704;  // speed of light in Kapton.
		double lambda = c_n/DopplerFrequency;
		double dphi = 2.*LMCConst::Pi() * D * njunctions / lambda;
		return dphi;
	}

	bool PowerCombiner::SetCenterFedDampingFactors()
	{
	    for (unsigned z_index=0; z_index<fnPatchesPerStrip; z_index++)
		{
			int njunctions = fabs((double)z_index - (double)fnPatchesPerStrip/2.) - 1;
			if (z_index >= fnPatchesPerStrip/2) njunctions += 1; // compensate for patches to the right of amp.
			if (z_index == 0 || z_index == fnPatchesPerStrip-1)
	      	{
				fdampingFactors[z_index] = fendPatchLoss*pow(fjunctionLoss, njunctions)*famplifierLoss;
	      	}
			else
	      	 {
				fdampingFactors[z_index] = fpatchLoss*pow(fjunctionLoss, njunctions)*famplifierLoss; // patch loss * junction loss * amplifier loss
	      	 }
		}
	  
		return true;

	}

	bool PowerCombiner::SetSlottedWaveguideDampingFactors()
	{
	    for (unsigned z_index=0; z_index<fnSlots; z_index++)
		{

	    	// uniform taper, normalize with 1/sqrt(fnSlots) so that
	    	// power from N slots scales linearly with N and not N^2.
	    	// This is conceptually similar to the voltage divider in that energy
	    	// conservation is enforced, but is different in that the taper is uniform.

	    	fdampingFactors[z_index] = 1.0 / sqrt(fnSlots);
		}

		return true;

	}


  bool PowerCombiner::SetSmatrixDampingFactors()
  {
    	SetTransmissionCoefficients();
    	for (unsigned z_index=0; z_index<fnPatchesPerStrip; z_index++)
    	{
    		fdampingFactors[z_index] = ftransmissionCoefficients[z_index];
    	}
        return true;

    }

    std::vector<double> PowerCombiner::GetSmatrixElements()
    {
    	std::vector<double> smatrix;
	       
    	if (fnPatchesPerStrip == 2) smatrix = fsMatrix2patch;
    	if (fnPatchesPerStrip == 4) smatrix = fsMatrix4patch;
    	if (fnPatchesPerStrip == 6) smatrix = fsMatrix6patch;
    	if (fnPatchesPerStrip == 8) smatrix = fsMatrix8patch;

        return smatrix;
    }

    bool PowerCombiner::SetTransmissionCoefficients()
    {
    	ftransmissionCoefficients.resize(fnPatchesPerStrip);
	 
    	std::vector<double> smatrix = GetSmatrixElements();
    	double patchImpedance = 100;
    	double ampImpedance = 50;

        for (int i = 0; i < fnPatchesPerStrip; i++)
        {
        	ftransmissionCoefficients[i] = smatrix[i+1]*sqrt(ampImpedance/patchImpedance);
		    //printf("Transmission Coeff for patch  %d is %f\n", i, ftransmissionCoefficients[i]);
        }

        return true;

    }

	bool PowerCombiner::SetVoltageDampingFactors(int anElementsPerStrip, double anElementSpacing)
	{

		if (anElementsPerStrip == 0)
		{
    		LERROR(lmclog,"Please configure nelements-per-strip to something other than zero.");
			return false;
		}

		if ((anElementSpacing == 0.)&&(anElementsPerStrip>1))
		{
    		LERROR(lmclog,"Please configure element-spacing to something other than zero.");
			return false;
		}

		if (fpowerCombiner != 8)
		{
			SetNPatchesPerStrip(anElementsPerStrip);
		}
		else
		{
			SetNSlots(anElementsPerStrip);
		}

		fdampingFactors.resize(anElementsPerStrip);

		if ((fpowerCombiner == 7) || (fpowerCombiner == 0) || (fpowerCombiner == 2) || (fpowerCombiner == 3) || (fpowerCombiner == 4))
		{
			if ( (fpowerCombiner != 7)&&(fnPatchesPerStrip%2 != 0) )
			{
	    		LERROR(lmclog,"Center-fed power combining config expects even number of patches per strip.");
				return false;
			}
			else if ( (fpowerCombiner == 7)&&(fnPatchesPerStrip != 1) )
			{
	    		LERROR(lmclog,"single-patch power combining config expects one patch per strip.");
				return false;
			}
			SetCenterFedDampingFactors();
		}

		else if (fpowerCombiner == 1)  // series
		{
			SetSeriesFedDampingFactors();
		}

		else if (fpowerCombiner == 5)  // voltage divider
		{
			if ( (fnPatchesPerStrip%2 != 0) )
			{
	    		LERROR(lmclog,"Power combining with voltage divider config expects even number of patches per strip.");
				return false;
			}
			SetVoltageDividerDampingFactors();
		}

		else if (fpowerCombiner == 6) // s-matrix
		{
	    	 if (fnPatchesPerStrip != 2 && fnPatchesPerStrip != 4 && fnPatchesPerStrip != 6 && fnPatchesPerStrip != 8)
	    	 {
	    		 LERROR(lmclog,"The S-matrix is implemented only for 2, 4, 6, or 8 patches per strip.");
	    		 return false;
	    	 }
			SetSmatrixDampingFactors();
		}

		else if (fpowerCombiner == 8) // slotted waveguide
		{
			SetSlottedWaveguideDampingFactors();
		}

		return true;
	}


	bool PowerCombiner::SetSMatrixParameters(int anElementsPerStrip)
	{

		if (fpowerCombiner != 8)
		{
			fnPatchesPerStrip = anElementsPerStrip;
		}
		else
		{
			fnSlots = anElementsPerStrip;
		}

		if (fpowerCombiner == 0) // corporate
		{
			fjunctionLoss = 1.0;
			fpatchLoss = 1.0;
			famplifierLoss = 0.425; // hard coded active S-matrix for 2 patches for now
			fendPatchLoss = 1.0;
		}

		else if (fpowerCombiner == 1) // series
		{
			fjunctionLoss = 0.87;
			fpatchLoss = 0.38;
			famplifierLoss = 0.66;
			fendPatchLoss = 0.38;
		}

		else if (fpowerCombiner == 2) // one-quarter
		{
			fjunctionLoss = 0.87;
			fpatchLoss = 0.38;
			famplifierLoss = 0.66;
			fendPatchLoss = 0.95;
		}

		else if (fpowerCombiner == 3) // seven-eighths
		{
			fjunctionLoss = 0.75;
			fpatchLoss = 0.6;
			famplifierLoss = 0.66;
			fendPatchLoss = 0.95;
		}

		else if (fpowerCombiner == 4) // nine-sixteenths
		{
			fjunctionLoss = 0.8;
			fpatchLoss = 0.52;
			famplifierLoss = 0.66;
			fendPatchLoss = 0.95;
		}

		else if (fpowerCombiner == 5) // voltage-divider
		{
			fjunctionResistance = 0.3;
			fpatchLoss = 0.6;
			famplifierLoss = 0.66;
		}

		else if (fpowerCombiner == 6) // s-matrix
		{
			famplifierLoss = 1.0;  // this has no effect.  S-matrix takes its place.
		}

		else if (fpowerCombiner == 7) // single-patch
		{
			// there is a single impedance transformation to the amplifier.
			fjunctionLoss = 1.0;
			fpatchLoss = 1.0;
			famplifierLoss = 0.7071;
			fendPatchLoss = 1.0;
		}

		else if (fpowerCombiner == 8) // slotted-waveguide
		{
			famplifierLoss = 1.0;  // this presently has no effect.
		}

		return true;

	}

	void PowerCombiner::SetNPatchesPerStrip(int aPatchesPerStrip)
	{
		fnPatchesPerStrip = aPatchesPerStrip;
	}

	void PowerCombiner::SetNSlots(int aNSlots)
	{
		fnSlots = aNSlots;
	}

	void PowerCombiner::SetJunctionLoss(double aJunctionLoss)
	{
		fjunctionLoss = aJunctionLoss;
	}

	void PowerCombiner::SetPatchLoss(double aPatchLoss)
	{
		fpatchLoss = aPatchLoss;
	}

	void PowerCombiner::SetAmplifierLoss(double aAmplifierLoss)
	{
		famplifierLoss = aAmplifierLoss;
	}

	void PowerCombiner::SetEndPatchLoss(double aEndPatchLoss)
	{
		fendPatchLoss = aEndPatchLoss;
	}


}


