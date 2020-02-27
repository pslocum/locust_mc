/*
 * LMCVoltageDivider.cc
 *
 *  Created on: Feb 25, 2020
 *      Author: pslocum
 */

#include "LMCVoltageDivider.hh"


namespace locust
{

    VoltageDivider::VoltageDivider()
    {
    }

    VoltageDivider::~VoltageDivider()
    {
    }

    void VoltageDivider::Initialize()
    {
    	SetSMatrixParameters();
    	SetVoltageDampingFactors();
    }

    void VoltageDivider::SayHello()
    {
    	printf("voltage divider says hello\n"); getchar();
    }


	double VoltageDivider::GetVoltageDividerWeight(double RJunction, double R0, double RGround, unsigned z_index)
	{
		int NPAIRS = fabs((double)z_index - (double)GetNElementsPerStrip()/2.);
		if (z_index >= GetNElementsPerStrip()/2) NPAIRS += 1; // compensate for patches to the right of amp.
		std::vector<double> D = GetPartialGains(RJunction, R0, RGround, NPAIRS);  // calculate new vector of gains.
		//double dampingfactor = 0.6*0.66;  // patch loss * T-junction loss.
		double dampingfactor = 0.425; // "active S-matrix" for the 2 patch case
		return dampingfactor * D[NPAIRS-1];
	}

	std::vector<double> VoltageDivider::GetResistances(double RJunction, double R0, double RGround, int NPAIRS)
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

	std::vector<double> VoltageDivider::GetPartialGains(double RJunction, double R0, double RGround, int NPAIRS)
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

	double VoltageDivider::GetParallelResistance(std::vector<double> R, int NRESISTORS, int resistorindex)
	{
		double OneOverR = 0.;
		for (unsigned i=0; i<NRESISTORS; i++)
		{
			if (i != resistorindex) OneOverR += 1./R[i];
		}
		return 1./OneOverR;
	}


	bool VoltageDivider::SetVoltageDampingFactors()
	{

		for (unsigned z_index=0; z_index<GetNElementsPerStrip(); z_index++)
		{
			printf("check 444\n");
			int NPAIRS = fabs(z_index - GetNElementsPerStrip()/2.);
			if (z_index >= GetNElementsPerStrip()/2) NPAIRS += 1; // compensate for patches to the right of amp.
			std::vector<double> D = GetPartialGains(GetJunctionResistance(), 1.0, 10.e6, NPAIRS);  // calculate new vector of gains.

			double aFactor = GetPatchLoss()*GetAmplifierLoss() * D[NPAIRS-1];  // patch loss * T-junction loss
			printf("elementsperstrip is %d\n", GetNElementsPerStrip());
			SetDampingFactor(z_index, aFactor);

		}

		return true;

	}

	bool VoltageDivider::SetSMatrixParameters()
	{
		SetJunctionResistance( 0.3 );
		SetPatchLoss( 0.6 );
		SetAmplifierLoss( 0.66 );
		return true;
	}




} /* namespace locust */
