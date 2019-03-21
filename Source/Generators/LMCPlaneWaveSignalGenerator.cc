/*
 * LMCPlaneWaveSignalGenerator.cc
 *
 *  Created on: Feb 19, 2018
 *      Author: pslocum
 */

#include "LMCPlaneWaveSignalGenerator.hh"

#include "logger.hh"
#include <thread>
#include <algorithm>

#include <iostream>
#include <fstream>
#include <math.h>       /* sin */
#include "LMCGlobalsDeclaration.hh"
#include "LMCDigitizer.hh"


namespace locust
{
    LOGGER( lmclog, "PlaneWaveSignalGenerator" );

    MT_REGISTER_GENERATOR(PlaneWaveSignalGenerator, "planewave-signal");

    PlaneWaveSignalGenerator::PlaneWaveSignalGenerator( const std::string& aName ) :
        Generator( aName ),
        fLO_Frequency( 0.),
        fRF_Frequency( 0.),
        fArrayRadius( 0. ),
        fPhaseDelay( 0 ),
        fVoltageDamping( 0 ),
        fNPatchesPerStrip( 0. ),
        fPatchSpacing( 0. ),
        fPowerCombiner( 0 ),
        phiLO_t(0.),
        VoltagePhase_t {0.},
    	fAOI( 0.)
    {
        fRequiredSignalState = Signal::kTime;
    }

    PlaneWaveSignalGenerator::~PlaneWaveSignalGenerator()
    {
    }

    bool PlaneWaveSignalGenerator::Configure( const scarab::param_node* aParam )
    {
        if( aParam == NULL) return true;

        if( aParam->has( "phase-delay" ) )
        {
            fPhaseDelay = aParam->get_value< bool >( "phase-delay" );
        }

        if( aParam->has( "voltage-damping" ) )
	  {
            fVoltageDamping = aParam->get_value< bool >( "voltage-damping" );
	  }

        if( aParam->has( "planewave-frequency" ) )
        {
            fRF_Frequency = aParam->get_value< double >( "planewave-frequency" );
        }

        if( aParam->has( "lo-frequency" ) )
        {
            fLO_Frequency = aParam->get_value< double >( "lo-frequency" );
        }
        if( aParam->has( "array-radius" ) )
        {
            fArrayRadius = aParam->get_value< double >( "array-radius" );
        }
        if( aParam->has( "npatches-per-strip" ) )
        {
            fNPatchesPerStrip = aParam->get_value< int >( "npatches-per-strip" );
        }
        if( aParam->has( "patch-spacing" ) )
        {
            fPatchSpacing = aParam->get_value< double >( "patch-spacing" );
        }
        if( aParam->has( "feed" ) )
        {
            if (aParam->get_value< std::string >( "feed" ) == "corporate")
                fPowerCombiner = 0;  // default
            else if (aParam->get_value< std::string >( "feed" ) == "series")
                fPowerCombiner = 1;
            else if (aParam->get_value< std::string >( "feed" ) == "one-quarter")
                fPowerCombiner = 2;
            else if (aParam->get_value< std::string >( "feed" ) == "seven-eighths")
            	fPowerCombiner = 3;
            else if (aParam->get_value< std::string >( "feed") == "nine-sixteenths")
            	fPowerCombiner = 4;
            else
            	fPowerCombiner = 0;  // default
        }
        if( aParam->has( "AOI" ) )
        {
        	fAOI = aParam->get_value< double >( "AOI" );
        	fAOI *= (2*LMCConst::Pi()/360); //convert to radians
        }

        return true;
    }

    void PlaneWaveSignalGenerator::Accept( GeneratorVisitor* aVisitor ) const
    {
        aVisitor->Visit( this );
        return;
    }



    double PlaneWaveSignalGenerator::GetMismatchFactor(double f)
    {
        //f /= 2.*LMCConst::Pi();
        // placeholder = 1 - mag(S11)
        // fit to HFSS output
        //double MismatchFactor = 1. - (-5.39e16 / ((f-25.9141e9)*(f-25.9141e9) + 7.23e16) + 0.88);
        //    printf("dopplerfrequency is %f and mismatchfactor is %g\n", f, MismatchFactor);  getchar();
        double MismatchFactor = 0.85;  // punt.
        return MismatchFactor;
    }
/*
    double PlaneWaveSignalGenerator::GetAOIFactor(LMCThreeVector IncidentKVector, double PatchPhi) const
    {
        LMCThreeVector PatchNormalVector;
        PatchNormalVector.SetComponents(cos(PatchPhi), sin(PatchPhi), 0.0);
        double AOIFactor = fabs(IncidentKVector.Unit().Dot(PatchNormalVector));
        //printf("cos aoi is %f\n", AOIFactor);
        return AOIFactor;
    }
    */

    double PlaneWaveSignalGenerator::GetAOIFactor(double AOI, LMCThreeVector PatchNormalVector){
    	LMCThreeVector IncidentKVector;
    	IncidentKVector.SetComponents(cos(AOI), 0.0, sin(AOI));
    	double AOIFactor = fabs(IncidentKVector.Dot(PatchNormalVector));
    	return AOIFactor;
    }

    double PlaneWaveSignalGenerator::GetVoltageAmpFromPlaneWave()
    {
        double AntennaFactor = 1./400.;

        // S = epsilon0 c E0^2 / 2.  // power/area
        //  0.6e-21 W/Hz * 24.e3 Hz / (0.00375*0.002916) = S = 1.3e-12 W/m^2
        // We should detect 0.6e-21 W/Hz * 24.e3 Hz in Katydid.
        // E0 = sqrt(2.*S/epsilon0/c)
        // effective patch area 0.00004583662 m^2 

        double S = 0.6e-21*24.e3/(0.00004271);  // W/m^2, effective aperture.
        double E0 = sqrt(2.*S/(LMCConst::EpsNull() * LMCConst::C()));
        //    double E0 = 1.0; // V/m, test case
        double amplitude = E0*AntennaFactor;  // volts
        return amplitude;


    }

    double PlaneWaveSignalGenerator::GetVoltageAmpFromPlaneWaveMutualCoupling(int z_index)
      {
    	double AntennaFactor = 0.;
       if(z_index == 0 || z_index == fNPatchesPerStrip)
       {
    	AntennaFactor = 1./430.;
       }
       else
       {
    	AntennaFactor = 1./460.;
       }

          // S = epsilon0 c E0^2 / 2.  // power/area
          //  0.6e-21 W/Hz * 24.e3 Hz / (0.00375*0.002916) = S = 1.3e-12 W/m^2
          // We should detect 0.6e-21 W/Hz * 24.e3 Hz in Katydid.
          // E0 = sqrt(2.*S/epsilon0/c)
          // effective patch area 0.00004583662 m^2

          double S = 0.6e-21*24.e3/(0.00004271);  // W/m^2, effective aperture.
          double E0 = sqrt(2.*S/(LMCConst::EpsNull() * LMCConst::C()));
          //    double E0 = 1.0; // V/m, test case
          double amplitude = E0*AntennaFactor;  // volts
          return amplitude;


      }


    // z-index ranges from 0 to npatches-per-strip-1.
    void PlaneWaveSignalGenerator::AddOnePatchVoltageToStripSum(Signal* aSignal, double VoltageAmplitude, double VoltagePhase, double phi_LO, unsigned sampleIndex, unsigned z_index, double DopplerFrequency)
    {


    	PowerCombiner aPowerCombiner;
    	if (fPowerCombiner == 0 ) //corporate feed, for testing
    	{
    		if (fVoltageDamping)
    		{
    			VoltageAmplitude *= aPowerCombiner.GetCorporateVoltageDamping();
    		}
    	}

    	if (fPowerCombiner == 1)  // series feed
        {
    		if (fPhaseDelay) // parameter from json
    		{
    			VoltagePhase += aPowerCombiner.GetSeriesPhaseDelay(z_index, DopplerFrequency, fPatchSpacing);
    		}
    		if (fVoltageDamping)  // parameter from json
    		{
    			VoltageAmplitude *= aPowerCombiner.GetSeriesVoltageDamping(z_index);
    		}
        }

    	if (fPowerCombiner == 2) // one-quarter power combining, center fed strip
        {
            if (fPhaseDelay)
            {
            	VoltagePhase += aPowerCombiner.GetCenterFedPhaseDelay(fNPatchesPerStrip, z_index, DopplerFrequency, fPatchSpacing);
            }
            if (fVoltageDamping)
            {
                VoltageAmplitude *= aPowerCombiner.GetOneQuarterVoltageDamping(fNPatchesPerStrip, z_index);
            }
        }

    	if (fPowerCombiner == 3) // seven-eighths power combining, center fed strip
    	{
    		if (fPhaseDelay)
    		{
            	VoltagePhase += aPowerCombiner.GetCenterFedPhaseDelay(fNPatchesPerStrip, z_index, DopplerFrequency, fPatchSpacing);
    		}
    		if (fVoltageDamping)
    		{
    			VoltageAmplitude *= aPowerCombiner.GetSevenEighthsVoltageDamping(fNPatchesPerStrip, z_index);
    		}
    	}

    	if (fPowerCombiner == 4) // nine-sixteenths power combining, center fed strip
    	{
    		if (fPhaseDelay)
    		{
    			VoltagePhase += aPowerCombiner.GetCenterFedPhaseDelay(fNPatchesPerStrip, z_index, DopplerFrequency, fPatchSpacing);
    		}
    		if (fVoltageDamping)
    		{
    			VoltageAmplitude *= aPowerCombiner.GetNineSixteenthsVoltageDamping(fNPatchesPerStrip, z_index);
    		}
    	}


        aSignal->LongSignalTimeComplex()[sampleIndex][0] += VoltageAmplitude * cos(VoltagePhase - phi_LO);
        aSignal->LongSignalTimeComplex()[sampleIndex][1] += VoltageAmplitude * sin(VoltagePhase - phi_LO);
        
//  if (VoltageAmplitude>0.) {printf("summedvoltageamplitude is %g\n", aSignal->LongSignalTimeComplex()[sampleIndex][0]); getchar();}
// printf("Voltage Amplitude is %g\n", VoltageAmplitude); getchar();

    }


    void* PlaneWaveSignalGenerator::DriveAntenna(int PreEventCounter, unsigned index, Signal* aSignal)
    {

        const int signalSize = aSignal->TimeSize();
        unsigned sampleIndex = 0;

        //Receiver Properties
        phiLO_t += 2. * LMCConst::Pi() * fLO_Frequency / (1.e6*fAcquisitionRate*aSignal->DecimationFactor());

        PatchAntenna *currentPatch;
        double timedelay;
        for(int channelIndex = 0; channelIndex < allChannels.size(); ++channelIndex)
        {
            for(int patchIndex = 0; patchIndex < allChannels[channelIndex].size(); ++patchIndex)
            {
                currentPatch = &allChannels[channelIndex][patchIndex]; 
                sampleIndex = channelIndex*signalSize*aSignal->DecimationFactor() + index;  // which channel and which sample

                VoltagePhase_t[channelIndex*fNPatchesPerStrip+patchIndex] +=
 2. * LMCConst::Pi() * fRF_Frequency / (1.e6 * fAcquisitionRate * aSignal->DecimationFactor()); // phi =+ f*dt

                // arrival time delay for skewed angle of incidence
                   if(index == 0){
                	timedelay = patchIndex*fPatchSpacing*sin(fAOI)/LMCConst::C();
                //	printf("timedelay is %e\n", timedelay);
                	VoltagePhase_t[channelIndex*fNPatchesPerStrip+patchIndex] -= 2.*LMCConst::Pi()*fRF_Frequency*timedelay;
                   }


               //  printf("voltagephase_t for patch %d is %g\n", patchIndex, VoltagePhase_t[channelIndex*fNPatchesPerStrip+patchIndex]); getchar();

               // double tVoltageAmplitude = GetVoltageAmpFromPlaneWave();
                double tVoltageAmplitude = GetVoltageAmpFromPlaneWaveMutualCoupling(patchIndex);

                tVoltageAmplitude *= GetAOIFactor(fAOI, currentPatch->GetNormalDirection());
             //   printf("aoi factor is %f\n", GetAOIFactor(fAOI, currentPatch->GetNormalDirection())); getchar();
            //    printf("amp is %g\n", tVoltageAmplitude); getchar();

                AddOnePatchVoltageToStripSum(aSignal, tVoltageAmplitude, VoltagePhase_t[channelIndex*fNPatchesPerStrip+patchIndex], phiLO_t, sampleIndex, patchIndex, fRF_Frequency);

            } // patch loop
            //printf("VI for time %d is %g\n", index, aSignal->LongSignalTimeComplex()[sampleIndex][0]);
                          //   getchar();


        } // channels loop

        return 0;
    }


    void PlaneWaveSignalGenerator::InitializePatchArray()
    {

        const unsigned nChannels = fNChannels;
        const int nReceivers = fNPatchesPerStrip;

        const double patchSpacingZ = fPatchSpacing;
        const double patchRadius = fArrayRadius;
        double zPosition;
        double theta;
        const double dThetaArray = 2. * LMCConst::Pi() / nChannels; //Divide the circle into nChannels

        PatchAntenna modelPatch;

        allChannels.resize(nChannels);

        for(int channelIndex = 0; channelIndex < nChannels; ++channelIndex)
        {
            theta = channelIndex * dThetaArray;

            for(int receiverIndex = 0; receiverIndex < nReceivers; ++receiverIndex)
            {
                zPosition =  (receiverIndex - (nReceivers - 1.) /2.) * patchSpacingZ;

                modelPatch.SetCenterPosition({patchRadius * cos(theta) , patchRadius * sin(theta) , zPosition }); 
                modelPatch.SetPolarizationDirection({sin(theta), -cos(theta), 0.}); 
                modelPatch.SetNormalDirection({-cos(theta), -sin(theta), 0.}); //Say normals point inwards
                allChannels[channelIndex].AddReceiver(modelPatch);
            }
        }
    }



    bool PlaneWaveSignalGenerator::DoGenerate( Signal* aSignal )
    {

        InitializePatchArray();

        //text file for VI for testing.
        std::ofstream voltagefile;
        voltagefile.open("voltagefile.txt");

        //n samples for event spacing.
        int PreEventCounter = 0;
        const int NPreEventSamples = 150000;
        PreEventCounter = NPreEventSamples; // jump past wait time.
        for (unsigned i=0; i < sizeof(VoltagePhase_t)/sizeof(VoltagePhase_t[0]); i++)
            {  // initialize voltage phases.
                VoltagePhase_t[i] = {0.0};
            }

        for( unsigned index = 0; index < aSignal->DecimationFactor()*aSignal->TimeSize(); ++index )
          {
          DriveAntenna(PreEventCounter, index, aSignal);
          voltagefile << index;
          voltagefile << "\n";
          voltagefile << aSignal->LongSignalTimeComplex()[index][0];
          voltagefile << "\n";
          voltagefile << aSignal->LongSignalTimeComplex()[index][1];
          voltagefile << "\n";
          }
        voltagefile.close();


        return true;
    }

} /* namespace locust */
