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
    fAOI( 0.),
    fPatchFIRfilter( 0.),
//    gpatchfilter_filename("blank.txt"),
//    fPatchFIRfilter_resolution( 0. ),
    fAmplitude( 0.),
    fFieldBufferMargin( 50 ),
    fRJunction( 0.3 )
  {
    fRequiredSignalState = Signal::kTime;
  }

  PlaneWaveSignalGenerator::~PlaneWaveSignalGenerator()
  {
  }

  bool PlaneWaveSignalGenerator::Configure( const scarab::param_node& aParam )
  {
      
    if(!fReceiverFIRHandler.Configure(aParam,false))
      {
          LERROR(lmclog,"Error configuring receiver FIRHandler class");
      }

    if( aParam.has( "phase-delay" ) )
      {
        SetPhaseDelay( aParam.get_value< bool >( "phase-delay", fPhaseDelay ) );
      }

    if( aParam.has( "voltage-damping" ) )
      {
        SetVoltageDamping( aParam.get_value< bool >( "voltage-damping", fVoltageDamping ) );
      }

    if( aParam.has( "planewave-frequency" ) )
      {
        SetPlaneWaveFrequency( aParam.get_value< double >( "planewave-frequency", fRF_Frequency ));
      }

    if( aParam.has( "lo-frequency" ) )
      {
        SetLOFrequency( aParam.get_value< double >( "fLO_Frequency", fLO_Frequency ));
      }

    if( aParam.has( "array-radius" ) )
      {
        SetArrayRadius( aParam.get_value< double >( "array-radius", fArrayRadius ));
      }
    if( aParam.has( "npatches-per-strip" ) )
      {
        SetNPatchesPerStrip( aParam.get_value< int >( "npatches-per-strip", fNPatchesPerStrip ));
      }
    if( aParam.has( "patch-spacing" ) )
      {
        SetPatchSpacing( aParam.get_value< double >( "patch-spacing", fPatchSpacing ) );
      }
    if( aParam.has( "feed" ) )
      {
        SetPowerCombiner( aParam["feed"]().as_string() );
      }
    if( aParam.has( "AOI" ) )
      {
        SetAOI( aParam.get_value< double >( "AOI", fAOI ));
      }/*
    if ( aParam.has(  "patch-filter"  ) )
      {
        SetPatchFIRfilter( aParam.get_value< bool >( "patch-filter", fPatchFIRfilter) );
      }
    if( aParam.has( "patch-filter-filename" ) )
      {
        SetPatchFIRfilter_filename( aParam.get_value< std::string >( "patch-filter-filename", gpatchfilter_filename) );
      }
    if( aParam.has( "patch-filter-resolution" ) )
      {
        SetPatchFIRfilter_resolution( aParam.get_value< double >( "patch-filter-resolution", fPatchFIRfilter_resolution) );
      }*/
    if( aParam.has( "amplitude" ) )
      {
	  SetAmplitude( aParam.get_value< double >( "amplitude", fAmplitude) );
      }
    if( aParam.has( "buffer-margin" ) )
      {
  	  SetBufferMargin( aParam.get_value< double >( "buffer-margin", fFieldBufferMargin) );
      }
    if( aParam.has( "junction-resistance" ) )
      {
      SetJunctionResistance( aParam.get_value< double >( "junction-resistance", fRJunction) );
      }

    return true;
  }

  bool PlaneWaveSignalGenerator::GetPhaseDelay() const
   {
       return fPhaseDelay;
   }

   void PlaneWaveSignalGenerator::SetPhaseDelay( bool aPhaseDelay )
   {
       fPhaseDelay = aPhaseDelay;
       return;
   }

   bool PlaneWaveSignalGenerator::GetVoltageDamping() const
    {
        return fVoltageDamping;
    }

    void PlaneWaveSignalGenerator::SetVoltageDamping( bool aVoltageDamping )
    {
        fVoltageDamping = aVoltageDamping;
        return;
    }

    double PlaneWaveSignalGenerator::GetPlaneWaveFrequency() const
     {
         return fRF_Frequency;
     }

     void PlaneWaveSignalGenerator::SetPlaneWaveFrequency( double aPlaneWaveFrequency )
     {
         fRF_Frequency = aPlaneWaveFrequency;
         return;
     }

     double PlaneWaveSignalGenerator::GetLOFrequency() const
       {
           return fLO_Frequency;
       }

       void PlaneWaveSignalGenerator::SetLOFrequency( double aLOFrequency )
       {
           fLO_Frequency = aLOFrequency;
           return;
       }

       double PlaneWaveSignalGenerator::GetArrayRadius() const
         {
             return fArrayRadius;
         }

         void PlaneWaveSignalGenerator::SetArrayRadius( double aArrayRadius )
         {
             fArrayRadius = aArrayRadius;
             return;
         }

         int PlaneWaveSignalGenerator::GetNPatchesPerStrip() const
           {
               return fNPatchesPerStrip;
           }

         void PlaneWaveSignalGenerator::SetNPatchesPerStrip( int aNPatchesPerStrip )
           {
               fNPatchesPerStrip = aNPatchesPerStrip;
               return;
           }

         double PlaneWaveSignalGenerator::GetPatchSpacing() const
           {
               return fPatchSpacing;
           }

         void PlaneWaveSignalGenerator::SetPatchSpacing( double aPatchSpacing )
           {
               fPatchSpacing = aPatchSpacing;
               return;
           }

         int PlaneWaveSignalGenerator::GetPowerCombiner() const
           {
               return fPowerCombiner;
           }

         void PlaneWaveSignalGenerator::SetPowerCombiner( std::string feed )
           {
               if (feed == "corporate")
               	fPowerCombiner = 0;  // default
               else if (feed == "series")
               	fPowerCombiner = 1;
               else if (feed == "one-quarter")
               	fPowerCombiner = 2;
               else if (feed == "seven-eighths")
               	fPowerCombiner = 3;
               else if (feed == "nine-sixteenths")
               	fPowerCombiner = 4;
               else if (feed == "voltage-divider")
               	fPowerCombiner = 5;
               else
               	fPowerCombiner = 0;  // default

               return;
           }

         double PlaneWaveSignalGenerator::GetAOI() const
           {
               return fAOI;
           }

         void PlaneWaveSignalGenerator::SetAOI( double aAOI )
           {
               fAOI = aAOI*(2*LMCConst::Pi()/360); //convert to radians
               return;
           }
         double PlaneWaveSignalGenerator::GetAmplitude() const
            {
                return fAmplitude;
            }

          void PlaneWaveSignalGenerator::SetAmplitude( double aAmplitude )
            {
                fAmplitude = aAmplitude;
                return;
            }

/*
         bool PlaneWaveSignalGenerator::GetPatchFIRfilter() const
           {
               return fPatchFIRfilter;
           }

         void PlaneWaveSignalGenerator::SetPatchFIRfilter( bool aPatchFIRfilter )
           {
               fPatchFIRfilter = aPatchFIRfilter;
               return;
           }
         std::string PlaneWaveSignalGenerator::GetPatchFIRfilter_filename() const
            {
                return gpatchfilter_filename;
            }

          void PlaneWaveSignalGenerator::SetPatchFIRfilter_filename( std::string aPatchFIRfilterfilename )
            {
                gpatchfilter_filename = aPatchFIRfilterfilename;
                return;
            }
          double PlaneWaveSignalGenerator::GetPatchFIRfilter_resolution() const
              {
                  return fPatchFIRfilter_resolution;
              }

          void PlaneWaveSignalGenerator::SetPatchFIRfilter_resolution( double aPatchFIRfilter_resolution )
             {
                 fPatchFIRfilter_resolution = aPatchFIRfilter_resolution;
                 return;
             }*/
          double PlaneWaveSignalGenerator::GetBufferMargin() const
              {
                  return fFieldBufferMargin;
              }

          void PlaneWaveSignalGenerator::SetBufferMargin( double aBufferMargin )
             {
                 fFieldBufferMargin = aBufferMargin;
                 return;
             }
          double PlaneWaveSignalGenerator::GetJunctionResistance() const
              {
                  return fRJunction;
              }

          void PlaneWaveSignalGenerator::SetJunctionResistance( double aRJunction )
             {
                 fRJunction = aRJunction;
                 return;
             }






  void PlaneWaveSignalGenerator::Accept( GeneratorVisitor* aVisitor ) const
  {
    aVisitor->Visit( this );
    return;
  }

  double PlaneWaveSignalGenerator::GetAOIFactor(double AOI, LMCThreeVector PatchNormalVector)
  {
    LMCThreeVector IncidentKVector;
    IncidentKVector.SetComponents(cos(AOI), 0.0, sin(AOI));
    double AOIFactor = fabs(IncidentKVector.Dot(PatchNormalVector));
    return AOIFactor;
  }


  double PlaneWaveSignalGenerator::GetVoltageAmpFromPlaneWave(int z_index)
  {
    // This is the method before the patch FIR filter was implemented
    double AntennaFactor = 0.;
    double amplitude = 0.;
    if(z_index == 0 || z_index == fNPatchesPerStrip-1)
      {
	AntennaFactor = 1./430.;
      }
    else
      {
	AntennaFactor = 1./460.;
      }

    // Calculation:
    // S = epsilon0 c E0^2 / 2.  // power/area
    //  0.6e-21 W/Hz * 24.e3 Hz / (0.00375*0.002916) = S = 1.3e-12 W/m^2
    // We should detect 0.6e-21 W/Hz * 24.e3 Hz in Katydid.
    // E0 = sqrt(2.*S/epsilon0/c)
    // effective patch area 0.00004583662 m^2

    double S = 0.6e-21*24.e3/(0.00004271);  // W/m^2, effective aperture.
    double E0 = sqrt(2.*S/(LMCConst::EpsNull() * LMCConst::C()));
    //    double E0 = 1.0; // V/m, test case
    amplitude = E0*AntennaFactor;  // volts
      
    // printf("amplitude is %g\n", amplitude); getchar();
    return amplitude;
  }

  double PlaneWaveSignalGenerator::GetPWPhaseDelayAtPatch(int z_index)
  {
    double phasedelay = 0.;
    if(fAOI >= 0)
      {
	phasedelay = 2*LMCConst::Pi()*z_index*fPatchSpacing*sin(fAOI)*fRF_Frequency/LMCConst::C();
      }
    else
      {
	phasedelay = (fNPatchesPerStrip - z_index)*2*LMCConst::Pi()*fPatchSpacing*sin(fAOI)*fRF_Frequency/LMCConst::C();	
      }
    return phasedelay;
  }
  
    /*
  void PlaneWaveSignalGenerator::ProcessFIRFilter(int nskips)
  {

    FILE *fp;
    double filter;
    double index;
    fp = fopen(gpatchfilter_filename.c_str(),"r");
    int count = 0;


    for (int i=0; i<sizeof(FIR_array)/sizeof(FIR_array[0]); i++)
      FIR_array[i] = -99.;


    while (!feof(fp))
      {
        fscanf(fp, "%lf %lf\n", &index, &filter);
        if (count%nskips==0) FIR_array[count/nskips] = filter;
        count += 1;
      }

    fclose(fp);

  }

  int PlaneWaveSignalGenerator::GetNFilterBins()
  {
    int nbins = 0;
    for (int i=0; i<sizeof(FIR_array)/sizeof(FIR_array[0]); i++)
      {
	if (FIR_array[i]>0.) nbins += 1;

	// TEST
	// printf("FIR_array[%d] is %g\n", i, FIR_array[i]); getchar();
	
      }    
    return nbins;
  }*/

  double PlaneWaveSignalGenerator::GetPatchFIRSample(double dottedamp, double startphase, int patchIndex)
  {   
   
//    double* generatedpoints = new double [nfilterbins];
    std::deque<double> generatedpoints;
    int nfilterbins = fReceiverFIRHandler.GetFilterSize();
    double dtfilter = fReceiverFIRHandler.GetFilterResolution();
//    double dtfilter = fPatchFIRfilter_resolution;
    //double phase = startphase;
    double phase = startphase + GetPWPhaseDelayAtPatch(patchIndex);
    double amp = dottedamp;
    
    for(int i=0; i < nfilterbins; i++)
    {
	generatedpoints.push_back(amp*cos(phase));
	phase += 2*LMCConst::Pi()*dtfilter*fRF_Frequency;

	// TEST PRINT STATEMENT
	//	printf("genpoints %d is %g, amp is %g\n", i, generatedpoints[i], amp); getchar();
    }

    double convolution=fReceiverFIRHandler.ConvolveWithFIRFilter(generatedpoints);
      
    generatedpoints.shrink_to_fit();  // memory deallocation.
//    double total = 0.;
//    for(int j=0; j < nfilterbins; j++)
//      {
//    total += generatedpoints[j]*FIR_array[j];
//      }
//    delete[] generatedpoints;=
    return convolution;
  }
  
  double* PlaneWaveSignalGenerator::GetHilbertMagPhase(unsigned bufferIndex)
  {
    double* magphase = new double[2];
    magphase[0] = 0.;
    magphase[1] = 0.;
    
    if (fabs(PWValueBuffer[bufferIndex].front()) > 0.)
      {
	HilbertTransform aHilbertTransform;
	double* HilbertMagPhaseMean = new double[3];
	
	HilbertMagPhaseMean = aHilbertTransform.GetMagPhaseMean(PWValueBuffer[bufferIndex], PWFreqBuffer[bufferIndex], fFieldBufferMargin, 1.e6*fAcquisitionRate*10);
	magphase[0] = HilbertMagPhaseMean[0];
	magphase[1] = HilbertMagPhaseMean[1];
	delete[] HilbertMagPhaseMean;
    
      }
    return magphase;
  }
  

  // z-index ranges from 0 to npatches-per-strip-1.
  /*
  void PlaneWaveSignalGenerator::AddOnePatchVoltageToStripSum(Signal* aSignal, unsigned bufferIndex, int patchIndex)
  {
    unsigned sampleIndex = SampleIndexBuffer[bufferIndex].front();
    double phi_LO = LOPhaseBuffer[bufferIndex].front();
    double VoltagePhase = 0.; // need to fix this later
    double VoltageAmplitude = PatchVoltageBuffer[bufferIndex].front();
    unsigned z_index = patchIndex;
    double DopplerFrequency = fRF_Frequency;
    
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

    if (fPowerCombiner == 5) // parallel voltage summing method
          {
            VoltageAmplitude *= aPowerCombiner.GetVoltageDividerWeight(fRJunction, 1.0, 10.e6, fNPatchesPerStrip, patchIndex);
          }
    
    // factor of 2 is needed for cosA*cosB = 1/2*(cos(A+B)+cos(A-B)); usually we leave out the 1/2 for e.g. sinusoidal RF.
    
    aSignal->LongSignalTimeComplex()[sampleIndex][0] += VoltageAmplitude * 2. * cos(phi_LO);
    aSignal->LongSignalTimeComplex()[sampleIndex][1] += VoltageAmplitude * 2. * cos(LMCConst::Pi()/2 + phi_LO);



  }
*/


  void* PlaneWaveSignalGenerator::DriveAntenna(int PreEventCounter, unsigned index, Signal* aSignal)
  {     
    unsigned bufferIndex = 0;
    const int signalSize = aSignal->TimeSize();
    const double timeSampleSize = 1./(1.e6 * fAcquisitionRate * aSignal->DecimationFactor());
    unsigned sampleIndex = 0;
    double fieldamp = 0.;
    double fieldphase = 0.;
    double fieldvalue = 0.;
    double* hilbertmagphase = new double[2];
    double phiLO = 0.;

    for(int channelIndex = 0; channelIndex < allChannels.size(); ++channelIndex)
      {
	for(int patchIndex = 0; patchIndex < allChannels[channelIndex].size(); ++patchIndex)
	  {
	    sampleIndex = channelIndex*signalSize*aSignal->DecimationFactor() + index;
	    bufferIndex = channelIndex*fNPatchesPerStrip+patchIndex;

	    phiLO = LOPhaseBuffer[bufferIndex].back();
	    phiLO +=  2. * LMCConst::Pi() * fLO_Frequency * timeSampleSize;
	    
	    PatchAntenna *currentPatch;
	    currentPatch = &allChannels[channelIndex][patchIndex];

	    fieldamp = fAmplitude*GetAOIFactor(fAOI, currentPatch->GetNormalDirection());
	    fieldphase = PWPhaseBuffer[bufferIndex].back();
	    fieldphase += 2. * LMCConst::Pi() * fRF_Frequency * timeSampleSize;
	    // fieldphase += GetPWPhaseDelayAtPatch(patchIndex); this really does not work here for some reason. need to investigate further.
	    fieldvalue = fieldamp*cos(fieldphase);

	    FillBuffers(bufferIndex, sampleIndex, phiLO, fieldphase, fieldvalue);
	    PopBuffers(bufferIndex);

	    hilbertmagphase = GetHilbertMagPhase(bufferIndex);

	    PatchVoltageBuffer[bufferIndex].emplace(PatchVoltageBuffer[bufferIndex].begin()+fFieldBufferMargin+1, GetPatchFIRSample(hilbertmagphase[0], hilbertmagphase[1], patchIndex));
	    PatchVoltageBuffer[bufferIndex].pop_front();
	    PatchVoltageBuffer[bufferIndex].shrink_to_fit();

//	    AddOnePatchVoltageToStripSum(aSignal, bufferIndex, patchIndex);
//        testPowerCombiner.AddOneVoltageToStripSum(aSignal, GetPatchFIRSample(hilbertmagphase[0], hilbertmagphase[1], patchIndex), LOPhaseBuffer[bufferIndex].front();, patchIndex, sampleIndex);
        testPowerCombiner.AddOneVoltageToStripSum(aSignal, PatchVoltageBuffer[bufferIndex].front(), LOPhaseBuffer[bufferIndex].front(), patchIndex, SampleIndexBuffer[bufferIndex].front());
	    
	   
	    // TEST PRINT STATEMENTS
/*
	      printf("Channel is %d\n", channelIndex);
	      printf("Patch is %d\n", patchIndex);
	      printf("Digitizer Sample is %d\n", index);

	      printf("fieldamp is %f\n", fieldamp);
	      printf("fieldphase is %f\n", fieldphase);
	      printf("fieldvalue is %f\n", fieldvalue);
	      printf("hilbertmagphase[0] is %g\n", hilbertmagphase[0]);
	      printf("hilbertmagphase[1] is %g\n", hilbertmagphase[1]);
	    
	      printf("SampleIndexBuffer[%d] is %u\n", bufferIndex, SampleIndexBuffer[bufferIndex].front());
	      printf("LOPhaseBuffer[%d] is %f\n", bufferIndex, LOPhaseBuffer[bufferIndex].front());
	      printf("PWFreqBuffer[%d] is %f\n", bufferIndex, PWFreqBuffer[bufferIndex].front());
	      printf("PWPhaseBuffer[%d] is %f\n", bufferIndex, PWPhaseBuffer[bufferIndex].front());
	      printf("PWValueBuffer[%d] is %f\n", bufferIndex, PWValueBuffer[bufferIndex].front());
	      printf("PatchVoltageBuffer[%d] is %e\n", bufferIndex, PatchVoltageBuffer[bufferIndex].front());
	      printf("Resulting VI[%d] is %e\n", sampleIndex, aSignal->LongSignalTimeComplex()[sampleIndex][0]);
	    
	      getchar();
*/

	    //text file for hilbert transform testing.
	    /*
	      std::ofstream hilbertfile;
	      hilbertfile.open("hilbertfile.txt", std::fstream::app);
	      if(patchIndex == 0){
	      hilbertfile << PWValueBuffer[bufferIndex].front();
	      hilbertfile << ", ";
	      hilbertfile << PWPhaseBuffer[bufferIndex].front();
	      hilbertfile << ", ";
	      hilbertfile << hilbertmagphase[0];
	      hilbertfile << ", ";
	      hilbertfile << hilbertmagphase[1];
	      hilbertfile << "\n";
	      hilbertfile.close();
	      }
	    */
	    
	  }
      }
  }
    
  

  void PlaneWaveSignalGenerator::FillBuffers(unsigned bufferIndex, int digitizerIndex, double phiLO, double pwphase, double pwval)
  {
    SampleIndexBuffer[bufferIndex].push_back(digitizerIndex);
    LOPhaseBuffer[bufferIndex].push_back(phiLO);
    PWFreqBuffer[bufferIndex].push_back(fRF_Frequency);
    PWPhaseBuffer[bufferIndex].push_back(pwphase);
    PWValueBuffer[bufferIndex].push_back(pwval);
  }

  void PlaneWaveSignalGenerator::PopBuffers(unsigned bufferIndex)
  {  
    SampleIndexBuffer[bufferIndex].pop_front();
    LOPhaseBuffer[bufferIndex].pop_front();
    PWFreqBuffer[bufferIndex].pop_front();
    PWPhaseBuffer[bufferIndex].pop_front();
    PWValueBuffer[bufferIndex].pop_front();

    SampleIndexBuffer[bufferIndex].shrink_to_fit();
    LOPhaseBuffer[bufferIndex].shrink_to_fit();
    PWFreqBuffer[bufferIndex].shrink_to_fit();
    PWPhaseBuffer[bufferIndex].shrink_to_fit();
    PWValueBuffer[bufferIndex].shrink_to_fit();
  }
  
  void PlaneWaveSignalGenerator::InitializeBuffers(unsigned fieldbuffersize)
  {
    const unsigned nchannels = fNChannels;
    const int nReceivers = fNPatchesPerStrip;
    
    FieldBuffer aFieldBuffer;

    SampleIndexBuffer = aFieldBuffer.InitializeUnsignedBuffer(nchannels, nReceivers, fieldbuffersize);
    LOPhaseBuffer = aFieldBuffer.InitializeBuffer(nchannels, nReceivers, fieldbuffersize);
    PWFreqBuffer = aFieldBuffer.InitializeBuffer(nchannels, nReceivers, fieldbuffersize);
    PWValueBuffer = aFieldBuffer.InitializeBuffer(nchannels, nReceivers, fieldbuffersize);
    PWPhaseBuffer = aFieldBuffer.InitializeBuffer(nchannels, nReceivers, fieldbuffersize);   
    PatchVoltageBuffer = aFieldBuffer.InitializeBuffer(nchannels, nReceivers, fieldbuffersize);
    
  }



  bool PlaneWaveSignalGenerator::InitializePowerCombining()
  {
  	testPowerCombiner.SetSMatrixParameters(fPowerCombiner, fNPatchesPerStrip);
  	testPowerCombiner.SetVoltageDampingFactors(fPowerCombiner, fNPatchesPerStrip);

  	return true;

  }



  bool PlaneWaveSignalGenerator::InitializePatchArray()
    {
    if(!fReceiverFIRHandler.ReadFIRFile())
    {
        return false;
    }
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
    return true;
  }



  bool PlaneWaveSignalGenerator::DoGenerate( Signal* aSignal )
  {

    InitializePatchArray();
    InitializePowerCombining();

//    // initialize FIR filter array
//    for (unsigned i=0; i < sizeof(FIR_array)/sizeof(FIR_array[0]); i++)
//      {
//    FIR_array[i] = 0.;
//      }
    
//    ProcessFIRFilter(1);
//    nfilterbins = GetNFilterBins();
    int nfilterbins = fReceiverFIRHandler.GetFilterSize();
    int tempfieldbuffersize = 100;
 
    InitializeBuffers(tempfieldbuffersize);

    // text file for VI for testing.
    // note: this is pre-low pass filter. better to put this into LMCDecimateSignalGenerator.cc
    //   std::ofstream voltagefile;
    //    voltagefile.open("voltagefile.txt");

    //n samples for event spacing.
    int PreEventCounter = 0;
    const int NPreEventSamples = 150000;
    PreEventCounter = NPreEventSamples; // jump past wait time.

    for( unsigned index = 0; index < aSignal->DecimationFactor()*aSignal->TimeSize(); ++index )
      {
	DriveAntenna(PreEventCounter, index, aSignal);
	/*
	  voltagefile << index;
	  voltagefile << "\n";
	  voltagefile << aSignal->LongSignalTimeComplex()[index][0];
	  voltagefile << "\n";
	  voltagefile << aSignal->LongSignalTimeComplex()[index][1];
	  voltagefile << "\n";
	*/
      }
    //    voltagefile.close();


    return true;
  }

} /* namespace locust */
