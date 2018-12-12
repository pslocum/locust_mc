/*
 * LMCFakeTrackSignalGenerator.cc
 *
 *  Created on: Aug 8 2018
 *      Author: plslocum
 */

#include <cmath>
#include "LMCFakeTrackSignalGenerator.hh"
#include "LMCDigitizer.hh"
#include "logger.hh"
#include "LMCConst.hh"
#include <random>
#include <math.h>

using std::string;

namespace locust
{
    LOGGER( lmclog, "FakeTrackSignalGenerator" );

    MT_REGISTER_GENERATOR(FakeTrackSignalGenerator, "fake-track");

    FakeTrackSignalGenerator::FakeTrackSignalGenerator( const std::string& aName ) :
        Generator( aName ),
        fDoGenerateFunc( &FakeTrackSignalGenerator::DoGenerateTime ),
        fSignalPower( 0. ),
        fStartFrequencyMax( 0. ),
        fStartFrequencyMin( 0. ),
        fStartVPhase( 0. ),
        fSlopeMean( 0. ),
        fSlopeStd( 0. ),
        fStartTimeMin( 0. ),
        fStartTimeMax( 0. ),
        fLO_frequency( 0. ),
        fTrackLengthMean( 0. ),
        fNTracksMean(1 ),
        fBField(1.0),
        fRandomSeed(0)
    {
        fRequiredSignalState = Signal::kTime;
    }

    FakeTrackSignalGenerator::~FakeTrackSignalGenerator()
    {
    }


    bool FakeTrackSignalGenerator::Configure( const scarab::param_node* aParam )
    {
        if( aParam == NULL) return true;

        if( aParam->has( "signal-power" ) )
            SetSignalPower( aParam->get_value< double >( "signal-power", fSignalPower ) );

        if( aParam->has( "start-frequency-max" ) )
            SetStartFrequencyMax( aParam->get_value< double >( "start-frequency-max", fStartFrequencyMax ) );

        if( aParam->has( "start-frequency-min" ) )
            SetStartFrequencyMin( aParam->get_value< double >( "start-frequency-min", fStartFrequencyMin ) );

        if( aParam->has( "start-vphase" ) )
            SetStartVPhase( aParam->get_value< double >( "start-vphase", fStartVPhase ) );

        if( aParam->has( "slope-mean" ) )
            SetSlopeMean( aParam->get_value< double >( "slope-mean", fSlopeMean ) );

        if( aParam->has( "slope-std" ) )
            SetSlopeStd( aParam->get_value< double >( "slope-std", fSlopeStd ) );

        if( aParam->has( "start-time-max" ) )
            SetStartTimeMax( aParam->get_value< double >( "start-time-max", fStartTimeMax ) );

        if( aParam->has( "start-time-min" ) )
            SetStartTimeMin( aParam->get_value< double >( "start-time-min", fStartTimeMin ) );

        if( aParam->has( "lo-frequency" ) )
            SetFrequency( aParam->get_value< double >( "lo-frequency", fLO_frequency ) );

        if( aParam->has( "track-length-mean" ) )
            SetTrackLengthMean( aParam->get_value< double >( "track-length-mean", fTrackLengthMean ) );

        if (aParam->has( "ntracks-mean") )
            SetNTracksMean( aParam->get_value< double >( "ntracks-mean",fNTracksMean) );

        if (aParam->has("magnetic-field") )
            SetBField(  aParam->get_value< double >("magnetic-field", fBField) );

        if (aParam->has( "random-seed") )
            SetRandomSeed(  aParam->get_value< int >( "random-seed",fRandomSeed) );



        if( aParam->has( "domain" ) )
        {
            string domain = aParam->get_value( "domain" );
            if( domain == "time" )
            {
                SetDomain( Signal::kTime );
                LDEBUG( lmclog, "Domain is equal to time.");
            }
            else if( domain == "freq" )
            {
                SetDomain( Signal::kFreq );
            }
            else
            {
                LERROR( lmclog, "Unable to use domain requested: <" << domain << ">" );
                return false;
            }
        }


        return true;



    }


    void FakeTrackSignalGenerator::Accept( GeneratorVisitor* aVisitor ) const
    {
        aVisitor->Visit( this );
        return;
    }

    double FakeTrackSignalGenerator::GetSignalPower() const
    {
        return fSignalPower;
    }

    void FakeTrackSignalGenerator::SetSignalPower( double aPower )
    {
        fSignalPower = aPower;
        return;
    }

    double FakeTrackSignalGenerator::GetStartFrequencyMax() const
    {
        return fStartFrequencyMax;
    }

    void FakeTrackSignalGenerator::SetStartFrequencyMax( double aFrequencyMax )
    {
        fStartFrequencyMax = aFrequencyMax;
        return;
    }

    double FakeTrackSignalGenerator::GetStartFrequencyMin() const
    {
        return fStartFrequencyMin;
    }

    void FakeTrackSignalGenerator::SetStartFrequencyMin( double aFrequencyMin )
    {
        fStartFrequencyMin = aFrequencyMin;
        return;
    }

    double FakeTrackSignalGenerator::GetStartVPhase() const
    {
        return fStartVPhase;
    }

    void FakeTrackSignalGenerator::SetStartVPhase( double aPhase )
    {
        fStartVPhase = aPhase;
        return;
    }

    double FakeTrackSignalGenerator::GetSlopeMean() const
    {
        return fSlopeMean;
    }

    void FakeTrackSignalGenerator::SetSlopeMean( double aSlopeMean )
    {
        fSlopeMean = aSlopeMean;
        return;
    }

    double FakeTrackSignalGenerator::GetSlopeStd() const
    {
        return fSlopeStd;
    }

    void FakeTrackSignalGenerator::SetSlopeStd( double aSlopeStd )
    {
        fSlopeStd = aSlopeStd;
        return;
    }

    double FakeTrackSignalGenerator::GetTrackLengthMean() const
    {
        return fTrackLengthMean;
    }

    void FakeTrackSignalGenerator::SetTrackLengthMean( double aTrackLengthMean )
    {
        fTrackLengthMean = aTrackLengthMean;
        return;
    }

    double FakeTrackSignalGenerator::GetStartTimeMin() const
    {
        return fStartTimeMin;
    }

    void FakeTrackSignalGenerator::SetStartTimeMin( double aTimeMin )
    {
        fStartTimeMin = aTimeMin;
        return;
    }

    double FakeTrackSignalGenerator::GetStartTimeMax() const
    {
        return fStartTimeMax;
    }

    void FakeTrackSignalGenerator::SetStartTimeMax( double aTimeMax )
    {
        fStartTimeMax = aTimeMax;
        return;
    }

    double FakeTrackSignalGenerator::GetFrequency() const
    {
        return fLO_frequency;
    }

    void FakeTrackSignalGenerator::SetFrequency( double aFrequency )
    {
        fLO_frequency = aFrequency;
        return;
    }

    double FakeTrackSignalGenerator::GetNTracksMean() const
    {
        return fNTracksMean;
    }

    void FakeTrackSignalGenerator::SetNTracksMean( double aNTracksMean )
    {
        fNTracksMean = aNTracksMean;
        return;
    }

    double FakeTrackSignalGenerator::GetBField() const
    {
        return fBField;
    }

    void FakeTrackSignalGenerator::SetBField( double aBField )
    {
        fBField = aBField;
        return;
    }

    int FakeTrackSignalGenerator::GetRandomSeed() const
    {
        return fRandomSeed;
    }

    void FakeTrackSignalGenerator::SetRandomSeed( int aRandomSeed )
    {
        fRandomSeed = aRandomSeed;
        return;
    }


    Signal::State FakeTrackSignalGenerator::GetDomain() const
    {
        return fRequiredSignalState;
    }

    void FakeTrackSignalGenerator::SetDomain( Signal::State aDomain )
    {
        if( aDomain == fRequiredSignalState ) return;
        fRequiredSignalState = aDomain;  // pls changed == to =.
        if( fRequiredSignalState == Signal::kTime )
        {
            fDoGenerateFunc = &FakeTrackSignalGenerator::DoGenerateTime;
        }
        else if( fRequiredSignalState == Signal::kFreq )
        {
            fDoGenerateFunc = &FakeTrackSignalGenerator::DoGenerateFreq;
        }
        else
        {
            LWARN( lmclog, "Unknown domain requested: " << aDomain );
        }
        return;
    }


    bool FakeTrackSignalGenerator::DoGenerate( Signal* aSignal )
    {
        return (this->*fDoGenerateFunc)( aSignal );
    }

    double FakeTrackSignalGenerator::rel_cyc(double energy, double b_field)
    {
        double cyc_freq = (q_C)*b_field/m_kg;
        double rel_cyc_freq = cyc_freq/(1+(energy/me_keV))/(2*PI);
        return rel_cyc_freq; // takes energy in keV, magnetic field in T, returns in Hz
    }

    double FakeTrackSignalGenerator::rel_energy(double frequency, double b_field)
    {
        double cyc_freq = (q_C)*b_field/m_kg;
        double rel_energy = (cyc_freq/(2*PI*frequency)-1)*me_keV;
        return rel_energy; // takes frequency in Hz, magnetic field in T, returns in keV        
    }

    float FakeTrackSignalGenerator::myErfInv(float x) // tolerance under +-6e-3, approximate inverse error function from "A handy approximation for the error function and its inverse" by Sergei Winitzki
    {
       float tt1, tt2, lnx, sgn;
       sgn = (x < 0) ? -1.0f : 1.0f;
       x = (1 - x)*(1 + x);        // x = 1 - x*x;
       lnx = logf(x);
       tt1 = 2/(PI*0.147) + 0.5f * lnx;
       tt2 = 1/(0.147) * lnx;

       return(sgn*sqrtf(-tt1 + sqrtf(tt1*tt1 - tt2)));
    }

    double FakeTrackSignalGenerator::scattering_inverseCDF(double p)
    {
        // Fit params from Aseev et al. paper
        double A1 = 0.204; // +/- 0.001 eV^-1
        double w1 = 1.85; // +/- 0.02 eV
        double eps1 = 12.6; // eV
        double A2 = 0.0556; // +/- 0.0003 eV^-1
        double w2 = 12.5; // +/- 0.1 eV
        double eps2 = 14.30; // +/- 0.02 eV

        double eps_c = 14.08948948948949; // calculated where the pieces of the PDF meet (eV)
        double inv_eps_c = 0.4476144865289551; // calculated as CDF @ epsilon_c
        double alpha, beta, energy_loss; 

        if (p < inv_eps_c)
        {
            energy_loss = w1/pow(2,0.5)*myErfInv(pow(2/PI,0.5)*(2*p/(A1*w1))-1)+eps1;
        }
        else
        {
            alpha = pow(PI/2,0.5)*A1*w1/2*(erf(pow(2,0.5)*(eps_c-eps1)/w1)+1);
            beta = A2*w2/2*atan(2*(eps_c-eps2)/w2);
            energy_loss = w2/2*tan(2/(A2*w2)*(p-alpha+beta))+eps2;
        }
        return energy_loss; // input must be random continuous variable following uniform(0,1), returns in eV
    }

    void FakeTrackSignalGenerator::SetTrackProperties(bool firsttrack, int event_tracks_counter)
    {

        int random_seed_val;
        double current_energy;
        double scattering_cdf_val;
        double energy_loss;
        double new_energy;

        if ( fRandomSeed != 0 )
        {
            random_seed_val = fRandomSeed;
        }
        else
        {   
            std::random_device rd;
            random_seed_val = rd();
        }

        std::default_random_engine generator(random_seed_val);
        std::normal_distribution<double> slope_distribution(fSlopeMean,fSlopeStd);
        std::uniform_real_distribution<double> startfreq_distribution(fStartFrequencyMin,fStartFrequencyMax);
        std::exponential_distribution<double> tracklength_distribution(1./fTrackLengthMean);
        std::uniform_real_distribution<double> starttime_distribution(fStartTimeMin,fStartTimeMax);
        std::uniform_real_distribution<double> dist(0,0.9798681926077586); // for scattering inverse cdf input, upper bound is cdf @ 100 eV

	if (firsttrack)
          {
          starttime_val = starttime_distribution(generator);
          startfreq_val = startfreq_distribution(generator);
          }
        else
          {
	      starttime_val = endtime_val + 0.;  // old track endtime + margin=0.
          current_energy = rel_energy(startfreq_val,fBField); // convert current frequency to energy
          std::default_random_engine generator2(random_seed_val+event_tracks_counter); // get new random value, different from above properties
          scattering_cdf_val = dist(generator2); // random continous variable for scattering inverse cdf input
          energy_loss = scattering_inverseCDF(scattering_cdf_val)/1.e3; // get a random energy loss using the inverse sampling theorem, scale to keV
          new_energy = current_energy - energy_loss; // new energy after loss, in keV
          jumpsize_val = rel_cyc(new_energy,fBField) - startfreq_val; // in Hz
          startfreq_val += jumpsize_val; // add jumpsize to frequency
          }

        slope_val = slope_distribution(generator);
        tracklength_val = tracklength_distribution(generator);
        endtime_val = starttime_val + tracklength_val;  // reset endtime.
	//		printf("startfreq is %g\n", startfreq_val);

    }


    bool FakeTrackSignalGenerator::DoGenerateTime( Signal* aSignal )
    {

        RunLengthCalculator *RunLengthCalculator1 = new RunLengthCalculator;

        const unsigned nchannels = fNChannels;
        double LO_phase = 0.;
        double dt = 1./aSignal->DecimationFactor()/(RunLengthCalculator1->GetAcquisitionRate()*1.e6);
        int random_seed_val;
        if ( fRandomSeed != 0 )
        {
            random_seed_val = fRandomSeed;
        }
        else
        {   
            std::random_device rd;
            random_seed_val = rd();
        }

        //LINFO( lmclog, "random_seed: " << random_seed_val ); // debugging

        std::default_random_engine generator(random_seed_val);
        std::exponential_distribution<double> ntracks_distribution(1./fNTracksMean);
        int ntracks_val = round(ntracks_distribution(generator));
        if ( ntracks_val == 0 ) // if we rounded to 0, let's simulate at least one tracks
        {
            ntracks_val = 1;
        }

        SetTrackProperties(1);

        for (unsigned ch = 0; ch < nchannels; ++ch) // over all channels
        {
            double voltage_phase = fStartVPhase;
            bool nexttrack_flag = false;
            int event_tracks_counter = 0;
            bool eventdone_flag = false; 

            for( unsigned index = 0; index < aSignal->TimeSize()*aSignal->DecimationFactor(); ++index ) // advance sampling time
            {
                double time = (double)index/aSignal->DecimationFactor()/(RunLengthCalculator1->GetAcquisitionRate()*1.e6);           
                LO_phase += 2.*LMCConst::Pi()*fLO_frequency*dt;

                if ( eventdone_flag == false ) // if not done with event
                {
                    if ( nexttrack_flag == false ) // if on same track
                    {
                        if ( (time >= starttime_val) && (time <= endtime_val) )
                        {
                            startfreq_val += slope_val*1.e6/1.e-3*dt;
                            voltage_phase += 2.*LMCConst::Pi()*startfreq_val*(dt);
                            aSignal->LongSignalTimeComplex()[ch*aSignal->TimeSize()*aSignal->DecimationFactor() + index][0] += sqrt(50.)*sqrt(fSignalPower)*cos(voltage_phase-LO_phase);
                            aSignal->LongSignalTimeComplex()[ch*aSignal->TimeSize()*aSignal->DecimationFactor() + index][1] += sqrt(50.)*sqrt(fSignalPower)*cos(-LMCConst::Pi()/2. + voltage_phase-LO_phase);
                        }
                        else if ( time>endtime_val )
                        {
                            event_tracks_counter += 1;
                            if (event_tracks_counter >= ntracks_val) // if done with all tracks in event
                            {
                                eventdone_flag = true; // mark end of event   
                                continue;  
                            }
                            nexttrack_flag = true; // next track
                            voltage_phase = 0.;  // interrupt phase.
                            SetTrackProperties(0,event_tracks_counter);
                        }
                    }
                    else if ( nexttrack_flag == true )
                    {
		        startfreq_val += slope_val*1.e6/1.e-3*dt;
                        voltage_phase += 2.*LMCConst::Pi()*startfreq_val*(dt);
                        aSignal->LongSignalTimeComplex()[ch*aSignal->TimeSize()*aSignal->DecimationFactor() + index][0] += sqrt(50.)*sqrt(fSignalPower)*cos(voltage_phase-LO_phase);
                        aSignal->LongSignalTimeComplex()[ch*aSignal->TimeSize()*aSignal->DecimationFactor() + index][1] += sqrt(50.)*sqrt(fSignalPower)*cos(-LMCConst::Pi()/2. + voltage_phase-LO_phase);
                        nexttrack_flag = false; // now we stay on this track
                    }
                }
            }   
        }
        delete RunLengthCalculator1;
        return true;
    }

    bool FakeTrackSignalGenerator::DoGenerateFreq( Signal* aSignal )
    {
        return true;
    }

} /* namespace locust */
