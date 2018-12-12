/*
 * LMCFakeTrackSignalGenerator.hh
 *
 *  Created on: Aug 8 2018
 *      Author: plslocum, L. Saldana
 */

#ifndef LMCFAKETRACKSIGNALGENERATOR_HH_
#define LMCFAKETRACKSIGNALGENERATOR_HH_

#include "LMCGenerator.hh"
#include "LMCRunLengthCalculator.hh"
#include <random>

const double PI = 3.141592653589793;
double m_kg = 9.10938291*1e-31; // electron mass in kg
double q_C = 1.60217657*1e-19; // electron charge in C
double me_keV = 510.998; // electron mass in keV

namespace scarab
{
    class param_node;
}

namespace locust
{
    class Digitizer;

    /*!
      @class FakeTrackSignalGenerator
      @author P. L. Slocum, L. Saldana

      @brief Generate custom, aka "fake", CRES track.

      @details
      Operates in time.

      Configuration name: "fake-track"

      Available configuration options:
      - "signal-power": double -- PSD of signal (W/Hz).
      - "start-frequency-max": double -- Upper bound for start frequency of signal (Hz); distribution: uniform.
      - "start-frequency-min": double -- Lower bound for start frequency of signal (Hz); distribution: uniform.
      - "track-length-mean": double -- Average of track length (s); distribution: exponential.
      - "start-vphase": double -- Starting voltage phase (V).
      - "slope-mean": double -- Mean value of Gaussian slope distribution (MHz/ms); distribution: gaussian.
      - "slope-std": double -- Standard deviation of Gaussian slope distribution (MHz/ms); distribution: gaussian.
      - "lo-frequency": double -- Frequency of local oscillator (Hz).
      - "start-time-max": double -- Upper bound for track start time (s); distribution: uniform.
      - "start-time-min": double -- Lower bound for track start time (s); distribution: uniform.
      - "ntracks-mean": double -- Average number of tracks per event (integer); distribution: exponential.
      - "magnetic-field": double -- Magnetic field used to convert from frequency to energy (for jumpsize) (T).
      - "random-seed": integer -- integer seed for random number generator for above pdfs, if set to 0 random_device will be used. 


*/
    class FakeTrackSignalGenerator : public Generator
    {
        public:
            FakeTrackSignalGenerator( const std::string& aName = "fake-track" );
            virtual ~FakeTrackSignalGenerator();

            bool Configure( const scarab::param_node* aNode );
            bool Configure2( const Digitizer* aDig );


            void Accept( GeneratorVisitor* aVisitor ) const;

            double GetFrequency() const;
            void SetFrequency( double aFrequency );

            double GetSignalPower() const;
            void SetSignalPower( double aPower );

            double GetStartFrequencyMax() const;
            void SetStartFrequencyMax( double aFrequencyMax );

            double GetStartFrequencyMin() const;
            void SetStartFrequencyMin( double aFrequencyMin );

            double GetTrackLengthMean() const;
            void SetTrackLengthMean( double aTrackLengthMean );

            double GetStartVPhase() const;
            void SetStartVPhase( double aPhase );

            double GetSlopeMean() const;
            void SetSlopeMean( double aSlopeMean );

            double GetSlopeStd() const;
            void SetSlopeStd( double aSlopeStd );

            double GetStartTimeMax() const;
            void SetStartTimeMax( double aTimeMax );

            double GetStartTimeMin() const;
            void SetStartTimeMin( double aTimeMin );

            double GetNTracksMean() const;
            void SetNTracksMean( double aNTracksMean );

            double GetBField() const;
            void SetBField( double aBField );

            int GetRandomSeed() const;
            void SetRandomSeed(  int aRandomSeed );


            Signal::State GetDomain() const;
            void SetDomain( Signal::State aDomain );

            void SetTrackProperties(bool firsttrack, int event_tracks_counter = 0);

            mutable double slope_val = 0.;
            mutable double tracklength_val = 0.;
            mutable double starttime_val = 0.;
            mutable double endtime_val = 0.;
            mutable double startfreq_val = 0.;
            mutable double jumpsize_val = 0.002e9;



        private:
            double rel_cyc(double energy, double b_field);
            double rel_energy(double frequency, double b_field);
            float myErfInv(float x);
            double scattering_inverseCDF(double p);

            bool DoGenerate( Signal* aSignal );

            bool DoGenerateTime( Signal* aSignal );
            bool DoGenerateFreq( Signal* aSignal );

            bool (FakeTrackSignalGenerator::*fDoGenerateFunc)( Signal* aSignal );

            double fSignalPower;
            double fStartFrequencyMax;
            double fStartFrequencyMin;
            double fStartVPhase;
            double fSlopeMean;
            double fSlopeStd;
            double fStartTimeMax;
            double fStartTimeMin;
            double fLO_frequency;
            double fTrackLengthMean;
            double fNTracksMean;
            double fBField;
            int fRandomSeed;



    };

} /* namespace locust */

#endif /* LMCFAKETRACKSIGNALGENERATOR_HH_ */

