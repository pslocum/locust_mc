/*
 * LMCFakeTrackSignalGenerator.hh
 *
 *  Created on: Aug 8 2018
 *      Author: plslocum, L. Saldana
 */

#ifndef LMCFAKETRACKSIGNALGENERATOR_HH_
#define LMCFAKETRACKSIGNALGENERATOR_HH_

#include "TFile.h"  // order of includes matters.
#include "TTree.h"  // include these first.

#include "LMCGenerator.hh"
#include "LMCRunLengthCalculator.hh"
#include "LMCEvent.hh"
#include <random>
#include <vector>
#include <boost/math/interpolators/barycentric_rational.hpp>

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

      @brief Generate custom, aka "fake", CRES event.

      @details
      Operates in time.

      Configuration name: "fake-track"

      Available configuration options:
      - "signal-power": double -- PSD of signal (at 90 degrees) (W/Hz).
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
      - "n-events": int -- Number of events per simulation, spaced by 0.5 ms (hardcoded).
      - "random-seed": integer -- integer seed for random number generator for above pdfs, if set to 0 random_device will be used.
      - "root-filename": str -- Name of root file containing event information to be written at output. 


*/
    class FakeTrackSignalGenerator : public Generator
    {
        public:
            FakeTrackSignalGenerator( const std::string& aName = "fake-track" );
            virtual ~FakeTrackSignalGenerator();

            bool Configure( const scarab::param_node* aNode );

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

            double GetHydrogenFraction() const;
            void SetHydrogenFraction( double aHydrogenFraction );

            int GetRandomSeed() const;
            void SetRandomSeed(  int aRandomSeed );

            int GetNEvents() const;
            void SetNEvents(  int aNEvents );


            Signal::State GetDomain() const;
            void SetDomain( Signal::State aDomain );
            void SetTrackProperties(Track &aTrack, int TrackID, double TimeOffset);
            void InitiateEvent(Event* anEvent, int eventID);
            void PackEvent(Track& aTrack, Event* anEvent, int trackID) const;
            double rel_cyc(double energy, double b_field) const;
            double rel_energy(double frequency, double b_field) const;
            double WaveguidePowerCoupling(double frequency, double theta);
            void ReadFile(std::string filename, std::vector<std::pair<double,double> > &data);
            void SetInterpolator(boost::math::barycentric_rational<double> &interpolant, std::vector< std::pair<double, double> > data);
            void InelasticScatter(double &eLoss, double &kTransfer);
            double CrossSection(double E, double K, bool hydrogenScatter);
            double GetCosTheta(double T, double eLoss, double kTransfer, bool hydrogenScatter);


            double slope_val = 0.;
            double pitch = 0.;
            double tracklength_val = 0.;
            double starttime_val = 0.;
            double endtime_val = 0.;
            double startfreq_val = 0.;
            double jumpsize_val = 0.002e9;
            int ntracks_val = 0;



        private:

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
            int fNEvents;
            double fHydrogenFraction;
            std::string fRoot_filename;
            std::default_random_engine fRandomEngine;
            boost::math::barycentric_rational<double> fH2Interpolant;
            boost::math::barycentric_rational<double> fKrInterpolant;


    };

} /* namespace locust */

#endif /* LMCFAKETRACKSIGNALGENERATOR_HH_ */

