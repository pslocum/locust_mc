/*
 * LMCKassSignalGenerator.hh
 *
 *  Created on: Mar 12, 2014
 *      Author: nsoblath
 */

#ifndef LMCKASSSIGNALGENERATOR_HH_
#define LMCKASSSIGNALGENERATOR_HH_

#include "LMCGenerator.hh"
#include "LMCConst.hh"
#include "LMCThreeVector.hh"
#include <vector>
using std::vector;


namespace locust
{

    /*!
     @class KassSignalGenerator
     @author N. S. Oblath

     @brief
     Generates a Phase I/ Phase II like signal in waveguide

     @details
     Operates in time space

     Configuration name: "kass-signal"

     Available configuration options:
     - "lo-frequency": double -- Frequency of downmixer
     - "xml-filename": string -- Name of the locust config file

    */
    class KassSignalGenerator : public Generator
    {
        public:

            KassSignalGenerator( const std::string& aName = "kass-signal" );
            virtual ~KassSignalGenerator();

            bool Configure( const scarab::param_node* aNode );

            void Accept( GeneratorVisitor* aVisitor ) const;

            double GetSpaceTimeInterval(const double &aParticleTime, const double &aReceiverTime, const LMCThreeVector &aParticlePosition, const LMCThreeVector &aReceiverPosition, double GroupVelocity);



        private:
            double fLO_Frequency;  // typically defined by a parameter in json file.
            bool DoGenerate( Signal* aSignal );
            void* DriveAntenna(int PreEventCounter, unsigned index, Signal* aSignal, FILE *fp);
            int FindNode(double tNew) const;
            int GetCurrentIndex(double t_old, double distance, int PreEventCounter, double GroupVelocity);
            double TE11ModeExcitation() const;
            double TE10ModeExcitation() const;
            std::string gxml_filename;
            std::string gpitchangle_filename;
            double phi_t1; // antenna voltage phase in radians.
            double phi_t2; // reflecting short voltage phase in radians.
            double phiLO_t; // voltage phase of LO in radians;
            mutable double fPreviousRetardedTime;
            mutable int fPreviousRetardedIndex;
            double EventStartTime;
            bool EventToFile;


    };

} /* namespace locust */

#endif /* LMCKASSSIGNALGENERATOR_HH_ */
