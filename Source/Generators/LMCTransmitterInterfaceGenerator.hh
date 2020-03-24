/*
 * LMCTransmitterInterfaceGenerator.hh
 *
 *  Created on: March 03, 2020
 *      Author: P. T. Surukuchi
 */

#ifndef LMCTRANSMITTERINTERFACEGENERATOR_HH_
#define LMCTRANSMITTERINTERFACEGENERATOR_HH_

#include "LMCThreeVector.hh"
#include "LMCGenerator.hh"
#include "LMCFieldBuffer.hh"
#include "LMCTFFileHandler.hh"
#include "LMCAntennaSignalTransmitter.hh"
#include "LMCPlaneWaveTransmitter.hh"
#include "LMCKassTransmitter.hh"
#include <vector>

namespace locust
{

    /*!
     @class TransmitterInterfaceGenerator
     @author P. T. Surukuchi

     @brief Generate signal with an emnforced transmitter 

     @details
     Operates in time space

     Configuration name: "transmitter-interface"

     Available configuration options:
     - "xml-filename" : std::string -- the name of the xml locust config file.
     - "buffer-size" :  std::int -- number of elements in deque buffers to contain field information.
     	 	 These buffers control arrival times of fields and provide a short time series for the Hilbert transform.
     - "swap-frequency":  number of digitizer samples after which buffer memory is reset.  This
     	 	 becomes more important for large numbers of patches
    */

    class TransmitterInterfaceGenerator : public Generator
    {
        public:

            TransmitterInterfaceGenerator( const std::string& aName = "transmitter-interface" );
            virtual ~TransmitterInterfaceGenerator();

            virtual bool Configure( const scarab::param_node& aNode );

            void Accept( GeneratorVisitor* aVisitor ) const;
              
        protected:
            Transmitter* fTransmitter; // transmitter object
            std::string gxml_filename;
            bool fTextFileWriting;
	    unsigned fNPoints;
            unsigned fFieldBufferSize;
	    unsigned fNFilterBins;
	    double fdtFilter;
            int fSwapFrequency;

            bool WakeBeforeEvent();
            bool ReceivedKassReady();

	    void InitializeFieldPoints();
            void RecordIncidentFields(FILE *fp, double t_old,LMCThreeVector pointOfInterest, double tEFieldCoPol);
            void InitializeBuffers(unsigned filterbuffersize, unsigned fieldbuffersize);
            void CleanupBuffers();
            void PopBuffers(unsigned pointIndex);
            void FillBuffers(Signal* aSignal, double DopplerFrequency, double EFieldValue, unsigned index, unsigned timeIndex);

            std::vector<LMCThreeVector> fAllFieldPoints;
            std::vector<LMCThreeVector> fAllFieldCopol;
            std::vector<std::deque<double>> EFieldBuffer;
            std::vector<std::deque<double>> EFrequencyBuffer;
            std::vector<std::deque<unsigned>> IndexBuffer;

            bool DoGenerate( Signal* aSignal );
            void DriveAntenna(FILE *fp, int PreEventCounter, unsigned index, Signal* aSignal, int nfilterbins, double dtfilter);
    };

} /* namespace locust */

#endif /* LMCTRANSMITTERINTERFACEGENERATOR_HH_ */
