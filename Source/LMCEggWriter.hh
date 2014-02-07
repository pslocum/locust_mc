/*
 * LMCEggWriter.hh
 *
 *  Created on: Feb 6, 2014
 *      Author: nsoblath
 */

#ifndef LMCEGGWRITER_HH_
#define LMCEGGWRITER_HH_

#include "Monarch.hpp"

namespace locust
{
    class ParamNode;
    class Signal;

    class EggWriter
    {
        public:
            enum State
            {
                kClosed,
                kPrepared,
                kWriting,
            };

        public:
            EggWriter();
            virtual ~EggWriter();

            bool Configure( const ParamNode* aNode );

            const std::string& GetFilename() const;
            void SetFilename( const std::string& filename );

            const std::string& GetDate() const;
            void SetDate( const std::string& date );

            const std::string& GetDescription() const;
            void SetDescription( const std::string& desc );

            monarch::RunType GetRunType() const;
            void SetRunType( monarch::RunType runType );

            unsigned GetBitDepth() const;
            bool SetBitDepth( unsigned bitDepth );

            unsigned GetDataTypeSize() const;

            double GetAcquisitionRate() const;
            void SetAcquisitionRate( double rate );

            double GetDuration() const;
            void SetDuration( double duration );

            unsigned GetRecordSize() const;
            void SetRecordSize( unsigned size );

        private:
            State fState;

            std::string fFilename;

            std::string fDate;

            std::string fDescription;

            monarch::RunType fRunType;

            unsigned fBitDepth;
            unsigned fDataTypeSize;

            double fAcquisitionRate;
            double fDuration;
            unsigned fRecordSize;

        public:
            bool PrepareEgg();

            bool WriteRecord( const Signal* aSignal );

            bool FinalizeEgg();

            monarch::Monarch* GetMonarch() const;

        private:
            monarch::Monarch* fMonarch;

            monarch::MonarchRecordBytes* fRecord;

    };

} /* namespace locust */

#endif /* LMCEGGWRITER_HH_ */
