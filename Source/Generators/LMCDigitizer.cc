/*
 * LMCDigitizer.cc
 *
 *  Created on: Mar 3, 2014
 *      Author: nsoblath
 */

#include "LMCDigitizer.hh"

#include "logger.hh"


using scarab::get_calib_params;
using scarab::dig_calib_params;
using scarab::a2d;

namespace locust
{
    LOGGER( lmclog, "Digitizer" );

    MT_REGISTER_GENERATOR(Digitizer, "digitizer");

    Digitizer::Digitizer( const std::string& aName ) :
            Generator( aName ),
            fADCValuesSigned( false )
    {
        fRequiredSignalState = Signal::kTime;

	//					        get_calib_params( 8, 1, -3.e-6, 6.e-6, false, &fParams );  // if Gaussian noise is included.

						      get_calib_params( 8, 1, -1.e-8, 2.e-8, false, &fParams );  // if Gaussian noise is not included.
    }

    Digitizer::~Digitizer()
    {
    }

    bool Digitizer::Configure( const scarab::param_node* aNode )
    {
        if( aNode == NULL ) return true;

        if( aNode->has( "adc-values-signed" ) )
            SetADCValuesSigned( aNode->get_value< bool >( "adc-values-signed", fADCValuesSigned ) );


        unsigned bitDepth = aNode->get_value( "bit-depth", fParams.bit_depth );
        unsigned dataTypeSize = aNode->get_value( "data-type-size", fParams.data_type_size );
        double vRange = aNode->get_value( "v-range", fParams.v_range );
        double vMin = aNode->get_value( "v-offset", fParams.v_offset );
        
      

        get_calib_params( bitDepth, dataTypeSize, vMin, vRange, false, &fParams );

        LDEBUG( lmclog, "Digitizer calibration parameters set" );

        return true;
    }


    void Digitizer::Accept( GeneratorVisitor* aVisitor ) const
    {
        aVisitor->Visit( this );
        return;
    }

    const dig_calib_params& Digitizer::DigitizerParams() const
    {
        return fParams;
    }

    dig_calib_params& Digitizer::DigitizerParams()
    {
        return fParams;
    }

    bool Digitizer::DoGenerate( Signal* aSignal )
    {
  
        unsigned nchannels = fNChannels;
        unsigned signalSize = aSignal->TimeSize();
        unsigned signalSizeComplex = 2*aSignal->TimeSize()*nchannels;

        double* analogData = aSignal->SignalTime();
//        uint64_t* digitizedData = new uint64_t[ signalSize ];

        std::ofstream dgvoltagefile;
        dgvoltagefile.open("digitizedvoltagefile.txt");

        if( fADCValuesSigned )
        {

            int8_t* digitizedData = new int8_t[ signalSizeComplex ];

            for (unsigned ch = 0; ch < nchannels; ++ch)
            {
            for( unsigned index = 0; index < signalSize; ++index )
            {
                
                digitizedData[2*ch*signalSize + index*2 ] = a2d< double, int8_t >( aSignal->SignalTimeComplex()[ch*signalSize + index ][0], &fParams );
                digitizedData[2*ch*signalSize + index*2+1 ] = a2d< double, int8_t >( aSignal->SignalTimeComplex()[ch*signalSize + index ][1], &fParams );

                if( index < 10 )
                {

                    LWARN( lmclog, "digitizing channel " << ch << ": " << index << " I: " << aSignal->SignalTimeComplex()[ch*signalSize + index ][0] << " --> " << (int) digitizedData[2*ch*signalSize + index*2 ] );  // pls added (int)
                    LWARN( lmclog, "digitizing channel " << ch << ": " << index << " Q: " << aSignal->SignalTimeComplex()[ch*signalSize + index ][1] << " --> " << (int) digitizedData[2*ch*signalSize + index*2+1 ] );  // pls added (int)
                }

                //print out digitized voltages

                dgvoltagefile << index;
                dgvoltagefile << "\n";
                dgvoltagefile << (int) digitizedData[2*ch*signalSize + index*2 ];
                dgvoltagefile << "\n";
                dgvoltagefile << (int) digitizedData[2*ch*signalSize + index*2+1 ] ;
                dgvoltagefile << "\n";

            } // signalsize
            } // channels
            aSignal->ToDigital( digitizedData, signalSizeComplex );
        }  // fADCValuesSigned
        else  // unsigned
        {
               
                uint8_t* digitizedData = new uint8_t[ signalSizeComplex ];
		//FILE *fp = fopen("/home/hep/baker/ps48/data/Simulation/Phase3/timedata.txt", "w");  // write raw time series for testing.


                for (unsigned ch = 0; ch < nchannels; ++ch)
                {
		  //fprintf(fp, "[%d]\n", ch);
                for( unsigned index = 0; index < signalSize; ++index )
                {

		  //fprintf(fp, "%g %g\n", aSignal->SignalTimeComplex()[ch*signalSize + index ][0], aSignal->SignalTimeComplex()[ch*signalSize + index ][1]);
		  // if (aSignal->SignalTimeComplex()[ch*signalSize + index ][0]>0.){printf("%g %g\n", aSignal->SignalTimeComplex()[ch*signalSize + index ][0], aSignal->SignalTimeComplex()[ch*signalSize + index ][1]); getchar();}


                    digitizedData[2*ch*signalSize + index*2 ] = a2d< double, uint8_t >( aSignal->SignalTimeComplex()[ch*signalSize + index ][0], &fParams );
                    digitizedData[2*ch*signalSize + index*2+1 ] = a2d< double, uint8_t >( aSignal->SignalTimeComplex()[ch*signalSize + index ][1], &fParams );


                    if( index < 20 )
                    {
                        LWARN( lmclog, "digitizing channel " << ch << ": " << index << " I: " << aSignal->SignalTimeComplex()[ch*signalSize + index ][0] << " --> " << (int) digitizedData[2*ch*signalSize + index*2 ] );  // pls added (int)
                        LWARN( lmclog, "digitizing channel " << ch << ": " << index << " Q: " << aSignal->SignalTimeComplex()[ch*signalSize + index ][1] << " --> " << (int) digitizedData[2*ch*signalSize + index*2+1 ] );  // pls added (int)
                    }

                    dgvoltagefile << index;
                    dgvoltagefile << "\n";
                    dgvoltagefile << (int) digitizedData[2*ch*signalSize + index*2 ];
                    dgvoltagefile << "\n";
                    dgvoltagefile << (int) digitizedData[2*ch*signalSize + index*2+1 ] ;
                    dgvoltagefile << "\n";

                } // signalsize
                } // channels
		  //fclose(fp);
                aSignal->ToDigital( digitizedData, signalSizeComplex );

        }  // unsigned

        dgvoltagefile.close();
        return true;

    }

} /* namespace locust */
