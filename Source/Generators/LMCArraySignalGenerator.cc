/*
 * LMCArraySignalGenerator.cc
 *
 *  Created on: Feb 19, 2018
 *      Author: pslocum
 */

#include "LMCArraySignalGenerator.hh"

#include "LMCRunKassiopeia.hh"

#include "logger.hh"

#include <chrono>
#include <thread>


namespace locust
{
    LOGGER( lmclog, "ArraySignalGenerator" );

    MT_REGISTER_GENERATOR(ArraySignalGenerator, "array-signal");

    ArraySignalGenerator::ArraySignalGenerator( const std::string& aName ) :
        Generator( aName ),
        fLO_Frequency( 0.),
        fArrayRadius( 0. ),
        fNElementsPerStrip( 0. ),
		fNSubarrays( 1 ),
		fZShiftArray( 0. ),
        fElementSpacing( 0. ),
        gxml_filename("blank.xml"),
		fTextFileWriting( 0 ),
        fphiLO(0.),
		fNPreEventSamples( 150000 ),
        EFieldBuffer( 1 ),
        EPhaseBuffer( 1 ),
        EAmplitudeBuffer( 1 ),
        EFrequencyBuffer( 1 ),
        LOPhaseBuffer( 1 ),
        IndexBuffer( 1 ),
        ElementFIRBuffer( 1 ),
        fFieldBufferSize( 50 ),
		fSwapFrequency( 1000 ),
		fInterface( new KassLocustInterface() )
    {
        fRequiredSignalState = Signal::kTime;

        KLInterfaceBootstrapper::get_instance()->SetInterface( fInterface );
    }

    ArraySignalGenerator::~ArraySignalGenerator()
    {
    }

    bool ArraySignalGenerator::Configure( const scarab::param_node& aParam )
    {

    	if (aParam.has( "power-combining-feed" ))
    	{
    		int npowercombiners = 0;

        	if(aParam["power-combining-feed"]().as_string() == "voltage-divider")
        	{
        		npowercombiners += 1;
        		fPowerCombiner = new VoltageDivider;
        		if(!fPowerCombiner->Configure(aParam))
        		{
        			LERROR(lmclog,"Error configuring voltage divider.");
        			exit(-1);
        		}
        		fAntennaElementPositioner = new AntennaElementPositioner;
           		if(!fAntennaElementPositioner->Configure(aParam))
            	{
            		LERROR(lmclog,"Error configuring antenna element positioner.");
            		exit(-1);
            	}
        	}

        	if(aParam["power-combining-feed"]().as_string() == "slotted-waveguide")
        	{
        		npowercombiners += 1;
        		fPowerCombiner = new SlottedWaveguide;
        		if(!fPowerCombiner->Configure(aParam))
        		{
        			LERROR(lmclog,"Error configuring slotted waveguide.");
        			exit(-1);
        		}
        		fAntennaElementPositioner = new AntennaElementPositioner;
           		if(!fAntennaElementPositioner->Configure(aParam))
            	{
            		LERROR(lmclog,"Error configuring antenna element positioner.");
            		exit(-1);
            	}
        	}

        	if(aParam["power-combining-feed"]().as_string() == "single-patch")
        	{
        		npowercombiners += 1;
        		fPowerCombiner = new SinglePatch;
        		if(!fPowerCombiner->Configure(aParam))
        		{
        			LERROR(lmclog,"Error configuring single patch.");
        			exit(-1);
        		}
        		fAntennaElementPositioner = new SinglePatchPositioner;
           		if(!fAntennaElementPositioner->Configure(aParam))
            	{
            		LERROR(lmclog,"Error configuring single patch positioner.");
            		exit(-1);
            	}
        	}

        	if(aParam["power-combining-feed"]().as_string() == "corporate")
        	{
        		npowercombiners += 1;
        		fPowerCombiner = new CorporateFeed;
        		if(!fPowerCombiner->Configure(aParam))
        		{
        			LERROR(lmclog,"Error configuring corporate feed.");
        			exit(-1);
        		}
        		fAntennaElementPositioner = new AntennaElementPositioner;
           		if(!fAntennaElementPositioner->Configure(aParam))
            	{
            		LERROR(lmclog,"Error configuring antenna element positioner.");
            		exit(-1);
            	}
        	}

        	if(aParam["power-combining-feed"]().as_string() == "s-matrix")
        	{
        		npowercombiners += 1;
        		fPowerCombiner = new SMatrix;
        		if(!fPowerCombiner->Configure(aParam))
        		{
        			LERROR(lmclog,"Error configuring s matrix.");
        			exit(-1);
        		}
        		fAntennaElementPositioner = new AntennaElementPositioner;
           		if(!fAntennaElementPositioner->Configure(aParam))
            	{
            		LERROR(lmclog,"Error configuring antenna element positioner.");
            		exit(-1);
            	}
        	}


        	if((aParam["power-combining-feed"]().as_string() == "unit-cell-one-quarter")||
               (aParam["power-combining-feed"]().as_string() == "unit-cell-seven-eighths")||
               (aParam["power-combining-feed"]().as_string() == "unit-cell-nine-sixteenths"))
        	{
        		npowercombiners += 1;
        		fPowerCombiner = new UnitCell;
        		if(!fPowerCombiner->Configure(aParam))
        		{
        			LERROR(lmclog,"Error configuring unit cell.");
        			exit(-1);
        		}
        		fAntennaElementPositioner = new AntennaElementPositioner;
           		if(!fAntennaElementPositioner->Configure(aParam))
            	{
            		LERROR(lmclog,"Error configuring antenna element positioner.");
            		exit(-1);
            	}
        	}


        	if(aParam["power-combining-feed"]().as_string() == "series-feed")
        	{
        		npowercombiners += 1;
        		fPowerCombiner = new SeriesFeed;
        		if(!fPowerCombiner->Configure(aParam))
        		{
        			LERROR(lmclog,"Error configuring series feed.");
        		}
        		fAntennaElementPositioner = new AntennaElementPositioner;
           		if(!fAntennaElementPositioner->Configure(aParam))
            	{
            		LERROR(lmclog,"Error configuring antenna element positioner.");
            		exit(-1);
            	}
        	}


        	if (npowercombiners != 1)
        	{
        		LERROR(lmclog,"LMCArraySignalGenerator needs a single power combiner.  Please choose one value for power-combining-feed in the config file.");
                exit(-1);
        	}

    	}
        else
        {
    		LERROR(lmclog,"LMCArraySignalGenerator has been configured without a power combiner.  Please choose a value for power-combiner-feed in the config file.");
            exit(-1);
        }


        if( aParam.has( "transmitter" ))
        {
        	int ntransmitters = 0;

        	if(aParam["transmitter"]().as_string() == "antenna")
        	{
        		ntransmitters += 1;
        		fTransmitter = new AntennaSignalTransmitter;
        		if(!fTransmitter->Configure(aParam))
        		{
        			LERROR(lmclog,"Error Configuring antenna signal transmitter class");
        		}
        		if(!fTransmitter->InitializeTransmitter())
        		{
        			exit(-1);
        		}
        	}

        	if(aParam["transmitter"]().as_string() == "planewave")
        	{
        		ntransmitters += 1;
        		fTransmitter = new PlaneWaveTransmitter;
        		if(!fTransmitter->Configure(aParam))
        		{
        			LERROR(lmclog,"Error Configuring planewave transmitter class");
        		}

        	}

        	if(aParam["transmitter"]().as_string() == "kassiopeia")
        	{
        		ntransmitters += 1;
        		fTransmitter = new KassTransmitter;
        		if(!fTransmitter->Configure(aParam))
        		{
        			LERROR(lmclog,"Error Configuring kassiopeia transmitter class");
        		}

        	}

        	if (ntransmitters != 1)
        	{
        		LERROR(lmclog,"LMCArraySignalGenerator needs a single transmitter.  Please choose transmitter:antenna or transmitter:planewave or transmitter:kassiopeia in the config file.");
                exit(-1);
        	}
        }
        else
        {
    		LERROR(lmclog,"LMCArraySignalGenerator has been configured without a transmitter.  Please choose transmitter:antenna or transmitter:planewave or transmitter:kassiopeia in the config file.");
            exit(-1);
        }


    	if(!fTFReceiverHandler.Configure(aParam))
    	{
    		LERROR(lmclog,"Error configuring receiver FIRHandler class");
    	}

        if( aParam.has( "buffer-size" ) )
        {
        	fFieldBufferSize = aParam["buffer-size"]().as_int();
        	fHilbertTransform.SetBufferSize(aParam["buffer-size"]().as_int());
        }

    	if(!fHilbertTransform.Configure(aParam))
    	{
    		LERROR(lmclog,"Error configuring buffer sizes in receiver HilbertTransform class");
    	}

        if( aParam.has( "lo-frequency" ) )
        {
            fLO_Frequency = aParam["lo-frequency"]().as_double();
        }

        if( aParam.has( "array-radius" ) )
        {
            fArrayRadius = aParam["array-radius"]().as_double();
        }

        if( aParam.has( "nelements-per-strip" ) )
        {
            fNElementsPerStrip = aParam["nelements-per-strip"]().as_int();
        }

        if( aParam.has( "n-subarrays" ) )
        {
            fNSubarrays = aParam["n-subarrays"]().as_int();
        }

        if( aParam.has( "element-spacing" ) )
        {
            fElementSpacing = aParam["element-spacing"]().as_double();
        }
        if( aParam.has( "zshift-array" ) )
        {
            fZShiftArray = aParam["zshift-array"]().as_double();
        }
        if( aParam.has( "event-spacing-samples" ) )
        {
            fNPreEventSamples = aParam["event-spacing-samples"]().as_int();
        }
        if( aParam.has( "swap-frequency" ) )
        {
            fSwapFrequency = aParam["swap-frequency"]().as_int();
        }
        if( aParam.has( "xml-filename" ) )
        {
            gxml_filename = aParam["xml-filename"]().as_string();
        }
        if( aParam.has( "text-filewriting" ) )
        {
            fTextFileWriting = aParam["text-filewriting"]().as_bool();
        }

        return true;
    }

    void ArraySignalGenerator::Accept( GeneratorVisitor* aVisitor ) const
    {
        aVisitor->Visit( this );
        return;
    }



    void ArraySignalGenerator::KassiopeiaInit(const std::string &aFile)
    {
        RunKassiopeia tRunKassiopeia;
        tRunKassiopeia.Run(aFile, fInterface);
        return;
    }

    void ArraySignalGenerator::InitializeFieldPoints(std::vector< Channel<Receiver*> > allRxChannels)
    {
	for(int channelIndex = 0; channelIndex < fNChannels; ++channelIndex)
	{
            for(int elementIndex = 0; elementIndex < fNElementsPerStrip; ++elementIndex)
            {
            	fTransmitter->InitializeFieldPoint(allRxChannels[channelIndex][elementIndex]->GetPosition());
            }
	}
    }

    void ArraySignalGenerator::WakeBeforeEvent()
    {
        fInterface->fPreEventCondition.notify_one();
        return;
    }

    bool ArraySignalGenerator::ReceivedKassReady()
    {

    	std::this_thread::sleep_for(std::chrono::milliseconds(100));
        LPROG( lmclog, "LMC about to wait" );

        if(!fInterface->fKassEventReady)
        {
            std::unique_lock< std::mutex >tLock( fInterface->fKassReadyMutex );
            fInterface->fKassReadyCondition.wait( tLock );
            return true;
        }
        else if (fInterface->fKassEventReady)
        {
        	return true;
        }
        else
        {
            printf("I am stuck.\n"); getchar();
            return true;
        }

    }


    // fields incident on element.
    void ArraySignalGenerator::RecordIncidentFields(FILE *fp,  double t_old, int elementIndex, double zelement, double tEFieldCoPol)
    {
    	if (t_old > 0.5e-9)
         	{
         	fprintf(fp, "%d %g %g\n", elementIndex, zelement, tEFieldCoPol);
         	}
    }



    double ArraySignalGenerator::GetFIRSample(int nfilterbins, double dtfilter, unsigned channel, unsigned element)
    {

    	double fieldfrequency = EFrequencyBuffer[channel*fNElementsPerStrip+element].front();
    	double HilbertMag = 0.;
    	double HilbertPhase = 0.;
    	double convolution = 0.0;

    	if (fabs(EFieldBuffer[channel*fNElementsPerStrip+element].front()) > 0.)  // field arrived yet?
    	{

    		std::vector<double> HilbertMagPhaseMean; HilbertMagPhaseMean.resize(3);
    		HilbertMagPhaseMean = fHilbertTransform.GetMagPhaseMean(EFieldBuffer[channel*fNElementsPerStrip+element], EFrequencyBuffer[channel*fNElementsPerStrip+element]);
    		HilbertMag = HilbertMagPhaseMean[0];
    		HilbertPhase = HilbertMagPhaseMean[1];

    		for (int i=0; i < nfilterbins; i++)  // populate filter with field.
    		{
    			HilbertPhase += 2.*3.1415926*fieldfrequency*dtfilter;
    			ElementFIRBuffer[channel*fNElementsPerStrip+element].push_back(HilbertMag*cos(HilbertPhase));
    			ElementFIRBuffer[channel*fNElementsPerStrip+element].pop_front();
    		}

    		convolution=fTFReceiverHandler.ConvolveWithFIRFilter(ElementFIRBuffer[channel*fNElementsPerStrip+element]);

    		return convolution;
    	}
    	else return 0.;

    }


    void ArraySignalGenerator::DriveAntenna(FILE *fp, int PreEventCounter, unsigned index, Signal* aSignal, int nfilterbins, double dtfilter)
    {

        const int signalSize = aSignal->TimeSize();
        unsigned sampleIndex = 0;
        const unsigned nChannels = fNChannels;
        const int nReceivers = fNElementsPerStrip;

        //Receiver Properties
        fphiLO += 2. * LMCConst::Pi() * fLO_Frequency * 1./(fAcquisitionRate*1.e6*aSignal->DecimationFactor());

        double tReceiverTime =  fInterface->fTOld;

        unsigned tTotalElementIndex = 0;

        for(int channelIndex = 0; channelIndex < nChannels; ++channelIndex)
        {
            double ElementPhi = (double)channelIndex*360./nChannels*LMCConst::Pi()/180.; // radians.
            for(int elementIndex = 0; elementIndex < nReceivers; ++elementIndex)
            {
            	Receiver* currentElement = allRxChannels[channelIndex][elementIndex];
                sampleIndex = channelIndex*signalSize*aSignal->DecimationFactor() + index;  // which channel and which sample

                std::vector<double> tFieldSolution; //tFieldSolution.resize(2);
                if (!fTransmitter->IsKassiopeia())
                {
                	tFieldSolution = fTransmitter->GetEFieldCoPol(tTotalElementIndex, 1./(fAcquisitionRate*1.e6*aSignal->DecimationFactor()));
                }
                else
                {
                	tFieldSolution = fTransmitter->SolveKassFields(currentElement->GetPosition(), currentElement->GetPolarizationDirection(), tReceiverTime, tTotalElementIndex);
                }

                tFieldSolution[0] *= currentElement->GetPatternFactor(fTransmitter->GetIncidentKVector(tTotalElementIndex), *currentElement);

                if (fTextFileWriting==1) RecordIncidentFields(fp,  fInterface->fTOld, elementIndex, currentElement->GetPosition().GetZ(), tFieldSolution[1]);

 	            FillBuffers(aSignal, tFieldSolution[1], tFieldSolution[0], fphiLO, index, channelIndex, elementIndex);
 	            double VoltageFIRSample = GetFIRSample(nfilterbins, dtfilter, channelIndex, elementIndex);
 	            fPowerCombiner->AddOneVoltageToStripSum(aSignal, VoltageFIRSample, fphiLO, elementIndex, IndexBuffer[channelIndex*fNElementsPerStrip+elementIndex].front());
                PopBuffers(channelIndex, elementIndex);

                ++tTotalElementIndex;

            } // element loop

        } // channels loop

        fInterface->fTOld += 1./(fAcquisitionRate*1.e6*aSignal->DecimationFactor());
        if ( index%fSwapFrequency == 0 ) CleanupBuffers();  // release memory

    }


    void ArraySignalGenerator::FillBuffers(Signal* aSignal, double DopplerFrequency, double EFieldValue, double LOPhase, unsigned index, unsigned channel, unsigned element)
    {
    	EFieldBuffer[channel*fNElementsPerStrip+element].push_back(EFieldValue);
    	EFrequencyBuffer[channel*fNElementsPerStrip+element].push_back(DopplerFrequency/2./LMCConst::Pi());
    	LOPhaseBuffer[channel*fNElementsPerStrip+element].push_back(LOPhase);
    	IndexBuffer[channel*fNElementsPerStrip+element].push_back(channel*aSignal->TimeSize()*aSignal->DecimationFactor() + index);
    }





    void ArraySignalGenerator::PopBuffers(unsigned channel, unsigned element)
    {

    	EFieldBuffer[channel*fNElementsPerStrip+element].pop_front();
    	EFrequencyBuffer[channel*fNElementsPerStrip+element].pop_front();
    	LOPhaseBuffer[channel*fNElementsPerStrip+element].pop_front();
    	IndexBuffer[channel*fNElementsPerStrip+element].pop_front();

    }




    void ArraySignalGenerator::InitializeBuffers(unsigned filterbuffersize, unsigned fieldbuffersize)
    {
    	FieldBuffer aFieldBuffer;
    	EFieldBuffer = aFieldBuffer.InitializeBuffer(fNChannels, fNElementsPerStrip, fieldbuffersize);
    	EFrequencyBuffer = aFieldBuffer.InitializeBuffer(fNChannels, fNElementsPerStrip, fieldbuffersize);
    	LOPhaseBuffer = aFieldBuffer.InitializeBuffer(fNChannels, fNElementsPerStrip, fieldbuffersize);
    	IndexBuffer = aFieldBuffer.InitializeUnsignedBuffer(fNChannels, fNElementsPerStrip, fieldbuffersize);
    	ElementFIRBuffer = aFieldBuffer.InitializeBuffer(fNChannels, fNElementsPerStrip, filterbuffersize);
    }


    void ArraySignalGenerator::CleanupBuffers()
    {
    	FieldBuffer aFieldBuffer;
    	EFieldBuffer = aFieldBuffer.CleanupBuffer(EFieldBuffer);
    	EFrequencyBuffer = aFieldBuffer.CleanupBuffer(EFrequencyBuffer);
    	LOPhaseBuffer = aFieldBuffer.CleanupBuffer(LOPhaseBuffer);
    	ElementFIRBuffer = aFieldBuffer.CleanupBuffer(ElementFIRBuffer);
    	IndexBuffer = aFieldBuffer.CleanupBuffer(IndexBuffer);
    }


    bool ArraySignalGenerator::InitializeElementArray()
    {

        if(!fTFReceiverHandler.ReadHFSSFile())
        {
            return false;
        }

        const unsigned nChannels = fNChannels;
        const unsigned nSubarrays = fNSubarrays;
        const int nReceivers = fNElementsPerStrip;

        const double elementSpacingZ = fElementSpacing;
        const double elementRadius = fArrayRadius;
        double zPosition;
        double theta;
        const double dThetaArray = 2. * LMCConst::Pi() / (nChannels/nSubarrays); //Divide the circle into nChannels
        const double dRotateVoltages = 0.;  // set to zero to not rotate element polarities.

        allRxChannels.resize(nChannels);

        	for(int channelIndex = 0; channelIndex < nChannels; ++channelIndex)
        	{
        		theta = fAntennaElementPositioner->GetTheta(channelIndex, dThetaArray);

        		for(int receiverIndex = 0; receiverIndex < nReceivers; ++receiverIndex)
        		{
        			zPosition = fAntennaElementPositioner->GetPositionZ(fZShiftArray, channelIndex, nChannels,
        						nSubarrays, nReceivers, elementSpacingZ, receiverIndex);

        			Receiver* modelElement = fPowerCombiner->ChooseElement();  // patch or slot?

        			fAntennaElementPositioner->PlaceElement(*modelElement, elementRadius, theta, zPosition);

        			allRxChannels[channelIndex].AddReceiver(modelElement);
        		}
        	}

        return true;
    }



    bool ArraySignalGenerator::DoGenerate( Signal* aSignal )
    {

        if(!InitializeElementArray())
        {
        	LERROR(lmclog,"Error configuring Element array");
            exit(-1);
        }

        FILE *fp;
        if (fTextFileWriting==1) fp = fopen("incidentfields.txt", "w");


        //n samples for event spacing in Kass.
        int PreEventCounter = 0;

        int nfilterbins = fTFReceiverHandler.GetFilterSize();
        double dtfilter = fTFReceiverHandler.GetFilterResolution();
        unsigned nfieldbufferbins = fFieldBufferSize;
        InitializeBuffers(nfilterbins, nfieldbufferbins);

        InitializeFieldPoints(allRxChannels);

        if (!fTransmitter->IsKassiopeia())
        {
        	for( unsigned index = 0; index < aSignal->DecimationFactor()*aSignal->TimeSize(); ++index )
        	{
        		DriveAntenna(fp, PreEventCounter, index, aSignal, nfilterbins, dtfilter);
        	}  // for loop
        	return true;
        }


        if (fTransmitter->IsKassiopeia())
        {
        	bool fTruth = false;
            fInterface->fKassTimeStep = 1./(fAcquisitionRate*1.e6*aSignal->DecimationFactor());
        	std::thread tKassiopeia (&ArraySignalGenerator::KassiopeiaInit, this, gxml_filename); // spawn new thread

            for( unsigned index = 0; index < aSignal->DecimationFactor()*aSignal->TimeSize(); ++index )
            {
                if ((!fInterface->fEventInProgress) && (!fInterface->fPreEventInProgress))
                {
                	if (ReceivedKassReady()) fInterface->fPreEventInProgress = true;
                	else
                	{
                		printf("breaking\n");
                		break;
                	}

                	LPROG( lmclog, "LMC ReceivedKassReady" );

                }

                if (fInterface->fPreEventInProgress)
                {
                    PreEventCounter += 1;

                    if (((!fTruth)&&(PreEventCounter > fNPreEventSamples))||((fTruth)&&(PreEventCounter > fNPreEventSamples)&&(index%(8192*aSignal->DecimationFactor())==0)  ))// finished pre-samples.  Start event.
                    {
                        fInterface->fPreEventInProgress = false;  // reset.
                        fInterface->fEventInProgress = true;
                        LPROG( lmclog, "LMC about to WakeBeforeEvent()" );
                        WakeBeforeEvent();  // trigger Kass event.
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                }

                if ((fInterface->fEventInProgress)&&(!fInterface->fKassEventReady))  // fEventInProgress
                {
                        std::unique_lock< std::mutex >tLock( fInterface->fMutexDigitizer, std::defer_lock );
                        tLock.lock();
                        fInterface->fDigitizerCondition.wait( tLock );
                        if (fInterface->fEventInProgress)
                        {
                    		DriveAntenna(fp, PreEventCounter, index, aSignal, nfilterbins, dtfilter);
                            PreEventCounter = 0; // reset
                        }
                        tLock.unlock();
                }
            }  // for loop


            fInterface->fDoneWithSignalGeneration = true;
            if (fTextFileWriting==1) fclose(fp);
            LPROG( lmclog, "Finished signal loop." );
            WakeBeforeEvent();
            tKassiopeia.join();  // finish thread


        }  // fTransmitter->IsKassiopeia()

        return true;

    }

} /* namespace locust */

