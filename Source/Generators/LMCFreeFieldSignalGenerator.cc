/*
 * LMCFreeFieldSignalGenerator.cc
 *
 *  Created on: Mar 12, 2017
 *      Author: buzinsky 
 */

#include "LMCFreeFieldSignalGenerator.hh"
#include "LMCEventHold.hh"
#include "LMCRunKassiopeia.hh"

#include "logger.hh"
#include <thread>
#include <algorithm>

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>

#include "LMCGlobalsDeclaration.hh"

namespace locust
{
    LOGGER( lmclog, "FreeFieldSignalGenerator" );

    MT_REGISTER_GENERATOR(FreeFieldSignalGenerator, "freefield-signal");

    FreeFieldSignalGenerator::FreeFieldSignalGenerator( const std::string& aName ) :
        Generator( aName ),
        fLO_Frequency( 0.),
        fArrayRadius(0.),
        fPatchSpacing(0.),
        fNPatchesPerStrip(0.),
        fCorporateFeed(1),
        fPileupSeed( 0.),
        fPileupMode( false),
        gxml_filename("blank.xml")
    {
        fRequiredSignalState = Signal::kTime;
    }

    FreeFieldSignalGenerator::~FreeFieldSignalGenerator()
    {
    }

    bool FreeFieldSignalGenerator::Configure( const scarab::param_node& aParam )
    {
        if( aParam.has( "xml-filename" ) )
        {
            gxml_filename = aParam["xml-filename"]().as_string();
        }

        if( aParam.has( "array-radius" ) )
        {
            fArrayRadius = aParam["array-radius"]().as_double();
        }

        if( aParam.has( "npatches-per-strip" ) )
        {
            fNPatchesPerStrip = aParam["npatches-per-strip"]().as_double();
        }

        if( aParam.has( "patch-spacing" ) )
        {
            fPatchSpacing = aParam["patch-spacing"]().as_double();
        }

        if( aParam.has( "pileup" ) )
        {
            fPileupMode = aParam["pileup"]().as_bool();
        }

        if( aParam.has( "pileup-seed" ) )
        {
            fPileupSeed = aParam["pileup-seed"]().as_int();
        }
        if( aParam.has( "feed" ) )
        {
            std::string feedInput = aParam["feed"]().as_string();
            if(feedInput == "series")
                fCorporateFeed = false;
        }


        return true;
    }

    void FreeFieldSignalGenerator::Accept( GeneratorVisitor* aVisitor ) const
    {
        aVisitor->Visit( this );
        return;
    }


    static void* KassiopeiaInit(const std::string &aFile)
    {
        RunKassiopeia *RunKassiopeia1 = new RunKassiopeia;
        RunKassiopeia1->Run(aFile);
        delete RunKassiopeia1;

        return 0;
    }

    static void WakeBeforeEvent()
    {
        fPreEventCondition.notify_one();
        return;
    }

    static bool ReceivedKassReady()
    {
        printf("LMC about to wait ..\n");

        std::unique_lock< std::mutex >tLock( fKassReadyMutex);
        fKassReadyCondition.wait( tLock, [](){return fKassEventReady;} );
        printf("LMC Got the fKassReadyCondition signal\n");

        return true;
    }

    void* FreeFieldSignalGenerator::DriveAntenna(int PreEventCounter, unsigned index, Signal* aSignal)
    {

        const int signalSize = aSignal->TimeSize();

        //Receiver Properties
        double tReceiverTime = t_old;
        double tRetardedTime = 0.; //Retarded time of particle corresponding to when emission occurs, reaching receiver at tReceiverTime
        const double tLocustStep = 1. / (fAcquisitionRate * 1e6 * aSignal->DecimationFactor());

        PatchAntenna *currentPatch;
        unsigned tTotalPatchIndex = 0;

        std::ofstream myfile;
        myfile.open("example.txt", std::ios::app);

        for(int channelIndex = 0; channelIndex < allChannels.size(); ++channelIndex)
        {
            for(int patchIndex = 0; patchIndex < allChannels[channelIndex].size(); ++patchIndex)
            {
                currentPatch = &allChannels[channelIndex][patchIndex]; 

                fFieldSolver.SetFieldEvent(tReceiverTime, tTotalPatchIndex);
                fFieldSolver.SolveFieldSolutions();

                LMCThreeVector tRadiatedElectricField = fFieldSolver.GetElectricField();
                LMCThreeVector tRadiatedMagneticField = fFieldSolver.GetMagneticField();
                locust::Particle tCurrentParticle = fFieldSolver.GetRetardedParticle();


                //////////////////////////////////////////////
                LMCThreeVector tDirection = currentPatch->GetPosition() - tCurrentParticle.GetPosition(true);
                double tVelZ = tCurrentParticle.GetVelocity(true).Z();
                double tCosTheta =  tVelZ * tDirection.Z() /  tDirection.Magnitude() / fabs(tVelZ);
                double tDopplerFrequency  = tCurrentParticle.GetCyclotronFrequency() / ( 1. - fabs(tVelZ) / LMCConst::C() * tCosTheta);

                currentPatch->SetInstantaneousFrequency( tDopplerFrequency / (2. * LMCConst::Pi() ));
                currentPatch->SetIncidentElectricField( tRadiatedElectricField );
                currentPatch->SetIncidentMagneticField( tRadiatedMagneticField );

                aSignal->LongSignalTimeComplex()[channelIndex*signalSize*aSignal->DecimationFactor() + index][0] += currentPatch->GetVoltage();
                if(tReceiverTime > 1e-7 && tReceiverTime < 2e-7)
                    myfile << tTotalPatchIndex <<" "<< tReceiverTime <<" "<<tRadiatedElectricField.X()<<" "<<tRadiatedElectricField.Y()<<" "<<tRadiatedElectricField.Z()<<std::endl;
                //aSignal->LongSignalTimeComplex()[channelIndex*signalSize*aSignal->DecimationFactor() + index][1] += currentPatch->GetVoltage();

                ++tTotalPatchIndex;

            } // z_position waveguide element stepping loop.
        } // nChannels loop.


        myfile.close();

        t_old += tLocustStep;

        return 0;
    }


    void FreeFieldSignalGenerator::InitializePatchArray()
    {

        fNChannels+=2;
        const unsigned nChannels = fNChannels;
        const int nReceivers = fNPatchesPerStrip; //Number of receivers per channel

        const double patchSpacingZ = fPatchSpacing;
        const double patchRadius = fArrayRadius;
        double zPosition;
        double theta;
        const double dThetaArray = 2. * LMCConst::Pi() / nChannels; //Divide the circle into nChannels

        PatchAntenna modelPatch;

        allChannels.resize(nChannels);

        for(int channelIndex = 0; channelIndex < nChannels-2; ++channelIndex)
        {
            theta = channelIndex * dThetaArray;

            for(int receiverIndex = 0; receiverIndex < nReceivers; ++receiverIndex)
            {
                zPosition =  (receiverIndex - (nReceivers - 1.) /2.) * fPatchSpacing;

                modelPatch.SetCenterPosition({patchRadius * cos(theta) , patchRadius * sin(theta) , zPosition }); 
                modelPatch.SetPolarizationDirection({sin(theta), -cos(theta), 0.}); 
                modelPatch.SetNormalDirection({-cos(theta), -sin(theta), 0.}); //Say normals point inwards
                allChannels[channelIndex].AddReceiver(modelPatch);
                fFieldSolver.AddFieldPoint(modelPatch.GetPosition());
            }
        }

        //create endcap channels
        double zMax =  (nReceivers - 1.) / 2. * fPatchSpacing;
        int nPatchesMax = std::max(int(2. * patchRadius / fPatchSpacing), 1);
        int nPatchesEnd = 0;
        for( int i=0;i<nPatchesMax;++i)
        {
            double xtmp = -patchRadius + i * fPatchSpacing;
            for( int j=0;j<nPatchesMax;++j)
            {
                double ytmp = -patchRadius + j * fPatchSpacing;
                double pointRad = sqrt(xtmp*xtmp+ytmp*ytmp);

                if( pointRad < patchRadius)
                {
                    ++nPatchesEnd;
                    modelPatch.SetPolarizationDirection({1,0,0});

                    modelPatch.SetCenterPosition({xtmp , ytmp, zMax });
                    modelPatch.SetNormalDirection({0,0,-1});
                    allChannels[nChannels-2].AddReceiver(modelPatch);
                    fFieldSolver.AddFieldPoint(modelPatch.GetPosition());

                    modelPatch.SetCenterPosition({xtmp , ytmp, -zMax });
                    modelPatch.SetNormalDirection({0,0,1});
                    allChannels[nChannels-1].AddReceiver(modelPatch);
                    fFieldSolver.AddFieldPoint(modelPatch.GetPosition());
                }

            }
        }
    }

    bool FreeFieldSignalGenerator::DoGenerate( Signal* aSignal )
    {
        InitializePatchArray();

        // Initialize random number generator for pileup
        if(fPileupMode)
        { 
            if(fPileupSeed!=0)
                srand(fPileupSeed);
            else 
                srand(time(NULL));
        }

        //n samples for event spacing.
        int PreEventCounter = 0;
        const int NPreEventSamples = 150000;
        fKassTimeStep = 1./(fAcquisitionRate*1.e6*aSignal->DecimationFactor());

        std::thread Kassiopeia (KassiopeiaInit, gxml_filename);     // spawn new thread

        fRunInProgress = true;
        unsigned index = 0;

        while( index < aSignal->DecimationFactor()*aSignal->TimeSize() && fRunInProgress)
        {
            if ((!fEventInProgress) && (fRunInProgress) && (!fPreEventInProgress))
            {
                if (ReceivedKassReady()) fPreEventInProgress = true;
            }

            if (fPreEventInProgress)
            {
                PreEventCounter += 1;
                if (PreEventCounter > NPreEventSamples)  // finished noise samples.  Start event.
                {
                    fPreEventInProgress = false;  // reset.
                    fEventInProgress = true;
                    WakeBeforeEvent();  // trigger Kass event.
                    if(fPileupMode)
                    {
                        index = rand() % aSignal->DecimationFactor()*aSignal->TimeSize();
                    }

                }
            }


            if (fEventInProgress)  // fEventInProgress
                if (fEventInProgress)  // check again.
                {
                    //printf("waiting for digitizer trigger ... index is %d\n", index);
                    std::unique_lock< std::mutex >tLock( fMutexDigitizer, std::defer_lock );
                    tLock.lock();
                    fDigitizerCondition.wait( tLock );
                    if (fEventInProgress)
                    {
                        //printf("about to drive antenna, PEV is %d\n", PreEventCounter);
                        DriveAntenna(PreEventCounter, index, aSignal);

                        PreEventCounter = 0; // reset
                    }
                    tLock.unlock();
                }

            ++index;

        }  // for loop

        //delete [] ImaginarySignal;

        // trigger any remaining events in Kassiopeia so that its thread can finish.
        while (fRunInProgress)
        {
            if (fRunInProgress)
                if (ReceivedKassReady()) WakeBeforeEvent();
        }

        Kassiopeia.join();  // wait for Kassiopeia to finish.

        return true;
    }

} /* namespace locust */
