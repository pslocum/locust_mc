/*
 * LMCComplexFFT.cc
 *
 *  Created on: Sep 30, 2019
 *      Author: P. T. Surukuchi
 */

#include "LMCComplexFFT.hh"

namespace locust
{
    ComplexFFT::ComplexFFT():
    IsInitialized(false),
    fTransformFlag("MEASURE"),
    fWisdomFilename("wisdom_complexfft.fftw3"),
    fInputArray(NULL),
    fOutputArray(NULL),
    fForwardPlan(),
    fReversePlan()
    {
    }
    
    ComplexFFT::~ComplexFFT()
    {
        if (fInputArray != NULL)
        {
            fftw_free(fInputArray);
            fInputArray = NULL;
        }
        
        if (fOutputArray != NULL)
        {
            fftw_free(fOutputArray);
            fOutputArray = NULL;
        }
        fftw_destroy_plan(fReversePlan);
        fftw_destroy_plan(fForwardPlan);
    }
    
    bool ComplexFFT::Configure(const scarab::param_node& aParam)
    {
        if( aParam.has("transform-flag"))
        {
            fTransformFlag=aParam["transform-flag"]().as_bool();
        }
        if(aParam.has("use-wisdom"))
        {
            fUseWisdom=aParam["use-wisdom"]().as_bool();
        }
        if(aParam.has("wisdom-filename"))
        {
            fWisdomFilename=aParam["wisdom-filename"]().as_string();
        }
        
        if(fTransformFlag.compare("MEASURE"))
        {
            fTransform=Transform::measure;
        }
        else if(fTransformFlag.compare("ESTIMATE"))
        {
            fTransform=Transform::estimate;
        }
        else
        {
            return false;
        }
        IsInitialized=true;
        return true;
    }
    
    bool ComplexFFT::ForwardFFT(int size, fftw_complex* in, fftw_complex* out)
    {
        if(IsInitialized) return false;
        
        fInputArray = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * size);
        fOutputArray = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * size);
        
        fForwardPlan = fftw_plan_dft_1d(size,fInputArray,fOutputArray,FFTW_FORWARD,fTransform);
        fftw_execute_dft(fForwardPlan,reinterpret_cast<fftw_complex*>(in),reinterpret_cast<fftw_complex*>(out));
        return true;
    }
    
    bool ComplexFFT::ReverseFFT(int size, fftw_complex* in, fftw_complex* out)
    {
        if(IsInitialized) return false;
        
        fInputArray = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * size);
        fOutputArray = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * size);
        
        fReversePlan = fftw_plan_dft_1d(size,fInputArray,fOutputArray,FFTW_BACKWARD,fTransform);
        fftw_execute_dft(fReversePlan,reinterpret_cast<fftw_complex*>(in),reinterpret_cast<fftw_complex*>(out));
        return true;
    }

} /* namespace locust */
