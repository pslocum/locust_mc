#ifndef LMCFIELDESTIMATOR_HH_
#define LMCFIELDESTIMATOR_HH_ 

//#include "LMCThreeVector.hh"
#include "LMCException.hh"
#include "param.hh"

/*
#include <string>
#include <vector>
#include <array>
#include <sstream>
*/
namespace locust
{
    /*!
         @class FieldEstimator
         @author P. T. Surukuchi
         @brief Estimates field based on a specified voltage provided to a particular antenna
         @details Estimates field based on a specified voltage provided to a particular antenna. Currently does this for a given HFSS-generated impulse response. Includes the convolution of FIR if it is used for the estimation of fields. 
	 Available configuration options:
     	 - "generator-type": string -- Define if the generator to be used is based on impulse response or a analytical
	 - "fir-filename": string -- The location of the file containing impulse response
    */
    
    class FieldEstimator 
    {
        public:
            FieldEstimator();
            virtual ~FieldEstimator();
	    
	    // Member functions
	    bool Configure( const scarab::param_node* aNode );
	    bool ReadFIRFile();

        private:

	    // Member variables
	    std::string fFIRFilename;
	    std::string fGeneratorType;
	    std::vector<double> fFIRFilter;

	    //Member functions
	    bool ends_with(const std::string &, const std::string &);
    };

} /*namespace locust*/

#endif/*LMCFIELDESTIMATOR_HH_ */
