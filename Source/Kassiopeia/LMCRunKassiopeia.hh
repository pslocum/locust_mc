/*
 * LMCRunKassiopeia.hh
 *
 *  Created on: Mar 10, 2016
 *      Author: nsoblath
 */

#ifndef LMC_LMCRUNKASSIOPEIA_HH_
#define LMC_LMCRUNKASSIOPEIA_HH_

#include <vector>
#include <string>






namespace locust
{
    /*!
     @class RunKassiopeia
     @author N. S. Oblath
     @brief Locust/Kassiopeia interface
     @details Locust control of submodule Kassiopeia
    */
    class RunKassiopeia
    {
        public:
            RunKassiopeia();
            virtual ~RunKassiopeia();

            int Run( const std::vector< std::string >& aFiles );
            int Run( const std::string& aFile );

        private:
           


    };

} /* namespace locust */

#endif /* LMC_LMCRUNKASSIOPEIA_HH_ */

