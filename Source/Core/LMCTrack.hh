/*
 * LMCTrack.hh
 *
 *  Created on: Dec 5, 2018
 *      Author: pslocum
 */

#ifndef LMCTRACK_HH_
#define LMCTRACK_HH_

#include "TObject.h"


namespace locust
{
 /*!
 @class Track
 @author P. Slocum
 @brief Class to describe tracks generated in LMCFakeTrackGenerator
 @details
 Available configuration options:
 No input parameters
 */
    class Track;
    class Track : public TObject
    {

        public:
            Track();
            virtual ~Track();
            int EventID = -99;
            double StartTime = -99.;
            double EndTime = -99.;
            double TrackLength = -99.;
            double StartFrequency = -99.;
            double Slope = -99.;

            ClassDef(Track,1)  // Root syntax.

    };

} /* namespace locust */

#endif /* LMCTRACK_HH_ */
