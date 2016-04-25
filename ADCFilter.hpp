////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file ADCFilter.hpp
///
/// A derived class for the low pass filters from the LowPassFilterFixedPt class.                                      
/// Takes advantage of a 50 us sample period to implement multiplications by shifting
///
/// @if REVISION_HISTORY_INCLUDED
/// @par Edit History
/// - thaley 20-Apr-2016 Original implementation
/// @endif
///
/// @ingroup NeoS Low Pass Filtering
///
/// @par Copyright (c) 2016 Rockwell Automation Technolgies, Inc.  All rights reserved.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ADC_FILTER_HPP
#define ADC_FILTER_HPP

// SYSTEM INCLUDES

// C PROJECT INCLUDES


// C++ PROJECT INCLUDES
#include "LowPassFiltersFixedPt.hpp"

namespace LowPassFilters
{
    class ADCFilter : public LowPassFilterFixedPt
    {
    public:
        //**************************************************************************************************************
        // Public methods
        //**************************************************************************************************************
    
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: ADCFilter::ADC Filter
        ///
        /// Constructor
        ///
        /// @par Full Description
        /// Constructor for filtering using the fast shift technique to multiply
        ///
        /// @param  uiCornerFreq         Initial corner frequency 
        /// @param  uiSamplingPeriod     Sampling period for the filter
        /// @param  LagCoefficient       InitialLagCoefficient
        /// @param  AtoDResolutionBits   Bit resolution of ADC channels
        /// 
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ADCFilter(uint32_t uiCornerFreq,
                  uint32_t uiSamplingPeriod,
                  uint32_t LagCoeffecient,
                  uint32_t AtoDResolutionBits)
            : m_FrequencyShiftFactor(0),
              LowPassFilterFixedPt( uiCornerFreq, uiSamplingPeriod, LagCoeffecient, AtoDResolutionBits, FIELD_SIDE_NUMBER_OF_POLES)
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: ADCFilter::~ADC Filter
        ///
        /// Destructor
        ///
        /// @par Full Description
        /// Destructor for filtering using the fast shift technique to multiply
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual ~ADCFilter()
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: ADCFilter::ApplyFilter
        ///
        /// Apply the low pass filter difference equation to unfiltered ADC input data
        ///
        /// @par Full Description
        /// Virtual method for the applying the low pass filter
        ///
        /// @param  iAtoDValueRead           Raw ADC data read by the ADC driver
        /// @param  uiCornerFreqToFilter     Corner Frequency for the filter.
        /// @param  rFilterOutput            Output from filter difference equation
        ///
        /// @return  true when the filter was applied, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool ApplyFilter(int32_t iAtoDValueRead, uint32_t uiCornerFreqToFilter,  int32_t & rFilterOutput);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: ADCFilter::ConfigureFilter
        ///
        /// Configure filter difference equation coefficients
        ///
        /// @par Full Description
        /// Method for the configuring the low pass filter
        ///
        /// @param  uiCornerFreq         Corner Frequency for the filter
        /// @param  uiSamplingPeriod     Sampling period for the filter in microseconds
        ///
        /// @return  true when the filter was configured, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool ConfigureFilter(uint32_t uiCornerFreq, unsigned uiSamplingPeriod);

    private:

        //**************************************************************************************************************
        // Private methods and attributes
        //**************************************************************************************************************
    
       // Number of poles for field side MCU ADC channels
        static const uint32_t FIELD_SIDE_NUMBER_OF_POLES = 4;

        // Bit position where no shift is required for 100 HZ corner frequencies
        static const uint32_t NO_SHIFT_BIT_POS = 22;

        // Enumerated constant for corner frequencies that can be filtered for
        typedef enum
        {
            FREQ_100_HZ = 100,
            FREQ_50_HZ = 50,
            FREQ_25_HZ = 25,
            FREQ_10_HZ = 10,
            FREQ_5_HZ = 5,
            FREQ_1_HZ = 1
        }
        VALID_FREQUENCIES;

        // Shift factor for multiplication by the lag coefficient
        // Determined at object instantiation time or during ADC 
        // AppplyFilter synchronous calls when the corner freq changes
        int32_t m_FrequencyShiftFactor;

        // inhibit default constructor, copy constructor, and assignment
        ADCFilter();

        ADCFilter(ADCFilter &);

        ADCFilter & operator=(ADCFilter const&); // assign op. hidden
    };
};
#endif