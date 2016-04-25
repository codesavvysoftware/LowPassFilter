////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file LowPassNeosFilter.hpp
///
/// The base class for the low pass filters that use floating point arithmetic and derived from the template base class.
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
#ifndef LOW_PASS_NEOS_FILTER_HPP
#define LOW_PASS_NEOS_FILTER_HPP

// SYSTEM INCLUDES

// C PROJECT INCLUDES


// C++ PROJECT INCLUDES
#include "LowPass.hpp"

namespace LowPassFilters
{
    class LowPassNeosFilter : public LowPass<float>
    {
    public:
        //**************************************************************************************************************
        // Public methods
        //**************************************************************************************************************
    
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassNeosFilter::LowPassNeosFilter
        ///
        /// Constructor
        ///
        /// @par Full Description
        /// Base class contructor for the low pass filter using floating point arithmetic
        ///
        /// @param  uiCornerFreq         Initial corner frequency 
        /// @param  uiSamplingPeriod     Sampling period for the filter
        /// @param  fLagCoefficient      InitialLagCoefficient
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
        LowPassNeosFilter( uint32_t uiCornerFreq,
                           uint32_t uiSamplingPeriod,
                           float    fLagCoefficient)
            : m_fPrevFilteredValue(0.0),
              m_fRemainder(0.0),
              LowPass(uiCornerFreq, uiSamplingPeriod, fLagCoefficient)
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassNeosFilter::~LowPassNeosFilter
        ///
        /// Destructor
        ///
        /// @par Full Description
        /// Base class Destructor for the low pass filter using floating point arithmetic
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual ~LowPassNeosFilter()
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassNeosFilter::ApplyFilter
        ///
        /// Apply the low pass filter difference equation to unfiltered ADC input data
        ///
        /// @par Full Description
        /// Virtual method for the applying the low pass filter
        ///
        /// @param  fAtoDValueRead           Raw ADC data read by the ADC driver
        /// @param  uiCornerFreqToFilter     Corner Frequency for the filter.
        /// @param  rfFilterOutput           Output from filter difference equation
        ///
        /// @return  true when the filter was applied, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool ApplyFilter(float        fAtoDValueRead, 
                         uint32_t     uiCornerFreqToFilter, 
                         float &      rfFilterOutput);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassNeosFilter::ConfigureFilter
        ///
        /// Configure filter difference equation coefficients
        ///
        /// @par Full Description
        /// Virtual method for the configuring the low pass filter
        ///
        /// @param  uiCornerFreq         Corner Frequency for the filter
        /// @param  uiSamplingPeriod     Sampling period for the filter in microseconds
        ///
        /// @return  true when the filter was configured, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool ConfigureFilter(uint32_t uiCornerFreq, unsigned uiSamplingPeriod); 

    protected:

        //**************************************************************************************************************
        // Protected methods and attributes
        //**************************************************************************************************************
    
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassNeosFilter::InitFilterDataForRestart
        ///
        /// Initialize filter data to put the filter in it's initial state
        ///
        /// @par Full Description
        /// virtual method for initializing filter data.
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual void InitFilterDataForRestart(float InitialFilterOutput);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassNeosFilter::IsFilterResultValid
        ///
        /// Determine validity of low pass filter output
        ///
        /// @par Full Description
        /// Method for determining the validity of the low filter output generated by difference equation
        ///
        /// @param  DiffEqTerm1     First term of difference equation add
        /// @param  DiffEqTerm2     Second term of difference equation add
        /// @param  fFilterOutput   Result of difference equation add
        ///
        /// @return  true when the result is valid, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool IsFilterOutputValid(float fDiffEqTerm1, float fDiffEqTerm2, float fFilterOutput);

    private:
        
        //**************************************************************************************************************
        // Private methods and attributes
        //**************************************************************************************************************
    
        // Previous filter value required for difference equation
        float m_fPrevFilteredValue;

        // Used in calculating filter output
        float m_fRemainder;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassNeosFilter::IsFloatValid
        ///
        /// Determine validity of input float
        ///
        /// @par Full Description
        /// Method for determining the validity of a float
        ///
        /// @param  f     Float number to check
        ///
        /// @return  true when input float val is valid, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool IsFloatValid(float f);

        // inhibit default constructor, copy constructor, and assignment
        LowPassNeosFilter();

        LowPassNeosFilter(LowPassNeosFilter &);

        LowPassNeosFilter & operator=(LowPassNeosFilter const&); // assign op. hidden
    };

};
#endif