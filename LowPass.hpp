////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file LowPass.hpp
///
/// This class characterizes the aborted state for a firmware update.
///
/// @if REVISION_HISTORY_INCLUDED
/// @par Edit History
/// - thaley 20-Apr-2016 Original implementation
/// @endif
///
/// @ingroup ???
///
/// @par Copyright (c) 2016 Rockwell Automation Technolgies, Inc.  All rights reserved.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef LOWPASS_HPP
#define	LOWPASS_HPP

// SYSTEM INCLUDES
// (none)

// C PROJECT INCLUDES
// (none)
#include <math.h>
#include <stdint.h>

// C++ PROJECT INCLUDES
#include <climits>

// FORWARD REFERENCES
// (none)


namespace LowPassFilters
{
    template<typename NumericFormat> class LowPass
    {
    public:
        //**************************************************************************************************************
        // Public methods
        //**************************************************************************************************************
    
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::LowPass
        ///
        /// Constructor
        ///
        /// @par Full Description
        /// Base class contructor for the Low Pass Single Order Lag Filters
        ///
        /// @param  uiCornerFreq       Initial corner frequency 
        /// @param  uiSamplingPeriod   Sampling period for the filter
        /// @param  LagCoefficient     InitialLagCoefficient
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        LowPass(uint32_t       uiCornerFreq,
                uint32_t       uiSamplingPeriod,
                NumericFormat  LagCoefficient)
            : m_uiCornerFreq(uiCornerFreq),
              m_uiSamplePeriod(uiSamplingPeriod),
              m_LagCoefficient(LagCoefficient),
              m_bFilteringEnabled(false),
              m_bFirstSample(true)
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::~LowPass
        ///
        /// Destrcutor
        ///
        /// @par Full Description
        /// Base class destructor for the Low Pass Single Order Lag Filters
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual ~LowPass()
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::ApplyFilter
        ///
        /// Apply the low pass filter difference equation to unfiltered ADC input data
        ///
        /// @par Full Description
        /// Pure virtual method for the applying the low pass filter
        ///
        /// @param  adcValueRead     Raw ADC data read by the ADC driver
        /// @param  uiCornerFreq     Corner Frequency for the filter.
        /// @param  rFilterOutput    Output from filter difference equation
        ///
        /// @return  true when the filter was applied, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual bool ApplyFilter(NumericFormat adcValueRead, uint32_t uiCornerFreq, NumericFormat & rFilterOutput) = 0;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::ConfigureFilter
        ///
        /// Configure filter difference equation coefficients
        ///
        /// @par Full Description
        /// Pure virtual method for the configuring the low pass filter
        ///
        /// @param  uiCornerFreq         Corner Frequency for the filter
        /// @param  uiSamplingPeriod     Sampling period for the filter in microseconds
        ///
        /// @return  true when the filter was configured, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual bool ConfigureFilter(uint32_t uiCornerFreq, uint32_t uiSamplingPeriod) = 0;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::EnableFiltering()
        ///
        /// Enable low pass filtering
        ///
        /// @par Full Description
        /// Enable low pass filtering of raw ADC data.
        ///
        /// @return None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void EnableFiltering()
        {
            m_bFilteringEnabled = true;
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::DisableFiltering()
        ///
        /// Disable low pass filtering
        ///
        /// @par Full Description
        /// Disable low pass filtering of raw ADC data.
        ///
        /// @return None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void DisableFiltering()
        {
            m_bFilteringEnabled = false;
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::IsFilteringEnabled()
        ///
        /// Query filtering enabled status
        ///
        /// @par Full Description
        /// Query filtering enabled status to determine if filtering is enabled or disabled.
        ///
        /// @return true when filtering is enabled, false when disabled
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool IsFilteringEnabled()
        {
            return m_bFilteringEnabled;
        }


    protected:

        //**************************************************************************************************************
        // Protected methods and attributes
        //**************************************************************************************************************
    
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::InitFilterDataForRestart
        ///
        /// Initialize filter data to put the filter in it's initial state
        ///
        /// @par Full Description
        /// Pure virtual method for initializing filter data.
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual void InitFilterDataForRestart(NumericFormat InitialFilterOutput) = 0;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::IsFilterResultValid
        ///
        /// Determine validity of low pass filter output
        ///
        /// @par Full Description
        /// Pure virtual method for determining the validity of the low filter output generated by difference equation
        ///
        /// @param  Term1           First term of binary aritmetic operation of filter being checked
        /// @param  Term2           Second term of binary aritmetic operation of filter being checked.
        /// @param  Result          Result of binary arithmetic operation of filter being checked
        ///
        /// @return  true when the result is valid, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual bool IsFilterOutputValid(NumericFormat Term1, NumericFormat Term2, NumericFormat Result) = 0;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::HasFilterRestarted
        ///
        /// Determine if the filter has started/restarted
        ///
        /// @par Full Description
        /// Determining if the filter has started/restarted. When starting restarting put filter in not restarting state
        /// and initialize the filter data to the starting state.
        ///
        /// @param  rInitialFilterOutput    For return fromm appying filter difference equation and initializing data
        ///
        /// @return  true when the filter is restarting, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool HasFilterRestarted(NumericFormat & rInitialFilterOutput)
        {
            bool bFilterRestarted = false;

            if (m_bFirstSample)
            {
                InitFilterDataForRestart(rInitialFilterOutput);

                m_bFirstSample = false;

                bFilterRestarted = true;
            }

            return bFilterRestarted;
        }

       ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::ReconfigureWithNewCornerFrequencey
        ///
        /// Reconfigure low pass filter
        ///
        /// @par Full Description
        /// Reconfigure low pass filter synchronously, when applying filter.
        ///
        /// @param  uiCornerFreqForFilter     Corner frequency to reconfigure filter with
        ///
        /// @return  true when filter reconfigured successfully, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool ReconfigureWithNewCornerFrequencey(uint32_t uiCornerFreqToFilter)
        {
            uint32_t uiCornerFreq = GetCornerFreq();

            bool bConfigureSuccess = true;

            if (uiCornerFreqToFilter != uiCornerFreq)
            {
                uint32_t uiSamplePeriod = GetSamplePeriod();

                bConfigureSuccess = !ConfigureFilter(uiCornerFreqToFilter, uiSamplePeriod);
            }

            return bConfigureSuccess;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::GetCornerFreq
        ///
        /// Get corner frequency
        ///
        /// @par Full Description
        /// Get corner frequency the filter is configured for.
        ///
        /// @return  The corner frequency the filter is configured for
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        uint32_t GetCornerFreq()
        {
            return m_uiCornerFreq;
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::GetSamplePeriod
        ///
        /// Get sample period
        ///
        /// @par Full Description
        /// Get sample period the filter is configured for.
        ///
        /// @return  The sample period the filter is configured for
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        uint32_t GetSamplePeriod()
        {
            return m_uiSamplePeriod;
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::GetLagCoefficient
        ///
        /// Get lag coefficient
        ///
        /// @par Full Description
        /// Get lag coefficient the filter is configured for.
        ///
        /// @return  The lag coefficient the filter is configured for
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        NumericFormat GetLagCoefficient()
        {
            return m_LagCoefficient;
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::RestartFiltering
        ///
        /// Restart Low Pass Filtering
        ///
        /// @par Full Description
        /// Restart digitial filtering from first sample
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void RestartFiltering()
        {
            m_bFirstSample = false;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::SetCornerFreq
        ///
        /// Set Corner Frequency
        ///
        /// @par Full Description
        /// Set new corner frequency to input corner frequency.
        ///
        /// @param  uiCornerFreq     Corner frequency for the filter
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void SetCornerFreq(uint32_t uiCornerFreq)
        {
            m_uiCornerFreq = uiCornerFreq;
        }

         ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::SetLagCoefficient
        ///
        /// Set lag coefficient
        ///
        /// @par Full Description
        /// Set new lag coefficient to input lag coefficient.
        ///
        /// @param  LagCoefficient     Lag coefficient for the filter
        ///
        /// @return  The lag coefficient the filter is configured for
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void SetLagCoefficient(NumericFormat LagCoefficient)
        {
            m_LagCoefficient = LagCoefficient;
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::SetSamplingPeriod
        ///
        /// Set Corner Frequency
        ///
        /// @par Full Description
        /// Set new sampling period to input sampling period.
        ///
        /// @param  uiSamplingPeriod     Sampling period for the filter
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void SetSamplingPeriod(uint32_t uiSamplingPeriod)
        {
            m_uiSamplePeriod = uiSamplingPeriod;
        }

    private:
        //**************************************************************************************************************
        // Private methods and attributes
        //**************************************************************************************************************
    
        // true when applying difference equation to filter is active
        bool           m_bFilteringEnabled;

        // corner frequency for the filter
        uint32_t       m_uiCornerFreq;

        // sampling period for the filter
        uint32_t       m_uiSamplePeriod;

        // lag coefficient for the filter
        NumericFormat  m_LagCoefficient;

        bool           m_bFirstSample;

        // inhibit default constructor, copy constructor, and assignment
        LowPass();

        LowPass(LowPass &);
        
        LowPass & operator=(LowPass const&); // assign op. hidden
    };

};
#endif


