////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file ADCFilter.cpp
///
/// Implementation of the ADCFilter class
///
/// @see ADCFilter.hpp for a detailed description of this class.
///
/// @if REVISION_HISTORY_INCLUDED
/// @par Edit History
/// - thaley1   22-Apr-2016 Original Implementation
/// @endif
///
/// @ingroup ???
///
/// @par Copyright (c) 2016 Rockwell Automation Technologies, Inc.  All rights reserved.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES

// C PROJECT INCLUDES

// C++ PROJECT INCLUDES
#include "ADCFilter.hpp"

namespace LowPassFilters
{
    //**************************************************************************************************************
    // Public methods
    //**************************************************************************************************************
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: ADCFilter::ApplyFilter
    ///
    /// Apply the low pass filter difference equation to unfiltered ADC input data
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool ADCFilter::ApplyFilter(int32_t iAtoDValueRead, uint32_t uiCornerFreqToFilter,  int32_t & rFilterOutput)
    {
        int32_t iScaledAtoD = iAtoDValueRead << 15;

        rFilterOutput = iScaledAtoD;
            
        bool bFilteringIsNotEnabled = !IsFilteringEnabled();

        if (bFilteringIsNotEnabled)
        {
            return false;
        }

        bool bReconfigureFailure = !ReconfigureWithNewCornerFrequencey(uiCornerFreqToFilter);

        if (bReconfigureFailure)
        {
            return false;
        }

        bool bLowPassFilteringStarting = HasFilterRestarted(rFilterOutput);

        if (bLowPassFilteringStarting )
        {
           return true;
        }

        int32_t iCurrentFilterOutput = iScaledAtoD;

        bool bFilterApplied = true;

        for (int32_t i = 0;
                (i < sizeof(m_pole) / sizeof(uint32_t))
             && (i < m_NumberOfPoles);
             i++)
        {
            //  y = adcData->y_last_0 + (x - adcData->y_last_0)/256;
            int32_t iLagTerm = iCurrentFilterOutput - m_pole[i];

            bool bInvalidFilterOutput = !IsFilterOutputValid(iLagTerm, iCurrentFilterOutput, m_pole[i]);

            if (bInvalidFilterOutput)
            {
                iCurrentFilterOutput = iScaledAtoD;

                bFilterApplied = false;

                break;
            }
                    
            if (m_FrequencyShiftFactor < 0)
            {
                iLagTerm <<= m_FrequencyShiftFactor;
            }
            else
            {
                iLagTerm >>= m_FrequencyShiftFactor;
            }

            iCurrentFilterOutput = m_pole[i] + iLagTerm;

            bInvalidFilterOutput = IsThereOverflowFromAddSbtrct(m_pole[i], iLagTerm, iCurrentFilterOutput);

            if (bInvalidFilterOutput)
            {
                iCurrentFilterOutput = iScaledAtoD;

                bFilterApplied = false;

                break;
            }

            m_pole[i] = iCurrentFilterOutput;
        }

        rFilterOutput = iCurrentFilterOutput;

        return bFilterApplied;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: ADCFilter::ConfigureFilter
    ///
    /// Configure filter difference equation coefficients
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool ADCFilter::ConfigureFilter(uint32_t uiCornerFreq, unsigned uiSamplingPeriod)
    {
        bool bFilterConfigured = true;

        int32_t iSmallestShiftFactor = GetNumberOfBitsInInt();
            
        iSmallestShiftFactor -= NO_SHIFT_BIT_POS;

        iSmallestShiftFactor -= GetNumberOfADCResolutionBits();

        switch (uiCornerFreq)
        {
        case FREQ_100_HZ:
                
            m_FrequencyShiftFactor = iSmallestShiftFactor;

            break;

        case FREQ_50_HZ:
        
            m_FrequencyShiftFactor = iSmallestShiftFactor + 1;

            break;

        case FREQ_25_HZ:
        
            m_FrequencyShiftFactor = iSmallestShiftFactor + 2;

            break;

        case FREQ_10_HZ:
     
            m_FrequencyShiftFactor = iSmallestShiftFactor + 3;

            break;

        case FREQ_5_HZ:
        
            m_FrequencyShiftFactor = iSmallestShiftFactor + 4;

            break;

        case FREQ_1_HZ:
          
            m_FrequencyShiftFactor = iSmallestShiftFactor + 6;

            break;

         default:
         
             bFilterConfigured = false;

            break;
        }

        if (bFilterConfigured)
        {
            SetCornerFreq(uiCornerFreq);
        }

        return bFilterConfigured;
    }
};