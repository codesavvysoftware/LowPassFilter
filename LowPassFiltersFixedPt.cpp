////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file LowPassFiltersFixedPt.cpp
///
/// Implementation of the LowPassFiltersFixedPt class
///
/// @see LowPassFiltersFixedPt.hpp for a detailed description of this class.
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
// (none)

// C++ PROJECT INCLUDES
#include "LowPassFiltersFixedPt.hpp"
//using namespace LowPassFilters;

namespace LowPassFilters
{
   
    //**************************************************************************************************************
    // Public methods
    //**************************************************************************************************************
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::ApplyFilter
    ///
    /// Apply the low pass filter difference equation to unfiltered ADC input data
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassFilterFixedPt::ApplyFilter(int32_t iAtoDValueRead, uint32_t uiCornerFreqToFilter, int32_t & rFilterOutput)
    {
        int32_t iScaledAtoD = iAtoDValueRead << m_ScaledIntegerLSBBitPos;

        rFilterOutput = iScaledAtoD;

        bool bFilteringIsNotEnabled = !this->IsFilteringEnabled();//  IsFilteringEnabled();

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

        int32_t iLagCoefficient = GetLagCoefficient();

        bool bErrorsCalcOfDiffEquation = !CalcDiffEquation(iScaledAtoD, iLagCoefficient, rFilterOutput);

        if (bErrorsCalcOfDiffEquation)
        {
            RestartFiltering();

            rFilterOutput = iScaledAtoD;

            return false;
        }

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::ConfigureFilter
    ///
    /// Configure filter difference equation coefficients
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassFilterFixedPt::ConfigureFilter(uint32_t uiCornerFreq, uint32_t uiSamplePeriod)
    {
        SetCornerFreq(uiCornerFreq);

        SetSamplingPeriod(uiSamplePeriod);

        // pi * corner_freq_hz * sample_period
        uint64_t llAccum = PI_OMEGA * uiCornerFreq * uiSamplePeriod;


        // (1-pi * corner_freq_hz * sample_period)
        uint64_t llSecondTermInApprox = (0x100000000 - llAccum);

        llAccum *= llSecondTermInApprox;

        llAccum += ROUND_OFF_FRAC_64;

        llAccum >>= (m_IntNumBitsInInt + (m_IntNumBitsInInt - m_NumberOfFrcntlBits));

        uint32_t uiLagCoefficient = llAccum;

        SetLagCoefficient(uiLagCoefficient);

        return true;
    }

    //**************************************************************************************************************
    // Protected methods and attributes
    //**************************************************************************************************************
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::InitFilterDataForRestart
    ///
    /// Initialize filter data to put the filter in it's initial state
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void LowPassFilterFixedPt::InitFilterDataForRestart(int32_t InitialFilterOutput)
    {            
        for (int32_t i = 0; 
                ( i < sizeof(m_pole) / sizeof(uint32_t))
             && ( i < m_NumberOfPoles );
             i++)
        {
            m_pole[i] = InitialFilterOutput;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::IsFilterResultValid
    ///
    /// Determine validity of low pass filter output
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassFilterFixedPt::IsFilterOutputValid(int32_t iDiffEqTerm1, int32_t iDiffEqTerm2, int32_t iFilterOuput)
    {
        return IsThereOverflowFromAddSbtrct(iDiffEqTerm1, iDiffEqTerm2, iFilterOuput);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::IsThereOverflowFromAddSbtrct
    ///
    /// Determine validity of low pass filter output
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassFilterFixedPt::IsThereOverflowFromAddSbtrct(uint32_t uiTerm1, uint32_t uiTerm2, uint32_t uiResult)
    {
        bool bOverflowOccurred = false;

        uint32_t uiTerm1MSB = uiTerm1 & m_IntMSBSet;

        uint32_t uiTerm2MSB = uiTerm2 & m_IntMSBSet;

        uint32_t uiResultMSB = uiResult & m_IntMSBSet;

        if (!(uiTerm1MSB ^ uiTerm2MSB) && (uiTerm1MSB ^ uiResultMSB))
        {
            bOverflowOccurred = true;
        }

        return bOverflowOccurred;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::CalcDiffEquation
    ///
    /// Calculate filtered output when applying the low pass filter
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassFilterFixedPt::CalcDiffEquation(int32_t   iScaledAtoD,
                                                int32_t   iLagCoefficient,
                                                int32_t & rFilteredValue)
    {
        int32_t i = 0;

        int32_t iCurrentFilterResult = iScaledAtoD;

        for (i = 0; 
                ( i <( sizeof(m_pole) / sizeof(int) ))
             && ( i < m_NumberOfPoles );
             i++)
        {
            int32_t iSecondTermOfDiffEq = iCurrentFilterResult - m_pole[i];

            bool bSubtractionOverflowed = IsThereOverflowFromAddSbtrct(iCurrentFilterResult, m_pole[i], iSecondTermOfDiffEq);

            if (bSubtractionOverflowed)
            {
                return false;
            }

            int32_t iSecondTermIntPart = iSecondTermOfDiffEq >> m_NumberOfFrcntlBits;

            int32_t iFilterResultIntPart = iSecondTermIntPart * iLagCoefficient;

            uint32_t uiFractionMask = (m_RoundOffValue - 1) | m_RoundOffValue;

            int32_t iFilterResultFracPart = iSecondTermOfDiffEq & uiFractionMask;

            iFilterResultFracPart *= iLagCoefficient;

            iFilterResultFracPart += m_RoundOffValue;

            iFilterResultFracPart >>= (m_IntNumBitsInInt - m_NumberOfFrcntlBits);

            iCurrentFilterResult = iFilterResultIntPart + iFilterResultFracPart;

            bool bAdditionOverflowed = IsThereOverflowFromAddSbtrct(iFilterResultIntPart, iFilterResultFracPart, iCurrentFilterResult);

            if (bAdditionOverflowed)
            {
                return false;
            }

            int32_t iLagValue = iCurrentFilterResult;

            iCurrentFilterResult += m_pole[i];

            bAdditionOverflowed = IsThereOverflowFromAddSbtrct(iLagValue, m_pole[i], iCurrentFilterResult);

            if (bAdditionOverflowed)
            {
                return false;
            }

            m_pole[i] = iCurrentFilterResult;

        }

        rFilteredValue = iCurrentFilterResult;

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::GetNumberOfADCResolutionBits
    ///
    /// Get the resolution in bits of the ADC inputs
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    uint32_t LowPassFilterFixedPt::GetNumberOfADCResolutionBits()
    {
        return m_AtoDResolutionBits;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::GetNumberOfBitsInInt
    ///
    /// Get the number of bits in an int
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    uint32_t LowPassFilterFixedPt::GetNumberOfBitsInInt()
    {
        return m_IntNumBitsInInt;
    }
};

