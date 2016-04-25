////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file LowPass.cpp
///
/// Implementation of the LowPassFilters class
///
/// @see LowPass.hpp for a detailed description of this class.
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
#include "LowPass.hpp"

namespace LowPassFilters
{
    //**********************************************************************************************************************
    // PUBLIC METHODS
    //**********************************************************************************************************************
        
    template <typename NumericFormat>
    LowPass<NumericFormat>::LowPass(uint32_t       uiCornerFreq,
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
    /// FUNCTION NAME: LowPass<NumericFormat>::EnableFiltering()
    ///
    /// Enable low pass filtering
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <typename NumericFormat>
    bool LowPass<NumericFormat>::IsFilteringEnabled()
    {
        return  m_bFilteringenabled;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPass<NumericFormat>::DisableFiltering()
    ///
    /// Disable low pass filtering
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //template <typename NumericFormat>
    //void LowPass<NumericFormat>::DisableFiltering()
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPass<NumericFormat>::RestartFiltering
    ///
    /// Restart Low Pass Filtering
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //template <typename NumericFormat>
    //void LowPass<NumericFormat>::RestartFiltering()
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPass<NumericFormat>::SetCornerFreq
    ///
    /// Set Corner Frequency
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //template <typename NumericFormat>
};
