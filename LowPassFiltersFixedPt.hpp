////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file LowPassFilterFixPt.hpp
///
/// The base class for the low pass filters that use fixed point arithmetic and derived from the template base class.
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
#ifndef LOW_PASS_FIXED_PT_HPP
#define LOW_PASS_FIXED_PT_HPP

// SYSTEM INCLUDES

// C PROJECT INCLUDES


// C++ PROJECT INCLUDES
#include "LowPass.hpp"

namespace LowPassFilters
{

    class LowPassFilterFixedPt : public LowPass<int32_t>
    {
    public:
        //**************************************************************************************************************
        // Public methods
        //**************************************************************************************************************
    
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::LowPassFixedPt
        ///
        /// Constructor
        ///
        /// @par Full Description
        /// Base class contructor for the low pass filter using fixed point arithmetic
        ///
        /// @param  uiCornerFreq         Initial corner frequency 
        /// @param  uiSamplingPeriod     Sampling period for the filter
        /// @param  LagCoefficient       InitialLagCoefficient
        /// @param  AtoDResolutionBits   Bit resolution of ADC channels
        /// @param  NumberOfPoles        Number of poles the filter implements
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        LowPassFilterFixedPt(uint32_t uiCornerFreq,
                             uint32_t uiSamplingPeriod,
                             uint32_t LagCoeffecient,
                             uint32_t AtoDResolutionBits,
                             uint32_t NumberOfPoles = DEFAULT_NUMBER_OF_POLES)
            : m_AtoDResolutionBits(AtoDResolutionBits),
              m_NumberOfFrcntlBits((sizeof(int) * CHAR_BIT) - AtoDResolutionBits - 1),
              m_RoundOffValue(1 << ((sizeof(int) * CHAR_BIT) - AtoDResolutionBits - 2)),
              m_ScaledIntegerLSBBitPos((sizeof(int) * CHAR_BIT) - AtoDResolutionBits - 1),
              m_IntMSBSet((uint32_t)INT_MAX + 1),
              m_IntNumBitsInInt(sizeof(int) * CHAR_BIT),
              m_NumberOfPoles(NumberOfPoles),
              LowPass<int32_t>(uiCornerFreq, uiSamplingPeriod, LagCoeffecient)
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::~LowPassFixedPt
        ///
        /// Destructor
        ///
        /// @par Full Description
        /// Base class destructor for the low pass filter using fixed point arithmetic
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual ~LowPassFilterFixedPt()
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::ApplyFilter
        ///
        /// Apply the low pass filter difference equation to unfiltered ADC input data
        ///
        /// @par Full Description
        /// Virtual method for the applying the low pass filter
        ///
        /// @param  adcValueRead     Raw ADC data read by the ADC driver
        /// @param  uiCornerFreq     Corner Frequency for the filter.
        /// @param  rFilterOutput    Output from filter difference equation
        ///
        /// @return  true when the filter was applied, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual bool ApplyFilter(int32_t iAtoDValueRead, uint32_t uiCornerFreqToFilter, int32_t & rFilterOutput);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::ConfigureFilter
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
        virtual bool ConfigureFilter(uint32_t uiCornerFreq, uint32_t uiSamplePeriod);

    protected:
        //**************************************************************************************************************
        // Protected methods and attributes
        //**************************************************************************************************************
    
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::InitFilterDataForRestart
        ///
        /// Initialize filter data to put the filter in it's initial state
        ///
        /// @par Full Description
        /// virtual method for initializing filter data.
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual void InitFilterDataForRestart(int32_t InitialFilterOutput);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::IsFilterResultValid
        ///
        /// Determine validity of low pass filter output
        ///
        /// @par Full Description
        /// Method for determining the validity of the low filter output generated by difference equation
        ///
        /// @param  DiffEqTerm1     First term of difference equation add
        /// @param  Term2           Second term of difference equation add
        /// @param  Result          Result of difference equation add
        ///
        /// @return  true when the result is valid, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool IsFilterOutputValid(int32_t iDiffEqTerm1, int32_t iDiffEqTerm2, int32_t iFilterOuput);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::IsThereOverflowFromAddSbtrct
        ///
        /// Determine validity of low pass filter output
        ///
        /// @par Full Description
        /// Method for determining if overflow occurred during a binary add/subtract
        ///
        /// @param  uiTerm1           First term of binary aritmetic operation of filter being checked
        /// @param  uiTerm2           Second term of binary aritmetic operation of filter being checked.
        /// @param  uiResult          Result of binary arithmetic operation of filter being checked
        ///
        /// @return  true when overflow occurred in the addition/subtraction, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool IsThereOverflowFromAddSbtrct(uint32_t uiTerm1, uint32_t uiTerm2, uint32_t uiResult);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::CalcDiffEquation
        ///
        /// Calculate filtered output when applying the low pass filter
        ///
        /// @par Full Description
        /// Virtual method for calculating filtered ADC value from the difference equation
        /// y(n) = y(n-1) + LagCoefficient * ( Raw_ADC_value_read - y(n-1) )
        /// where 
        ///   LagCoefficient
        ///    = (2 * pi * corner_frequenchy * sample_period ) / ( ( 2 * pi  * corner_frequenchy * sample_period ) + 2 )
        ///
        /// @param  iScaledAtoD           Raw ADC data scaled to a binary point
        /// @param  iLagCoefficient       Coefficient of the lag term.
        /// @param  rFilteredValue        Value of raw ADC data filtered
        ///
        /// @return  true when calculation occurred without error, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual bool CalcDiffEquation(int32_t   iScaledAtoD,
                                      int32_t   iLagCoefficient,
                                      int32_t & rFilteredValue);
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::GetNumberOfADCResolutionBits
        ///
        /// Get the resolution in bits of the ADC inputs
        ///
        /// @par Full Description
        /// Get the resolution in bits of the ADC inputs
        ///
        /// @return  Number of ADC resolution bits
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        uint32_t GetNumberOfADCResolutionBits();

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::GetNumberOfBitsInInt
        ///
        /// Get the number of bits in an int
        ///
        /// @par Full Description
        /// Get the number of bits in an int
        ///
        /// @return  Number of bits in an int.
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        uint32_t GetNumberOfBitsInInt();

        // Maximum number of poles that filter can process
        static const uint32_t  MAX_NUMBER_OF_POLES = 4;
        
        // Approx of Pi * .000001 with 32 bits of fractional precision
        static const uint32_t  PI_OMEGA          = 13493;

        // For rounding off 64 bit fractional value;
        static const uint64_t  ROUND_OFF_FRAC_64 = 0x800000000000;

        // Number of poles in the filter
        uint32_t m_NumberOfPoles;

        // Poles used in applying the filter.
        int32_t  m_pole[MAX_NUMBER_OF_POLES];

    private:
        //**************************************************************************************************************
        // Private methods and attributes
        //**************************************************************************************************************
    
        // Default number of poles to use in filtering
        const static uint32_t DEFAULT_NUMBER_OF_POLES = 1;
        
        // Number of ADC resolution bits
        uint32_t m_AtoDResolutionBits;

        // Number of fractionas bits in the scaled integers.  
        // Places to the right of the binary point.
        uint32_t m_NumberOfFrcntlBits;

        // For rounding to least significant bit of scaled data
        uint32_t m_RoundOffValue;

        // Bit position of LSB of whole number part of scaled number
        uint32_t m_ScaledIntegerLSBBitPos;

        // Most significant bit for ints is a one, all others are zero
        uint32_t m_IntMSBSet;

        // Number of bits in an integer 
        uint32_t m_IntNumBitsInInt;

        // inhibit default constructor, copy constructor, and assignment
        LowPassFilterFixedPt();

        LowPassFilterFixedPt(LowPassFilterFixedPt &);

        LowPassFilterFixedPt & operator=(LowPassFilterFixedPt const&); // assign op. hidden
    };
};
#endif

