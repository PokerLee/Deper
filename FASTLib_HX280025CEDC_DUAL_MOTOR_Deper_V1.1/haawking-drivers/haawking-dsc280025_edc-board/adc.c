//###########################################################################
//
// FILE:   adc.c
//
// TITLE:  H28x ADC driver.
//
//###########################################################################
// $HAAWKING Release: Hal Driver Support Library V1.0.5 $
// $Release Date: 2023-04-28 06:49:27 $
// $Copyright:
// Copyright (C) 2019-2023 Beijing Haawking Technology Co.,Ltd - http://www.haawking.com/
//###########################################################################


#include "adc.h"

//*****************************************************************************
//
// Defines for locations of ADC calibration functions in OTP for use in
// ADC_setVREF(). Not intended for use by application code.
//
//*****************************************************************************
#define ADC_OFFSET_TRIM_OTP    0x7A0864U

#define ADC_VOLTAGE_REF_REG_OFFSET              8U

//*****************************************************************************
//
// ADC_setVREF
//
//*****************************************************************************
void
ADC_setVREF(uint32_t base, ADC_ReferenceMode refMode,
            ADC_ReferenceVoltage refVoltage)
{
    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    EALLOW;
    //
    // Configure the reference mode (internal or external).
    //
    if((refMode == ADC_REFERENCE_INTERNAL)&&(refVoltage == ADC_REFERENCE_2_5V))
    {
        HWREG( ADCA_BASE + ADC_O_CTL1 ) |= ADC_CTL1_PGADIS;
        HWREG( ADCC_BASE + ADC_O_CTL1 ) |= ADC_CTL1_PGADIS;
        HWREG(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) &=
             ( ~ ( ASYSCTL_ANAREFCTL_ANAREFCSEL | ASYSCTL_ANAREFCTL_ANAREFASEL |
                ASYSCTL_ANAREFCTL_ANAREFSEL ))| ASYSCTL_ANAREFCTL_ANAREF2P5SEL;
    }
    else if((refMode == ADC_REFERENCE_INTERNAL)&&(refVoltage == ADC_REFERENCE_3_3V))
    {
    	HWREG(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) = ASYSCTL_ANAREFCTL_ANAREFASEL
    			                                  | ASYSCTL_ANAREFCTL_ANAREFCSEL;
    }
    else if(refMode == ADC_REFERENCE_EXTERNAL)
    {
        HWREG(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) =
             ( ~( ASYSCTL_ANAREFCTL_ANAREFASEL | ASYSCTL_ANAREFCTL_ANAREFCSEL )
            		 | ASYSCTL_ANAREFCTL_ANAREFSEL );
    }
    EDIS;
    //
    // Set ADC offset trim for the all the ADC instances
    //
    ADC_setOffsetTrimAll(refMode, refVoltage);
}

//*****************************************************************************
//
// ADC_setOffsetTrim
//
//*****************************************************************************
void
ADC_setOffsetTrim(uint32_t base)
{
    uint16_t *inter_offset;
    uint16_t *exter_offset;
    uint32 adctrim_flag;
    uint16_t *inter_key;
    uint16_t *exter_key;
    uint32_t moduleShiftVal;
    uint32_t analogRefRegVal;
    ADC_ReferenceMode refMode;
    ADC_ReferenceVoltage refVoltage;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Assign a shift amount corresponding to which ADC module is being
    // configured.
    //
    switch(base)
    {
        case ADCA_BASE:
            moduleShiftVal = 0U;
            break;
        case ADCC_BASE:
            moduleShiftVal = 1U;
            break;
        default:
            //
            // Invalid base address!!
            //
            moduleShiftVal = 0U;
            break;
    }
    //
    // Read the Analog Reference Control Register value to determine the
    // ADC reference mode and reference voltage value.
    //
    analogRefRegVal = HWREG(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL);

    //
    // Calculate refMode and refVoltage based on input ADC base
    //
    refMode = (ADC_ReferenceMode)((analogRefRegVal >> moduleShiftVal) & 1U);
    refVoltage = (ADC_ReferenceVoltage)((analogRefRegVal >>
                 (ADC_VOLTAGE_REF_REG_OFFSET + moduleShiftVal)) & 1U);

    //
    // Offset trim for internal VREF 3.3V is unique and stored in upper byte.
    //
    if(refMode == ADC_REFERENCE_INTERNAL)
    {
    	adctrim_flag = 1U;
    }
    else
    {
    	adctrim_flag = 0U;
    }

    //
    // Set up pointer to offset trim in OTP for ADCA.
    //
	if(base == ADCA_BASE)
	{
      inter_offset = (uint16_t *)((uint32_t)ADC_OFFSET_TRIM_OTP);
      inter_key = (uint16_t *)((uint32_t)ADC_OFFSET_TRIM_OTP + (uint32_t)2);
      exter_offset = (uint16_t *)((uint32_t)ADC_OFFSET_TRIM_OTP+4);
      exter_key = (uint16_t *)((uint32_t)ADC_OFFSET_TRIM_OTP + (uint32_t)6);
	}
	else if(base == ADCC_BASE)
	{
	  inter_offset = (uint16_t *)((uint32_t)ADC_OFFSET_TRIM_OTP);
      inter_key = (uint16_t *)((uint32_t)ADC_OFFSET_TRIM_OTP + (uint32_t)2);
      exter_offset = (uint16_t *)((uint32_t)ADC_OFFSET_TRIM_OTP+4);
      exter_key = (uint16_t *)((uint32_t)ADC_OFFSET_TRIM_OTP + (uint32_t)6);
	}
    //
    // Get offset trim from OTP and write it to the register for ADCA.
    //
    EALLOW;
    if(adctrim_flag == 1U)
    {
    	if(*inter_key == 0x5A5A)
    	{
         HWREG(base + ADC_O_OFFTRIM) = *inter_offset;
    	}
    }
    else
    {
    	if(*exter_key == 0x5A5A)
    	{
         HWREG(base + ADC_O_OFFTRIM) = *exter_offset;
    	}
    }

    //
    // Configure the reference mode (internal or external).
    //
    if((refMode == ADC_REFERENCE_INTERNAL)&&(refVoltage == ADC_REFERENCE_2_5V))
    {
        HWREG( ADCA_BASE + ADC_O_CTL1 ) |= ADC_CTL1_PGADIS;
        HWREG( ADCC_BASE + ADC_O_CTL1 ) |= ADC_CTL1_PGADIS;
        HWREG(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) &=
             ( ~ ( ASYSCTL_ANAREFCTL_ANAREFCSEL | ASYSCTL_ANAREFCTL_ANAREFASEL |
                ASYSCTL_ANAREFCTL_ANAREFSEL ))| ASYSCTL_ANAREFCTL_ANAREF2P5SEL;
    }
    else if((refMode == ADC_REFERENCE_INTERNAL)&&(refVoltage == ADC_REFERENCE_3_3V))
    {
    	HWREG(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) = ASYSCTL_ANAREFCTL_ANAREFASEL
    			                                  | ASYSCTL_ANAREFCTL_ANAREFCSEL;
    }
    else if(refMode == ADC_REFERENCE_EXTERNAL)
    {
        HWREG(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) =
             ( ~( ASYSCTL_ANAREFCTL_ANAREFASEL | ASYSCTL_ANAREFCTL_ANAREFCSEL )
            		 | ASYSCTL_ANAREFCTL_ANAREFSEL );
    }
    EDIS;
}

//*****************************************************************************
//
// ADC_setOffsetTrimAll
//
//*****************************************************************************
void
ADC_setOffsetTrimAll(ADC_ReferenceMode refMode, ADC_ReferenceVoltage refVoltage)
{
    uint16_t *inter_offset;
    uint16_t *exter_offset;
    uint16_t *inter_key;
    uint16_t *exter_key;
    uint32 adctrim_flag = 0;

    //
    // Offset trim for internal VREF 3.3V is unique and stored in upper byte.
    //
    if(refMode == ADC_REFERENCE_EXTERNAL)
    {
    	adctrim_flag = 0U;
    }
    else if(refMode == ADC_REFERENCE_INTERNAL)
    {
    	adctrim_flag = 1U;
    }
	
	
   //
    // Set up pointer to offset trim in OTP for ADCA.
    //
    inter_offset = (uint16_t *)((uint32_t)ADC_OFFSET_TRIM_OTP);
    inter_key = (uint16_t *)((uint32_t)ADC_OFFSET_TRIM_OTP + (uint32_t)2);
    exter_offset = (uint16_t *)((uint32_t)ADC_OFFSET_TRIM_OTP+4);
    exter_key = (uint16_t *)((uint32_t)ADC_OFFSET_TRIM_OTP + (uint32_t)6);
    //
    // Get offset trim from OTP and write it to the register for ADCA.
    //
    EALLOW;
    if(adctrim_flag == 1U)
    {
    	if(*inter_key == 0x5A5A)
    	{
         HWREG(ADCA_BASE + ADC_O_OFFTRIM) = *inter_offset;
    	}
    }
    else
    {
    	if(*exter_key == 0x5A5A)
    	{
         HWREG(ADCA_BASE + ADC_O_OFFTRIM) = *exter_offset;
    	}
    }

    //
    // Set up pointer to offset trim in OTP for ADCB.
    //
    inter_offset = (uint16_t *)(ADC_OFFSET_TRIM_OTP + (uint32_t)8U);
    inter_key = (uint16_t *)(ADC_OFFSET_TRIM_OTP + (uint32_t)10);
    exter_offset = (uint16_t *)(ADC_OFFSET_TRIM_OTP + (uint32_t)12);
    exter_key = (uint16_t *)(ADC_OFFSET_TRIM_OTP + (uint32_t)14);

    //
    // Get offset trim from OTP and write it to the register for ADCC.
    //
    if(adctrim_flag == 1U)
    {
    	if(*inter_key == 0x5A5A)
    	{
         HWREG(ADCC_BASE + ADC_O_OFFTRIM) = *inter_offset;
    	}
    }
    else
    {
    	if(*exter_key == 0x5A5A)
    	{
         HWREG(ADCC_BASE + ADC_O_OFFTRIM) = *exter_offset;
    	}
    }
    //
    // Configure the reference mode (internal or external).
    //
    if((refMode == ADC_REFERENCE_INTERNAL)&&(refVoltage == ADC_REFERENCE_2_5V))
    {
        HWREG( ADCA_BASE + ADC_O_CTL1 ) |= ADC_CTL1_PGADIS;
        HWREG( ADCC_BASE + ADC_O_CTL1 ) |= ADC_CTL1_PGADIS;
        HWREG(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) &=
             ( ~ ( ASYSCTL_ANAREFCTL_ANAREFCSEL | ASYSCTL_ANAREFCTL_ANAREFASEL |
                ASYSCTL_ANAREFCTL_ANAREFSEL ))| ASYSCTL_ANAREFCTL_ANAREF2P5SEL;
    }
    else if((refMode == ADC_REFERENCE_INTERNAL)&&(refVoltage == ADC_REFERENCE_3_3V))
    {
    	HWREG(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) = ASYSCTL_ANAREFCTL_ANAREFASEL
    			                                  | ASYSCTL_ANAREFCTL_ANAREFCSEL;
    }
    else if(refMode == ADC_REFERENCE_EXTERNAL)
    {
        HWREG(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) =
             ( ~( ASYSCTL_ANAREFCTL_ANAREFASEL | ASYSCTL_ANAREFCTL_ANAREFCSEL )
            		 | ASYSCTL_ANAREFCTL_ANAREFSEL );
    }
    EDIS;
}


//*****************************************************************************
//
// ADC_setPPBTripLimits
//
//*****************************************************************************
void
ADC_setPPBTripLimits(uint32_t base, ADC_PPBNumber ppbNumber,
                     int32_t tripHiLimit, int32_t tripLoLimit)
{
    uint32_t ppbHiOffset;
    uint32_t ppbLoOffset;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));
    ASSERT((tripHiLimit <= 65535) && (tripHiLimit >= -65536));
    ASSERT((tripLoLimit <= 65535) && (tripLoLimit >= -65536));

    //
    // Get the offset to the appropriate trip limit registers.
    //
    ppbHiOffset = (ADC_PPBxTRIPHI_STEP * (uint32_t)ppbNumber) +
                  ADC_O_PPB1TRIPHI;
    ppbLoOffset = (ADC_PPBxTRIPLO_STEP * (uint32_t)ppbNumber) +
                  ADC_O_PPB1TRIPLO;

    EALLOW;

    //
    // Set the trip high limit.
    //
    HWREG(base + ppbHiOffset) =
        (HWREG(base + ppbHiOffset) & ~ADC_PPBTRIP_MASK) |
        ((uint32_t)tripHiLimit & ADC_PPBTRIP_MASK);

    //
    // Set the trip low limit.
    //
    HWREG(base + ppbLoOffset) =
        (HWREG(base + ppbLoOffset) & ~ADC_PPBTRIP_MASK) |
        ((uint32_t)tripLoLimit & ADC_PPBTRIP_MASK);

    EDIS;
}
