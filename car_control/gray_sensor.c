#include "gray_sensor.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>

#define GRAY_SENSOR_MUX_SETTLE_CYCLES \
    ((CPUCLK_FREQ / 1000000U) * 20U)
#define GRAY_SENSOR_ADC_TIMEOUT_COUNT \
    (CPUCLK_FREQ / 1000U)
#define GRAY_SENSOR_ADC_AVERAGE_SAMPLES    (4U)
#define GRAY_SENSOR_MIN_CALIBRATION_SPAN   (32U)

/*
 * Safe initial values for a 12-bit ADC. Replace these with measurements made
 * at the final mounting height. The variables are deliberately non-const so
 * they can also be adjusted from the CCS debugger during calibration.
 */
volatile uint16_t g_graySensorWhite[GRAY_SENSOR_CHANNEL_COUNT] = {
    3194U, 3134U, 3120U, 3231U,
    3214U, 3249U, 3297U, 3253U
};

volatile uint16_t g_graySensorBlack[GRAY_SENSOR_CHANNEL_COUNT] = {
    124U, 127U, 127U, 126U,
    127U, 127U, 107U, 128U
};

volatile uint16_t g_graySensorRaw[GRAY_SENSOR_CHANNEL_COUNT];
volatile uint32_t g_graySensorAdcTimeoutCount;

static uint8_t g_graySensorBits;
static uint8_t g_graySensorInitializedMask;

static void GraySensor_SelectChannel(uint8_t channel)
{
    uint32_t address_pins =
        GPIO_GRAY_AD0_PIN |
        GPIO_GRAY_AD1_PIN |
        GPIO_GRAY_AD2_PIN;
    uint32_t selected_pins = 0U;

    DL_GPIO_clearPins(GPIO_GRAY_PORT, address_pins);

    if ((channel & 0x01U) != 0U)
    {
        selected_pins |= GPIO_GRAY_AD0_PIN;
    }

    if ((channel & 0x02U) != 0U)
    {
        selected_pins |= GPIO_GRAY_AD1_PIN;
    }

    if ((channel & 0x04U) != 0U)
    {
        selected_pins |= GPIO_GRAY_AD2_PIN;
    }

    if (selected_pins != 0U)
    {
        DL_GPIO_setPins(GPIO_GRAY_PORT, selected_pins);
    }
}

static bool GraySensor_ReadAdc(uint16_t *result)
{
    uint32_t timeout = GRAY_SENSOR_ADC_TIMEOUT_COUNT;

    DL_ADC12_clearInterruptStatus(
        ADC_GRAY_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(ADC_GRAY_INST);
    DL_ADC12_startConversion(ADC_GRAY_INST);

    while ((DL_ADC12_getRawInterruptStatus(
                ADC_GRAY_INST,
                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) == 0U) &&
           (timeout > 0U))
    {
        timeout--;
    }

    if (timeout == 0U)
    {
        DL_ADC12_stopConversion(ADC_GRAY_INST);
        DL_ADC12_disableConversions(ADC_GRAY_INST);
        DL_ADC12_enableConversions(ADC_GRAY_INST);
        g_graySensorAdcTimeoutCount++;
        return false;
    }

    *result = (uint16_t)DL_ADC12_getMemResult(
        ADC_GRAY_INST,
        ADC_GRAY_ADCMEM_0);

    DL_ADC12_clearInterruptStatus(
        ADC_GRAY_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(ADC_GRAY_INST);
    return true;
}

static bool GraySensor_ReadChannel(
    uint8_t channel,
    uint16_t *result)
{
    uint16_t sample;
    uint32_t sum = 0U;
    uint8_t i;

    GraySensor_SelectChannel(channel);
    delay_cycles(GRAY_SENSOR_MUX_SETTLE_CYCLES);

    /*
     * Discard the first conversion after changing the 74HC4051 address. This
     * removes charge left in the ADC sample capacitor by the previous input.
     */
    if (!GraySensor_ReadAdc(&sample))
    {
        return false;
    }

    for (i = 0U; i < GRAY_SENSOR_ADC_AVERAGE_SAMPLES; i++)
    {
        if (!GraySensor_ReadAdc(&sample))
        {
            return false;
        }

        sum += sample;
    }

    *result = (uint16_t)(
        sum / GRAY_SENSOR_ADC_AVERAGE_SAMPLES);
    return true;
}

static void GraySensor_UpdateDigitalState(
    uint8_t channel,
    uint16_t raw)
{
    uint8_t channel_mask = (uint8_t)(1U << channel);
    uint16_t white = g_graySensorWhite[channel];
    uint16_t black = g_graySensorBlack[channel];
    uint16_t gray_white;
    uint16_t gray_black;
    uint16_t span;

    if (white >= black)
    {
        span = (uint16_t)(white - black);
    }
    else
    {
        span = (uint16_t)(black - white);
    }

    if (span < GRAY_SENSOR_MIN_CALIBRATION_SPAN)
    {
        return;
    }

    gray_white = (uint16_t)(
        ((uint32_t)black + 2U * (uint32_t)white) / 3U);
    gray_black = (uint16_t)(
        (2U * (uint32_t)black + (uint32_t)white) / 3U);

    if ((g_graySensorInitializedMask & channel_mask) == 0U)
    {
        uint16_t midpoint = (uint16_t)(
            ((uint32_t)white + (uint32_t)black) / 2U);
        bool is_white =
            (white > black) ? (raw >= midpoint) : (raw <= midpoint);

        if (is_white)
        {
            g_graySensorBits |= channel_mask;
        }
        else
        {
            g_graySensorBits &= (uint8_t)(~channel_mask);
        }

        g_graySensorInitializedMask |= channel_mask;
        return;
    }

    if (white > black)
    {
        if ((g_graySensorBits & channel_mask) != 0U)
        {
            if (raw <= gray_black)
            {
                g_graySensorBits &= (uint8_t)(~channel_mask);
            }
        }
        else if (raw >= gray_white)
        {
            g_graySensorBits |= channel_mask;
        }
    }
    else
    {
        if ((g_graySensorBits & channel_mask) != 0U)
        {
            if (raw >= gray_black)
            {
                g_graySensorBits &= (uint8_t)(~channel_mask);
            }
        }
        else if (raw <= gray_white)
        {
            g_graySensorBits |= channel_mask;
        }
    }
}

void GraySensor_Init(void)
{
    uint8_t channel;

    g_graySensorBits = 0xFFU;
    g_graySensorInitializedMask = 0U;
    g_graySensorAdcTimeoutCount = 0U;

    for (channel = 0U;
         channel < GRAY_SENSOR_CHANNEL_COUNT;
         channel++)
    {
        g_graySensorRaw[channel] = 0U;
    }

    GraySensor_SelectChannel(0U);
    DL_ADC12_enableConversions(ADC_GRAY_INST);
}

uint8_t GraySensor_Read(void)
{
    uint8_t channel;

    for (channel = 0U;
         channel < GRAY_SENSOR_CHANNEL_COUNT;
         channel++)
    {
        uint16_t raw;

        if (GraySensor_ReadChannel(channel, &raw))
        {
            g_graySensorRaw[channel] = raw;
            GraySensor_UpdateDigitalState(channel, raw);
        }
    }

    return g_graySensorBits;
}
