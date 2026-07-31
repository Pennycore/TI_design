#include "oled.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>

#define OLED_ADDRESS                (0x3CU)
#define OLED_WIDTH                  (128U)
#define OLED_PAGE_COUNT             (8U)
#define OLED_CHARACTER_WIDTH        (6U)
#define OLED_I2C_TIMEOUT_COUNT       (CPUCLK_FREQ / 100U)

static uint8_t g_oledReady;
static uint8_t g_oledWriteFailed;

/*
 * 等待I2C控制器空闲。
 *
 * OLED未连接、地址错误或总线被拉低时必须超时返回，
 * 否则原来的无限循环会让整车控制永久停止。
 */
static bool OLED_WaitIdle(void)
{
    uint32_t timeout = OLED_I2C_TIMEOUT_COUNT;

    while (timeout > 0U) {
        uint32_t status =
            DL_I2C_getControllerStatus(I2C_OLED_INST);

        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            return false;
        }

        if ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) {
            return true;
        }

        timeout--;
    }

    return false;
}

/*
 * 发送一个控制字节和一个数据字节。
 */
static bool OLED_Write(uint8_t control, uint8_t data)
{
    uint8_t packet[2];

    packet[0] = control;
    packet[1] = data;

    if (!OLED_WaitIdle()) {
        DL_I2C_resetControllerTransfer(I2C_OLED_INST);
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_OLED_INST);

    DL_I2C_fillControllerTXFIFO(
        I2C_OLED_INST,
        packet,
        sizeof(packet));

    DL_I2C_startControllerTransfer(
        I2C_OLED_INST,
        OLED_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        sizeof(packet));

    if (!OLED_WaitIdle()) {
        DL_I2C_resetControllerTransfer(I2C_OLED_INST);
        return false;
    }

    return true;
}

static void OLED_WriteCommand(uint8_t command)
{
    if ((g_oledWriteFailed == 0U) &&
        !OLED_Write(0x00U, command)) {
        g_oledWriteFailed = 1U;
        g_oledReady = 0U;
    }
}

static void OLED_WriteData(uint8_t data)
{
    if ((g_oledWriteFailed == 0U) &&
        !OLED_Write(0x40U, data)) {
        g_oledWriteFailed = 1U;
        g_oledReady = 0U;
    }
}

/*
 * 设置写入位置。
 */
static void OLED_SetPosition(uint8_t column, uint8_t page)
{
    if (column >= OLED_WIDTH) {
        column = 0U;
    }

    if (page >= OLED_PAGE_COUNT) {
        page = 0U;
    }

    OLED_WriteCommand((uint8_t)(0xB0U + page));
    OLED_WriteCommand((uint8_t)(0x00U + (column & 0x0FU)));
    OLED_WriteCommand((uint8_t)(0x10U + (column >> 4)));
}

/*
 * 获取5×7 ASCII字符。
 *
 * 当前只加入整车显示需要使用的字符：
 * A～Z、0～9、空格、冒号、负号和小数点。
 */
static void OLED_GetCharacterBitmap(
    char character,
    uint8_t bitmap[5])
{
    uint8_t i;

    for (i = 0U; i < 5U; i++) {
        bitmap[i] = 0x00U;
    }

    switch (character)
    {
        case 'A':
            bitmap[0] = 0x7EU;
            bitmap[1] = 0x11U;
            bitmap[2] = 0x11U;
            bitmap[3] = 0x11U;
            bitmap[4] = 0x7EU;
            break;

        case 'B':
            bitmap[0] = 0x7FU;
            bitmap[1] = 0x49U;
            bitmap[2] = 0x49U;
            bitmap[3] = 0x49U;
            bitmap[4] = 0x36U;
            break;

        case 'C':
            bitmap[0] = 0x3EU;
            bitmap[1] = 0x41U;
            bitmap[2] = 0x41U;
            bitmap[3] = 0x41U;
            bitmap[4] = 0x22U;
            break;

        case 'D':
            bitmap[0] = 0x7FU;
            bitmap[1] = 0x41U;
            bitmap[2] = 0x41U;
            bitmap[3] = 0x22U;
            bitmap[4] = 0x1CU;
            break;

        case 'E':
            bitmap[0] = 0x7FU;
            bitmap[1] = 0x49U;
            bitmap[2] = 0x49U;
            bitmap[3] = 0x49U;
            bitmap[4] = 0x41U;
            break;

        case 'F':
            bitmap[0] = 0x7FU;
            bitmap[1] = 0x09U;
            bitmap[2] = 0x09U;
            bitmap[3] = 0x09U;
            bitmap[4] = 0x01U;
            break;

        case 'G':
            bitmap[0] = 0x3EU;
            bitmap[1] = 0x41U;
            bitmap[2] = 0x49U;
            bitmap[3] = 0x49U;
            bitmap[4] = 0x7AU;
            break;

        case 'H':
            bitmap[0] = 0x7FU;
            bitmap[1] = 0x08U;
            bitmap[2] = 0x08U;
            bitmap[3] = 0x08U;
            bitmap[4] = 0x7FU;
            break;

        case 'I':
            bitmap[0] = 0x00U;
            bitmap[1] = 0x41U;
            bitmap[2] = 0x7FU;
            bitmap[3] = 0x41U;
            bitmap[4] = 0x00U;
            break;

        case 'J':
            bitmap[0] = 0x20U;
            bitmap[1] = 0x40U;
            bitmap[2] = 0x41U;
            bitmap[3] = 0x3FU;
            bitmap[4] = 0x01U;
            break;

        case 'K':
            bitmap[0] = 0x7FU;
            bitmap[1] = 0x08U;
            bitmap[2] = 0x14U;
            bitmap[3] = 0x22U;
            bitmap[4] = 0x41U;
            break;

        case 'L':
            bitmap[0] = 0x7FU;
            bitmap[1] = 0x40U;
            bitmap[2] = 0x40U;
            bitmap[3] = 0x40U;
            bitmap[4] = 0x40U;
            break;

        case 'M':
            bitmap[0] = 0x7FU;
            bitmap[1] = 0x02U;
            bitmap[2] = 0x0CU;
            bitmap[3] = 0x02U;
            bitmap[4] = 0x7FU;
            break;

        case 'N':
            bitmap[0] = 0x7FU;
            bitmap[1] = 0x04U;
            bitmap[2] = 0x08U;
            bitmap[3] = 0x10U;
            bitmap[4] = 0x7FU;
            break;

        case 'O':
            bitmap[0] = 0x3EU;
            bitmap[1] = 0x41U;
            bitmap[2] = 0x41U;
            bitmap[3] = 0x41U;
            bitmap[4] = 0x3EU;
            break;

        case 'P':
            bitmap[0] = 0x7FU;
            bitmap[1] = 0x09U;
            bitmap[2] = 0x09U;
            bitmap[3] = 0x09U;
            bitmap[4] = 0x06U;
            break;

        case 'Q':
            bitmap[0] = 0x3EU;
            bitmap[1] = 0x41U;
            bitmap[2] = 0x51U;
            bitmap[3] = 0x21U;
            bitmap[4] = 0x5EU;
            break;

        case 'R':
            bitmap[0] = 0x7FU;
            bitmap[1] = 0x09U;
            bitmap[2] = 0x19U;
            bitmap[3] = 0x29U;
            bitmap[4] = 0x46U;
            break;

        case 'S':
            bitmap[0] = 0x46U;
            bitmap[1] = 0x49U;
            bitmap[2] = 0x49U;
            bitmap[3] = 0x49U;
            bitmap[4] = 0x31U;
            break;

        case 'T':
            bitmap[0] = 0x01U;
            bitmap[1] = 0x01U;
            bitmap[2] = 0x7FU;
            bitmap[3] = 0x01U;
            bitmap[4] = 0x01U;
            break;

        case 'U':
            bitmap[0] = 0x3FU;
            bitmap[1] = 0x40U;
            bitmap[2] = 0x40U;
            bitmap[3] = 0x40U;
            bitmap[4] = 0x3FU;
            break;

        case 'V':
            bitmap[0] = 0x1FU;
            bitmap[1] = 0x20U;
            bitmap[2] = 0x40U;
            bitmap[3] = 0x20U;
            bitmap[4] = 0x1FU;
            break;

        case 'W':
            bitmap[0] = 0x3FU;
            bitmap[1] = 0x40U;
            bitmap[2] = 0x38U;
            bitmap[3] = 0x40U;
            bitmap[4] = 0x3FU;
            break;

        case 'X':
            bitmap[0] = 0x63U;
            bitmap[1] = 0x14U;
            bitmap[2] = 0x08U;
            bitmap[3] = 0x14U;
            bitmap[4] = 0x63U;
            break;

        case 'Y':
            bitmap[0] = 0x07U;
            bitmap[1] = 0x08U;
            bitmap[2] = 0x70U;
            bitmap[3] = 0x08U;
            bitmap[4] = 0x07U;
            break;

        case 'Z':
            bitmap[0] = 0x61U;
            bitmap[1] = 0x51U;
            bitmap[2] = 0x49U;
            bitmap[3] = 0x45U;
            bitmap[4] = 0x43U;
            break;

        case '0':
            bitmap[0] = 0x3EU;
            bitmap[1] = 0x51U;
            bitmap[2] = 0x49U;
            bitmap[3] = 0x45U;
            bitmap[4] = 0x3EU;
            break;

        case '1':
            bitmap[0] = 0x00U;
            bitmap[1] = 0x42U;
            bitmap[2] = 0x7FU;
            bitmap[3] = 0x40U;
            bitmap[4] = 0x00U;
            break;

        case '2':
            bitmap[0] = 0x42U;
            bitmap[1] = 0x61U;
            bitmap[2] = 0x51U;
            bitmap[3] = 0x49U;
            bitmap[4] = 0x46U;
            break;

        case '3':
            bitmap[0] = 0x21U;
            bitmap[1] = 0x41U;
            bitmap[2] = 0x45U;
            bitmap[3] = 0x4BU;
            bitmap[4] = 0x31U;
            break;

        case '4':
            bitmap[0] = 0x18U;
            bitmap[1] = 0x14U;
            bitmap[2] = 0x12U;
            bitmap[3] = 0x7FU;
            bitmap[4] = 0x10U;
            break;

        case '5':
            bitmap[0] = 0x27U;
            bitmap[1] = 0x45U;
            bitmap[2] = 0x45U;
            bitmap[3] = 0x45U;
            bitmap[4] = 0x39U;
            break;

        case '6':
            bitmap[0] = 0x3CU;
            bitmap[1] = 0x4AU;
            bitmap[2] = 0x49U;
            bitmap[3] = 0x49U;
            bitmap[4] = 0x30U;
            break;

        case '7':
            bitmap[0] = 0x01U;
            bitmap[1] = 0x71U;
            bitmap[2] = 0x09U;
            bitmap[3] = 0x05U;
            bitmap[4] = 0x03U;
            break;

        case '8':
            bitmap[0] = 0x36U;
            bitmap[1] = 0x49U;
            bitmap[2] = 0x49U;
            bitmap[3] = 0x49U;
            bitmap[4] = 0x36U;
            break;

        case '9':
            bitmap[0] = 0x06U;
            bitmap[1] = 0x49U;
            bitmap[2] = 0x49U;
            bitmap[3] = 0x29U;
            bitmap[4] = 0x1EU;
            break;

        case ':':
            bitmap[0] = 0x00U;
            bitmap[1] = 0x36U;
            bitmap[2] = 0x36U;
            bitmap[3] = 0x00U;
            bitmap[4] = 0x00U;
            break;

        case '-':
            bitmap[0] = 0x08U;
            bitmap[1] = 0x08U;
            bitmap[2] = 0x08U;
            bitmap[3] = 0x08U;
            bitmap[4] = 0x08U;
            break;

        case '.':
            bitmap[0] = 0x00U;
            bitmap[1] = 0x60U;
            bitmap[2] = 0x60U;
            bitmap[3] = 0x00U;
            bitmap[4] = 0x00U;
            break;

        case ' ':
        default:
            break;
    }
}

static void OLED_DrawCharacter(
    uint8_t column,
    uint8_t page,
    char character)
{
    uint8_t bitmap[5];
    uint8_t i;

    OLED_GetCharacterBitmap(character, bitmap);
    OLED_SetPosition(column, page);

    for (i = 0U; i < 5U; i++) {
        OLED_WriteData(bitmap[i]);
    }

    OLED_WriteData(0x00U);
}

void OLED_ShowString(
    uint8_t column,
    uint8_t page,
    const char *text)
{
    if ((g_oledReady == 0U) || (text == 0)) {
        return;
    }

    while ((*text != '\0') &&
           (column <= (OLED_WIDTH - OLED_CHARACTER_WIDTH))) {
        OLED_DrawCharacter(column, page, *text);

        column =
            (uint8_t)(column + OLED_CHARACTER_WIDTH);

        text++;
    }
}

void OLED_ShowUnsigned(
    uint8_t column,
    uint8_t page,
    uint32_t value,
    uint8_t width)
{
    char text[11];
    uint8_t i;

    if ((g_oledReady == 0U) || (width == 0U)) {
        return;
    }

    if (width > 10U) {
        width = 10U;
    }

    text[width] = '\0';

    for (i = 0U; i < width; i++) {
        text[width - 1U - i] =
            (char)('0' + (value % 10U));

        value /= 10U;
    }

    OLED_ShowString(column, page, text);
}

void OLED_ShowSigned(
    uint8_t column,
    uint8_t page,
    int32_t value,
    uint8_t width)
{
    uint32_t magnitude;

    if ((g_oledReady == 0U) || (width < 2U)) {
        return;
    }

    if (value < 0) {
        OLED_DrawCharacter(column, page, '-');

        magnitude =
            (uint32_t)(-(value + 1)) + 1U;
    } else {
        OLED_DrawCharacter(column, page, ' ');
        magnitude = (uint32_t)value;
    }

    OLED_ShowUnsigned(
        (uint8_t)(column + OLED_CHARACTER_WIDTH),
        page,
        magnitude,
        (uint8_t)(width - 1U));
}

void OLED_ClearLine(uint8_t page)
{
    uint8_t column;

    if ((g_oledReady == 0U) ||
        (page >= OLED_PAGE_COUNT)) {
        return;
    }

    OLED_SetPosition(0U, page);

    for (column = 0U; column < OLED_WIDTH; column++) {
        OLED_WriteData(0x00U);
    }
}

void OLED_Clear(void)
{
    uint8_t page;

    if (g_oledReady == 0U) {
        return;
    }

    for (page = 0U; page < OLED_PAGE_COUNT; page++) {
        OLED_ClearLine(page);
    }
}

void OLED_Init(void)
{
    g_oledReady = 0U;
    g_oledWriteFailed = 0U;

    /*
     * 上电后等待约100 ms。
     */
    delay_cycles(CPUCLK_FREQ / 10U);

    OLED_WriteCommand(0xAEU);
    OLED_WriteCommand(0x20U);
    OLED_WriteCommand(0x02U);
    OLED_WriteCommand(0xB0U);
    OLED_WriteCommand(0xC8U);
    OLED_WriteCommand(0x00U);
    OLED_WriteCommand(0x10U);
    OLED_WriteCommand(0x40U);
    OLED_WriteCommand(0x81U);
    OLED_WriteCommand(0x7FU);
    OLED_WriteCommand(0xA1U);
    OLED_WriteCommand(0xA6U);
    OLED_WriteCommand(0xA8U);
    OLED_WriteCommand(0x3FU);
    OLED_WriteCommand(0xA4U);
    OLED_WriteCommand(0xD3U);
    OLED_WriteCommand(0x00U);
    OLED_WriteCommand(0xD5U);
    OLED_WriteCommand(0x80U);
    OLED_WriteCommand(0xD9U);
    OLED_WriteCommand(0xF1U);
    OLED_WriteCommand(0xDAU);
    OLED_WriteCommand(0x12U);
    OLED_WriteCommand(0xDBU);
    OLED_WriteCommand(0x30U);
    OLED_WriteCommand(0x8DU);
    OLED_WriteCommand(0x14U);
    OLED_WriteCommand(0xAFU);

    if (g_oledWriteFailed != 0U) {
        return;
    }

    g_oledReady = 1U;
    OLED_Clear();

    if (g_oledWriteFailed != 0U) {
        g_oledReady = 0U;
    }
}

uint8_t OLED_IsReady(void)
{
    return g_oledReady;
}

void OLED_ShowTest(void)
{
    OLED_Clear();
    OLED_ShowString(20U, 2U, "OLED");
    OLED_ShowString(20U, 4U, "TEST");
    OLED_ShowString(68U, 4U, "OK");
}
