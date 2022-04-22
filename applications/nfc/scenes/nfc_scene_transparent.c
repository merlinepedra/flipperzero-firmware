#include "../nfc_i.h"
#include <st25r3916.h>
#include <st25r3916_irq.h>
#include <stm32wbxx_ll_dma.h>
#include <stm32wbxx_ll_tim.h>
#include <furi/common_defines.h>

#define TAG "FuriHalNfc"

#define BUFF_SIZE (10000)

uint16_t one = 0;
uint16_t zero = 0;

static void add_one(uint16_t* buff, size_t* i) {
    buff[*i] = one;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = one;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = one;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = one;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
}

static void add_zero(uint16_t* buff, size_t* i) {
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = one;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = one;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = one;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
    buff[*i] = one;
    *i = *i + 1;
    buff[*i] = zero;
    *i = *i + 1;
}

void add_byte(uint16_t* buff, size_t* i, uint8_t byte, bool parity) {
    for(size_t j = 0; j < 8; j++) {
        if(byte & (1 << j)) {
            add_one(buff, i);
        } else {
            add_zero(buff, i);
        }
    }
    if(parity) {
        add_one(buff, i);
    } else {
        add_zero(buff, i);
    }
}

uint16_t buff[BUFF_SIZE] = {};
uint16_t buff2[BUFF_SIZE] = {};

uint32_t arr[] = {37, 37, 37, 37, 37, 36};

size_t i = 0;

void nfc_scene_transparent_on_enter(void* context) {
    Nfc* nfc = context;

    // uint8_t reg = 0;
    ReturnCode ret;

    furi_hal_nfc_exit_sleep();

    ret = rfalSetMode(RFAL_MODE_LISTEN_NFCA, RFAL_BR_106, RFAL_BR_106);
    FURI_LOG_W("LISTEN START", "RETURN %d", ret);

    platformDisableIrqCallback();

    st25r3916SetRegisterBits(
        ST25R3916_REG_OP_CONTROL,
        ST25R3916_REG_OP_CONTROL_en | ST25R3916_REG_OP_CONTROL_rx_en |
            ST25R3916_REG_OP_CONTROL_en_fd_auto_efd);
    st25r3916WriteRegister(ST25R3916_REG_IO_CONF1, 0x00);

    furi_hal_nfc_dump_regs();

    // Try timer + DMA
    // Init GPIO data
    uint16_t reg = gpio_spi_r_mosi.port->ODR;
    one = reg | gpio_spi_r_mosi.pin;
    zero = reg & ~(gpio_spi_r_mosi.pin);
    // uint32_t one = gpio_spi_r_mosi.pin;
    // uint32_t zero = (uint32_t)gpio_spi_r_mosi.pin << 16;

    i = 0;
    // SoF
    add_one(&buff[i], &i);
    // Bytes
    for(size_t j = 0; j < 20; j++) {
        add_byte(buff, &i, j, true);
    }

    // finish with zero
    buff[i] = zero;
    i++;
    FURI_LOG_I(TAG, "One byte 16_t len: %d", i);
    // for(size_t i = 0; i < BUFF_SIZE; i++) {
    //     if(i % 2) {
    //         buff[i] = one;
    //     } else {
    //         buff[i] = zero;
    //     }
    // }
    // TIM
    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM2);
    furi_hal_delay_us(100);
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM2);
    furi_hal_delay_us(100);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    // Configure timer
    LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetClockDivision(TIM2, LL_TIM_CLOCKDIVISION_DIV1);
    LL_TIM_SetPrescaler(TIM2, 0);
    LL_TIM_SetAutoReload(TIM2, 37);
    LL_TIM_SetCounter(TIM2, 0);
    LL_TIM_EnableUpdateEvent(TIM2);
    LL_TIM_EnableDMAReq_UPDATE(TIM2);

    // LL_TIM_EnableIT_UPDATE(TIM2);
    // NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    // NVIC_EnableIRQ(TIM2_IRQn);

    // DMA
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);

    // LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_TIM2_UP);
    // LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    // LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_VERYHIGH);
    // LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);
    // LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
    // LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
    // LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_WORD);
    // LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_WORD);
    // LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)buff);
    // LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)gpio_spi_r_mosi.port->BSRR);
    // LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, BUFF_SIZE);
    // // DMAMUX
    // LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_0, LL_DMAMUX_REQ_GENERATOR0);
    // LL_DMAMUX_SetRequestSignalID(DMAMUX1, LL_DMAMUX_REQ_GEN_0, LL_DMAMUX_REQ_GEN_EXTI_LINE7);
    // LL_DMAMUX_SetRequestGenPolarity(DMAMUX1, LL_DMAMUX_REQ_GEN_0, LL_DMAMUX_REQ_GEN_POL_RISING_FALLING);
    // LL_DMAMUX_SetGenRequestNb(DMAMUX1, LL_DMAMUX_REQ_GEN_0, 1);
    // // SYNC
    // LL_DMAMUX_SetSyncRequestNb(DMAMUX1, LL_DMAMUX_CHANNEL_0, 1);
    // LL_DMAMUX_SetSyncPolarity(DMAMUX1, LL_DMAMUX_CHANNEL_0, LL_DMAMUX_SYNC_POL_RISING_FALLING);
    // LL_DMAMUX_SetSyncID(DMAMUX1, LL_DMAMUX_CHANNEL_0, LL_DMAMUX_SYNC_EXTI_LINE7);
    // LL_DMAMUX_EnableSync(DMAMUX1, LL_DMAMUX_CHANNEL_0);

    // Init gpio dma
    LL_DMA_InitTypeDef dma_config = {0};
    dma_config.MemoryOrM2MDstAddress = (uint32_t)buff;
    dma_config.PeriphOrM2MSrcAddress = (uint32_t) & (gpio_spi_r_mosi.port->ODR);
    dma_config.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    dma_config.Mode = LL_DMA_MODE_NORMAL;
    dma_config.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    dma_config.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    dma_config.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
    dma_config.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
    dma_config.NbData = i;
    dma_config.PeriphRequest = LL_DMAMUX_REQ_TIM2_UP;
    dma_config.Priority = LL_DMA_PRIORITY_VERYHIGH;
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &dma_config);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, i);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

    // Init ARR dma
    dma_config.MemoryOrM2MDstAddress = (uint32_t)arr;
    dma_config.PeriphOrM2MSrcAddress = (uint32_t) & (TIM2->ARR);
    dma_config.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    dma_config.Mode = LL_DMA_MODE_CIRCULAR;
    dma_config.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    dma_config.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    dma_config.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
    dma_config.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
    dma_config.NbData = SIZEOF_ARRAY(arr);
    dma_config.PeriphRequest = LL_DMAMUX_REQ_TIM2_UP;
    dma_config.Priority = LL_DMA_PRIORITY_VERYHIGH;
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_2, &dma_config);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, SIZEOF_ARRAY(arr));
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

    // LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    // NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    // NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    st25r3916ExecuteCommand(ST25R3916_CMD_TRANSPARENT_MODE);
    osDelay(3);

    // Reconfigure gpio and start
    furi_hal_spi_bus_handle_deinit(&furi_hal_spi_bus_handle_nfc);
    furi_hal_gpio_init(&gpio_spi_r_mosi, GpioModeOutputPushPull, GpioPullNo, GpioSpeedVeryHigh);
    furi_hal_gpio_write(&gpio_spi_r_mosi, false);
    osDelay(10);
    LL_TIM_GenerateEvent_UPDATE(TIM2);
    LL_TIM_EnableCounter(TIM2);
    // Reconfigure SPI
    // furi_hal_spi_bus_handle_init(&furi_hal_spi_bus_handle_nfc_transparent);

    // // RX
    // furi_hal_gpio_init(&gpio_spi_r_miso, GpioModeInput, GpioPullUp, GpioSpeedVeryHigh);

    // // TX
    // furi_hal_gpio_init(&gpio_spi_r_mosi, GpioModeOutputPushPull, GpioPullNo, GpioSpeedVeryHigh);

    // uint16_t test_buff[16 * 20];
    // size_t i = 0;
    // // SoF
    // add_one(&test_buff[i]);
    // i += 16;
    // // 0xDE = 0b11011110
    // // 0
    // add_zero(&test_buff[i]);
    // i += 16;
    // // 1
    // add_one(&test_buff[i]);
    // i += 16;
    // // 1
    // add_one(&test_buff[i]);
    // i += 16;
    // // 1
    // add_one(&test_buff[i]);
    // i += 16;
    // // 1
    // add_one(&test_buff[i]);
    // i += 16;
    // // 0
    // add_zero(&test_buff[i]);
    // i += 16;
    // // 1
    // add_one(&test_buff[i]);
    // i += 16;
    // // 1
    // add_one(&test_buff[i]);
    // i += 16;
    // // Parity 1
    // add_one(&test_buff[i]);
    // i += 16;

    // furi_hal_spi_acquire(&furi_hal_spi_bus_handle_nfc_transparent);
    // for(size_t j = 0; j < 10000000; j++) {
    //     // furi_hal_gpio_write(&gpio_spi_r_mosi, false);
    //     // furi_hal_delay_us(10);
    //     // furi_hal_gpio_write(&gpio_spi_r_mosi, true);
    //     // furi_hal_delay_us(10);
    //     furi_hal_spi_bus_tx_16(&furi_hal_spi_bus_handle_nfc_transparent, test_buff, i, 100000);
    //     furi_hal_delay_ms(100);
    // }
    // furi_hal_spi_release(&furi_hal_spi_bus_handle_nfc_transparent);

    Popup* popup = nfc->popup;
    popup_set_header(
        popup,
        "Field is on\nDon't leave device\nin this mode for too long.",
        64,
        11,
        AlignCenter,
        AlignTop);
    view_dispatcher_switch_to_view(nfc->view_dispatcher, NfcViewPopup);

    notification_internal_message(nfc->notifications, &sequence_set_blue_255);
}

void DMA1_Channel1_IRQHandler(void) {
    if(LL_DMA_IsActiveFlag_TC1(DMA1)) {
        LL_DMA_ClearFlag_TC1(DMA1);
    }
}

void TIM2_IRQHandler(void) {
    static size_t i = 0;
    if(LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
        LL_TIM_ClearFlag_UPDATE(TIM2);
        if(i % 2) {
            gpio_spi_r_mosi.port->ODR |= gpio_spi_r_mosi.pin;
        } else {
            gpio_spi_r_mosi.port->ODR &= ~gpio_spi_r_mosi.pin;
        }
        i++;
        // furi_hal_gpio_write(&gpio_spi_r_mosi, (i++) % 2);
    }
}

bool nfc_scene_transparent_on_event(void* context, SceneManagerEvent event) {
    // static bool high = true;
    if(event.type == SceneManagerEventTypeTick) {
        // furi_hal_gpio_write(&gpio_spi_r_mosi, high);
        // high = !high;
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
        LL_DMA_ClearFlag_TC1(DMA1);
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)buff);
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, i);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    }
    return false;
}

void nfc_scene_transparent_on_exit(void* context) {
    Nfc* nfc = context;

    notification_internal_message(nfc->notifications, &sequence_reset_blue);
    popup_reset(nfc->popup);
}
