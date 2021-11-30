#include <furi.h>
#include <furi-hal.h>
#include <stream_buffer.h>
#include "midi/midi-parser.h"

using namespace Midi;

typedef struct {
    uint32_t a;
    StreamBufferHandle_t rx_stream;
} MusicMidiApp;

static void midi_on_irq_cb(UartIrqEvent ev, uint8_t data, void* context) {
    furi_assert(context);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    MusicMidiApp* app = static_cast<MusicMidiApp*>(context);
    (void)app;

    if(ev == UartIrqEventRXNE) {
        xStreamBufferSendFromISR(app->rx_stream, &data, 1, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static MusicMidiApp* music_midi_app_alloc() {
    MusicMidiApp* app = static_cast<MusicMidiApp*>(furi_alloc(sizeof(MusicMidiApp)));
    app->rx_stream = xStreamBufferCreate(2048, 1);

    // Enable uart listener
    furi_hal_uart_init(FuriHalUartIdLPUART1, 31250);
    furi_hal_uart_set_irq_cb(FuriHalUartIdLPUART1, midi_on_irq_cb, app);

    furi_hal_spi_bus_handle_init(&furi_hal_spi_config_external);

    return app;
}

static void music_midi_app_free(MusicMidiApp* app) {
    furi_assert(app);
    furi_hal_uart_deinit(FuriHalUartIdLPUART1);
    vStreamBufferDelete(app->rx_stream);

    furi_hal_spi_bus_handle_deinit(&furi_hal_spi_config_external);

    // Free rest
    free(app);
}

static float note_to_freq(float note) {
    float a = 440.0f; // frequency of A (coomon value is 440Hz)
    return (a / 32.0f) * pow(2, ((note - 9.0f) / 12.0f));
}

extern "C" int32_t music_midi_app(void* p) {
    MusicMidiApp* app = music_midi_app_alloc();
    MidiParser* parser = new MidiParser();

    // uint8_t dac_value1[] = {0xff, 0xff};
    // uint8_t dac_value2[] = {0xf0, 0xff};

    // while(1) {
    //     furi_hal_spi_acquire(&furi_hal_spi_config_external);
    //     furi_hal_spi_bus_tx(&furi_hal_spi_config_external, dac_value1, 2, osWaitForever);
    //     furi_hal_spi_release(&furi_hal_spi_config_external);
    //     delay(10);

    //     furi_hal_spi_acquire(&furi_hal_spi_config_external);
    //     furi_hal_spi_bus_tx(&furi_hal_spi_config_external, dac_value2, 2, osWaitForever);
    //     furi_hal_spi_release(&furi_hal_spi_config_external);
    //     delay(10);
    // }

    while(1) {
        uint8_t data;
        xStreamBufferReceive(app->rx_stream, &data, 1, osWaitForever);

        if(parser->parse(data)) {
            MidiEvent* event = parser->get_message();

            switch(event->type) {
            case MidiMessageType::NoteOn: {
                NoteOnEvent note_event = event->AsNoteOn();
                printf("Note ON, %02x %02x\r\n", note_event.note, note_event.velocity);
                hal_pwm_set(0.5, note_to_freq(note_event.note), &SPEAKER_TIM, SPEAKER_CH);
            }; break;
            case MidiMessageType::NoteOff: {
                NoteOffEvent note_event = event->AsNoteOff();
                printf("Note OFF, %02x %02x\r\n", note_event.note, note_event.velocity);
                hal_pwm_stop(&SPEAKER_TIM, SPEAKER_CH);
            }; break;
            default:
                break;
            }
        }
    }

    delete parser;
    music_midi_app_free(app);
    return 0;
}