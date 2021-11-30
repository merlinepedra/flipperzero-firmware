#include <stdint.h>
#include "../core/core.h"

namespace UsbMidi {
enum CodeIndex {
    Misc = 0x0, /**< Reserved,  MIDI Size: 1, 2, 3 */
    CableEvent = 0x1, /**< Reserved, MIDI Size: 1, 2, 3 */
    SysEx2Byte = 0x2, /**< MIDI Size: 2 */
    SysEx3Byte = 0x3, /**< MIDI Size: 3 */
    SysExStart = 0x4, /**< MIDI Size: 3 */
    Common1Byte = 0x5, /**< MIDI Size: 1 */
    SysExEnd1Byte = 0x5, /**< MIDI Size: 1 */
    SysExEnd2Byte = 0x6, /**< MIDI Size: 2 */
    SysExEnd3Byte = 0x7, /**< MIDI Size: 3 */
    NoteOff = 0x8, /**< MIDI Size: 3 */
    NoteOn = 0x9, /**< MIDI Size: 3 */
    PolyKeyPress = 0xA, /**< MIDI Size: 3 */
    ControlChange = 0xB, /**< MIDI Size: 3 */
    ProgramChange = 0xC, /**< MIDI Size: 2 */
    ChannelPressure = 0xD, /**< MIDI Size: 2 */
    PitchBendChange = 0xE, /**< MIDI Size: 3 */
    SingleByte = 0xF, /**< MIDI Size: 1 */
};

uint8_t extract_cable_number(uint8_t data) {
    return (data >> 4) & 0b00001111;
}

CodeIndex extract_code_index(uint8_t data) {
    return static_cast<CodeIndex>(data & 0b00001111);
}

#define RETURN_TYPE_NAME(A) \
    case A:                 \
        return #A;          \
        break;

const char* code_index_name(CodeIndex code_index) {
    switch(code_index) {
        RETURN_TYPE_NAME(Misc);
        RETURN_TYPE_NAME(CableEvent);
        RETURN_TYPE_NAME(SysEx2Byte);
        RETURN_TYPE_NAME(SysEx3Byte);
        RETURN_TYPE_NAME(SysExStart);
        RETURN_TYPE_NAME(Common1Byte);
        /* RETURN_TYPE_NAME(SysExEnd1Byte); */
        RETURN_TYPE_NAME(SysExEnd2Byte);
        RETURN_TYPE_NAME(SysExEnd3Byte);
        RETURN_TYPE_NAME(NoteOff);
        RETURN_TYPE_NAME(NoteOn);
        RETURN_TYPE_NAME(PolyKeyPress);
        RETURN_TYPE_NAME(ControlChange);
        RETURN_TYPE_NAME(ProgramChange);
        RETURN_TYPE_NAME(ChannelPressure);
        RETURN_TYPE_NAME(PitchBendChange);
        RETURN_TYPE_NAME(SingleByte);
    default:
        return "";
        break;
    }
}

uint8_t valid_message_data_size(CodeIndex code_index) {
    uint8_t data_size = 0;
    switch(code_index) {
    case Common1Byte:
    /* case SysExEnd1Byte: */
    case SingleByte:
        data_size = 1;
        break;
    case SysEx2Byte:
    case SysExEnd2Byte:
    case ProgramChange:
    case ChannelPressure:
        data_size = 2;
        break;
    case SysEx3Byte:
    case SysExStart:
    case SysExEnd3Byte:
    case NoteOff:
    case NoteOn:
    case PolyKeyPress:
    case ControlChange:
    case PitchBendChange:
        data_size = 3;
        break;
    default:
        break;
    }

    return data_size;
}

struct Message {
    uint8_t cable_number;
    CodeIndex code_index;
    uint8_t midi[3];

    Message(uint8_t* data, uint32_t length);
};

Message::Message(uint8_t* data, uint32_t length) {
    if(length != 4) core_crash("Invalid MIDI-USB protocol");

    cable_number = extract_cable_number(data[0]);
    code_index = extract_code_index(data[0]);
    midi[0] = data[1];
    midi[1] = data[2];
    midi[2] = data[3];
}

}