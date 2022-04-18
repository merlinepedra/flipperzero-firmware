#include "../nfc_i.h"
#include <dolphin/dolphin.h>

void nfc_scene_emulate_mifare_classic_on_enter(void* context) {
    Nfc* nfc = context;
    DOLPHIN_DEED(DolphinDeedNfcEmulate);

    // Setup view
    Popup* popup = nfc->popup;
    popup_set_icon(popup, 0, 3, &I_RFIDDolphinSend_97x61);
    popup_set_header(popup, "Emulating\nMf Classic", 56, 31, AlignLeft, AlignTop);

    // Setup and start worker
    view_dispatcher_switch_to_view(nfc->view_dispatcher, NfcViewPopup);
    nfc_worker_start(
        nfc->worker,
        NfcWorkerStateEmulateMifareClassic,
        &nfc->dev->dev_data,
        NULL,
        nfc);
}

bool nfc_scene_emulate_mifare_classic_on_event(void* context, SceneManagerEvent event) {
    Nfc* nfc = context;
    bool consumed = false;

    if(event.type == SceneManagerEventTypeTick) {
        notification_message(nfc->notifications, &sequence_blink_blue_10);
        consumed = true;
    }

    return consumed;
}

void nfc_scene_emulate_mifare_classic_on_exit(void* context) {
    Nfc* nfc = context;

    // Clear view
    popup_reset(nfc->popup);
}
