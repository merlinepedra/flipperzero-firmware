#include "../nfc_i.h"

void nfc_scene_set_uid_byte_input_callback(void* context) {
    Nfc* nfc = context;

    view_dispatcher_send_custom_event(nfc->view_dispatcher, NfcCustomEventByteInputDone);
}

void nfc_scene_set_uid_on_enter(void* context) {
    Nfc* nfc = context;

    // Setup view
    ByteInput* byte_input = nfc->byte_input;
    byte_input_set_header_text(byte_input, "Enter uid in hex");
    nfc->dev_edit_data = nfc->dev->dev_data.nfc_data;
    byte_input_set_result_callback(
        byte_input,
        nfc_scene_set_uid_byte_input_callback,
        NULL,
        nfc,
        nfc->dev_edit_data.uid,
        nfc->dev_edit_data.uid_len);
    view_dispatcher_switch_to_view(nfc->view_dispatcher, NfcViewByteInput);
}

bool nfc_scene_set_uid_on_event(void* context, SceneManagerEvent event) {
    Nfc* nfc = (Nfc*)context;
    bool consumed = false;

    if(event.type == SceneManagerEventTypeCustom) {
        if(event.event == NfcCustomEventByteInputDone) {
            if(scene_manager_has_previous_scene(nfc->scene_manager, NfcSceneSavedMenu)) {
                nfc->dev->dev_data.nfc_data = nfc->dev_edit_data;
                // Create path by adding nfc/ to the dev_name or using the load path
                if(furi_string_end_with(nfc->dev->load_path, NFC_APP_EXTENSION)) {
                    if(nfc_device_save(nfc->dev, furi_string_get_cstr(nfc->dev->load_path))) {
                        scene_manager_next_scene(nfc->scene_manager, NfcSceneSaveSuccess);
                        consumed = true;
                    } else {
                        scene_manager_next_scene(nfc->scene_manager, NfcSceneSaveName);
                        consumed = true;
                    }
                } else {
                    char* path = malloc(strlen(nfc->dev->dev_name) + 5);
                    strcpy(path, ANY_PATH("nfc/"));
                    strcat(path, nfc->text_store);
                    strcat(path, NFC_APP_EXTENSION);
                    if(nfc_device_save(nfc->dev, path)) {
                        scene_manager_next_scene(nfc->scene_manager, NfcSceneSaveSuccess);
                        consumed = true;
                    } else {
                        scene_manager_next_scene(nfc->scene_manager, NfcSceneSaveName);
                        consumed = true;
                    }
                    free(path);
                }
            }
        }
    }
    return consumed;
}

void nfc_scene_set_uid_on_exit(void* context) {
    Nfc* nfc = context;

    // Clear view
    byte_input_set_result_callback(nfc->byte_input, NULL, NULL, NULL, NULL, 0);
    byte_input_set_header_text(nfc->byte_input, "");
}
