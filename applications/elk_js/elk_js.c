#include <furi.h>
#include "elk.h"
#include <notification/notification_messages.h>
#include <storage/storage.h>
#include <dialogs/dialogs.h>

static void notify(const NotificationSequence* sequence) {
    NotificationApp* notification = furi_record_open(RECORD_NOTIFICATION);
    notification_message(notification, sequence);
    furi_record_close(RECORD_NOTIFICATION);
}

static jsval_t js_global_delay(struct js* js, jsval_t* args, int nargs) {
    UNUSED(js);
    UNUSED(nargs);
    furi_delay_ms(js_getnum(args[0]));
    return js_mknum(0);
}

static jsval_t js_led_red(struct js* js, jsval_t* args, int nargs) {
    UNUSED(js);
    UNUSED(args);
    UNUSED(nargs);
    notify(&sequence_set_only_red_255);
    return js_mknum(0);
}

static jsval_t js_led_green(struct js* js, jsval_t* args, int nargs) {
    UNUSED(js);
    UNUSED(args);
    UNUSED(nargs);
    notify(&sequence_set_only_green_255);
    return js_mknum(0);
}

static jsval_t js_led_blue(struct js* js, jsval_t* args, int nargs) {
    UNUSED(js);
    UNUSED(args);
    UNUSED(nargs);
    notify(&sequence_set_only_blue_255);
    return js_mknum(0);
}

static bool js_do(const char* text) {
    bool result = false;
    const size_t memory_size = 16 * 1024;
    uint8_t* memory = (uint8_t*)malloc(memory_size);
    struct js* js = js_create(memory, memory_size);

    jsval_t global = js_glob(js);
    jsval_t led = js_mkobj(js);
    js_set(js, global, "delay", js_mkfun(js_global_delay));
    js_set(js, global, "led", led);
    js_set(js, led, "red", js_mkfun(js_led_red));
    js_set(js, led, "green", js_mkfun(js_led_green));
    js_set(js, led, "blue", js_mkfun(js_led_blue));

    jsval_t res = js_eval(js, text, ~0U);

    if(js_type(res) == JS_ERR) {
        FURI_LOG_E("JS:", "%s", js_str(js, res));
        result = false;
    } else {
        result = true;
    }

    free(memory);
    return result;
}

int32_t elk_js_app(void* arg) {
    UNUSED(arg);

    Storage* storage = furi_record_open(RECORD_STORAGE);
    DialogsApp* dialogs = furi_record_open(RECORD_DIALOGS);
    string_t name;
    string_init_set(name, EXT_PATH(""));
    File* file = storage_file_alloc(storage);
    char* data = NULL;

    do {
        if(!dialog_file_browser_show(dialogs, name, name, ".js", true, NULL, false)) break;
        if(!storage_file_open(file, string_get_cstr(name), FSAM_READ, FSOM_OPEN_EXISTING)) break;
        size_t size = storage_file_size(file);
        data = (char*)malloc(size + 1);
        if(storage_file_read(file, data, size) != size) break;
        data[size] = '\0';
        js_do(data);
    } while(false);

    if(data) free(data);
    string_clear(name);
    storage_file_free(file);
    return 0;
}