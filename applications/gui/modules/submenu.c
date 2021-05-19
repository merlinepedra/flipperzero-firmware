#include "submenu.h"
#include <m-array.h>
#include <furi.h>
#include <gui/elements.h>

struct SubmenuItem {
    const char* label;
    uint32_t index;
    SubmenuItemCallback callback;
    void* callback_context;
};

ARRAY_DEF(SubmenuItemArray, SubmenuItem, M_POD_OPLIST);

struct Submenu {
    View* view;
};

typedef struct {
    SubmenuItemArray_t items;
    uint8_t position;
    uint8_t window_position;
} SubmenuModel;

static void submenu_process_up(Submenu* submenu);
static void submenu_process_down(Submenu* submenu);
static void submenu_process_ok(Submenu* submenu);

static void submenu_view_draw_callback(Canvas* canvas, void* _model) {
    SubmenuModel* model = _model;

    const uint8_t item_height = 16;
    const uint8_t item_width = 123;

    canvas_clear(canvas);
    canvas_set_font(canvas, FontSecondary);

    uint8_t position = 0;
    SubmenuItemArray_it_t it;

    for(SubmenuItemArray_it(it, model->items); !SubmenuItemArray_end_p(it);
        SubmenuItemArray_next(it)) {
        uint8_t item_position = position - model->window_position;

        if(item_position < 4) {
            if(position == model->position) {
                canvas_set_color(canvas, ColorBlack);
                canvas_draw_box(
                    canvas, 0, (item_position * item_height) + 1, item_width, item_height - 2);
                canvas_set_color(canvas, ColorWhite);

                canvas_draw_dot(canvas, 0, (item_position * item_height) + 1);
                canvas_draw_dot(canvas, 0, (item_position * item_height) + item_height - 2);
                canvas_draw_dot(canvas, item_width - 1, (item_position * item_height) + 1);
                canvas_draw_dot(
                    canvas, item_width - 1, (item_position * item_height) + item_height - 2);
            } else {
                canvas_set_color(canvas, ColorBlack);
            }
            canvas_draw_str(
                canvas,
                6,
                (item_position * item_height) + item_height - 4,
                SubmenuItemArray_cref(it)->label);
        }

        position++;
    }

    elements_scrollbar(canvas, model->position, SubmenuItemArray_size(model->items));
}

static bool submenu_view_input_callback(InputEvent* event, void* context) {
    Submenu* submenu = context;
    furi_assert(submenu);
    bool consumed = false;

    if(event->type == InputTypeShort) {
        switch(event->key) {
        case InputKeyUp:
            consumed = true;
            submenu_process_up(submenu);
            break;
        case InputKeyDown:
            consumed = true;
            submenu_process_down(submenu);
            break;
        case InputKeyOk:
            consumed = true;
            submenu_process_ok(submenu);
            break;
        default:
            break;
        }
    }

    return consumed;
}

Submenu* submenu_alloc() {
    Submenu* submenu = furi_alloc(sizeof(Submenu));
    submenu->view = view_alloc();
    view_set_context(submenu->view, submenu);
    view_allocate_model(submenu->view, ViewModelTypeLocking, sizeof(SubmenuModel));
    view_set_draw_callback(submenu->view, submenu_view_draw_callback);
    view_set_input_callback(submenu->view, submenu_view_input_callback);

    with_view_model(
        submenu->view, (SubmenuModel * model) {
            SubmenuItemArray_init(model->items);
            model->position = 0;
            model->window_position = 0;
            return true;
        });

    return submenu;
}

void submenu_free(Submenu* submenu) {
    furi_assert(submenu);

    with_view_model(
        submenu->view, (SubmenuModel * model) {
            SubmenuItemArray_clear(model->items);
            return true;
        });
    view_free(submenu->view);
    free(submenu);
}

View* submenu_get_view(Submenu* submenu) {
    furi_assert(submenu);
    return submenu->view;
}

SubmenuItem* submenu_add_item(
    Submenu* submenu,
    const char* label,
    uint32_t index,
    SubmenuItemCallback callback,
    void* callback_context) {
    SubmenuItem* item = NULL;
    furi_assert(label);
    furi_assert(submenu);

    with_view_model(
        submenu->view, (SubmenuModel * model) {
            item = SubmenuItemArray_push_new(model->items);
            item->label = label;
            item->index = index;
            item->callback = callback;
            item->callback_context = callback_context;
            return true;
        });

    return item;
}

void submenu_clean(Submenu* submenu) {
    furi_assert(submenu);

    with_view_model(
        submenu->view, (SubmenuModel * model) {
            SubmenuItemArray_clean(model->items);
            model->position = 0;
            model->window_position = 0;
            return true;
        });
}

void submenu_process_up(Submenu* submenu) {
    with_view_model(
        submenu->view, (SubmenuModel * model) {
            if(model->position > 0) {
                model->position--;
                if((model->position - model->window_position) < 1 && model->window_position > 0) {
                    model->window_position--;
                }
            } else {
                model->position = SubmenuItemArray_size(model->items) - 1;
                if(model->position > 3) {
                    model->window_position = model->position - 3;
                }
            }
            return true;
        });
}

void submenu_process_down(Submenu* submenu) {
    with_view_model(
        submenu->view, (SubmenuModel * model) {
            if(model->position < (SubmenuItemArray_size(model->items) - 1)) {
                model->position++;
                if((model->position - model->window_position) > 2 &&
                   model->window_position < (SubmenuItemArray_size(model->items) - 4)) {
                    model->window_position++;
                }
            } else {
                model->position = 0;
                model->window_position = 0;
            }
            return true;
        });
}

void submenu_process_ok(Submenu* submenu) {
    SubmenuItem* item = NULL;

    with_view_model(
        submenu->view, (SubmenuModel * model) {
            if(model->position < (SubmenuItemArray_size(model->items))) {
                item = SubmenuItemArray_get(model->items, model->position);
            }
            return true;
        });

    if(item && item->callback) {
        item->callback(item->callback_context, item->index);
    }
}
