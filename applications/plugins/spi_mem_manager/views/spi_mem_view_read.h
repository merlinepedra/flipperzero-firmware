#pragma once
#include <gui/view.h>

typedef struct SPIMemReadView SPIMemReadView;
typedef void (*SPIMemViewReadCallback)(void* context);

View* spi_mem_view_read_get_view(SPIMemReadView* app);
SPIMemReadView* spi_mem_view_read_alloc();
void spi_mem_view_read_free(SPIMemReadView* app);
void spi_mem_view_read_set_callback(
    SPIMemReadView* app,
    SPIMemViewReadCallback callback,
    void* cb_ctx);
void spi_mem_view_read_set_progress(SPIMemReadView* app, float progress);
