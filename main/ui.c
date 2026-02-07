#include "lvgl.h"
#include "ui.h"

void ui_init(void)
{
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello LVGL + ILI9488!");
    lv_obj_center(label);
}