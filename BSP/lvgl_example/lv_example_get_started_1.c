#include "lvgl.h"

static void btn_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        static uint8_t cnt = 0;
        cnt++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "Button: %d", cnt);
    }
}

/**
 * Create a button with a label and react on click event.
 */
void lv_example_get_started_1(void)
{
    lv_obj_t * btn = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
    lv_obj_set_pos(btn, 0, 0);                            /*Set its position*/
    lv_obj_set_size(btn, 100, 40);                          /*Set its size*/
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    lv_obj_t * label = lv_label_create(btn);          /*Add a label to the button*/
    lv_label_set_text(label, "Button");                     /*Set the labels text*/
    lv_obj_center(label);

    lv_obj_t * btn1 = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
    lv_obj_set_pos(btn1, 20, 200);                            /*Set its position*/
    lv_obj_set_size(btn1, 100, 40);                          /*Set its size*/
    lv_obj_add_event_cb(btn1, btn_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    lv_obj_t * label1 = lv_label_create(btn1);          /*Add a label to the button*/
    lv_label_set_text(label1, "Button1");                     /*Set the labels text*/
    lv_obj_center(label1);

    lv_obj_t * btn2 = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
    lv_obj_set_pos(btn2, 120, 0);                            /*Set its position*/
    lv_obj_set_size(btn2, 100, 40);                          /*Set its size*/
    lv_obj_add_event_cb(btn2, btn_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    lv_obj_t * label2 = lv_label_create(btn2);          /*Add a label to the button*/
    lv_label_set_text(label2, "Button2");                     /*Set the labels text*/
    lv_obj_center(label2);
}

