//
// Created by RM UI Designer
//

#include "ui_default_SuperCap_Group_1.h"

#define FRAME_ID 0
#define GROUP_ID 3
#define START_ID 0
#define OBJ_NUM 2
#define FRAME_OBJ_NUM 2

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_SuperCap_Group_1;
ui_interface_line_t *ui_default_SuperCap_Group_SuperCap_Energy = (ui_interface_line_t *)&(ui_default_SuperCap_Group_1.data[0]);
ui_interface_rect_t *ui_default_SuperCap_Group_Supercap_Rect = (ui_interface_rect_t *)&(ui_default_SuperCap_Group_1.data[1]);

void _ui_init_default_SuperCap_Group_1() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_SuperCap_Group_1.data[i].figure_name[0] = FRAME_ID;
        ui_default_SuperCap_Group_1.data[i].figure_name[1] = GROUP_ID;
        ui_default_SuperCap_Group_1.data[i].figure_name[2] = i + START_ID;
        ui_default_SuperCap_Group_1.data[i].operate_tpyel = 1;
    }


ui_default_SuperCap_Group_SuperCap_Energy->figure_tpye = 0;
    ui_default_SuperCap_Group_SuperCap_Energy->layer = 1;
    ui_default_SuperCap_Group_SuperCap_Energy->start_x = 654;
    ui_default_SuperCap_Group_SuperCap_Energy->start_y = 36;
    ui_default_SuperCap_Group_SuperCap_Energy->end_x = 1292;
    ui_default_SuperCap_Group_SuperCap_Energy->end_y = 36;
    ui_default_SuperCap_Group_SuperCap_Energy->color = 2;
    ui_default_SuperCap_Group_SuperCap_Energy->width = 45;

    ui_default_SuperCap_Group_Supercap_Rect->figure_tpye = 1;
    ui_default_SuperCap_Group_Supercap_Rect->layer = 1;
    ui_default_SuperCap_Group_Supercap_Rect->start_x = 654;
    ui_default_SuperCap_Group_Supercap_Rect->start_y = 11;
    ui_default_SuperCap_Group_Supercap_Rect->color = 6;
    ui_default_SuperCap_Group_Supercap_Rect->width = 7;
    ui_default_SuperCap_Group_Supercap_Rect->end_x = 1288;
    ui_default_SuperCap_Group_Supercap_Rect->end_y = 61;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_SuperCap_Group_1);
    SEND_MESSAGE((uint8_t *) &ui_default_SuperCap_Group_1, sizeof(ui_default_SuperCap_Group_1));
}

void _ui_update_default_SuperCap_Group_1() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_SuperCap_Group_1.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_SuperCap_Group_1);
    SEND_MESSAGE((uint8_t *) &ui_default_SuperCap_Group_1, sizeof(ui_default_SuperCap_Group_1));
}

void _ui_remove_default_SuperCap_Group_1() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_SuperCap_Group_1.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_SuperCap_Group_1);
    SEND_MESSAGE((uint8_t *) &ui_default_SuperCap_Group_1, sizeof(ui_default_SuperCap_Group_1));
}
