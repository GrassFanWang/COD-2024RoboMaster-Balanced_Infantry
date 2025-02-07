//
// Created by RM UI Designer
//

#include "ui_default_Supercap_Per_Group_0.h"

#define FRAME_ID 0
#define GROUP_ID 6
#define START_ID 0
#define OBJ_NUM 1
#define FRAME_OBJ_NUM 1

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Supercap_Per_Group_0;
ui_interface_number_t *ui_default_Supercap_Per_Group_Supercap = (ui_interface_number_t *)&(ui_default_Supercap_Per_Group_0.data[0]);

void _ui_init_default_Supercap_Per_Group_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Supercap_Per_Group_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_Supercap_Per_Group_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_Supercap_Per_Group_0.data[i].figure_name[2] = i + START_ID;
        ui_default_Supercap_Per_Group_0.data[i].operate_tpyel = 1;
    }
//    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
//        ui_default_Supercap_Per_Group_0.data[i].operate_tpyel = 0;
//    }

    ui_default_Supercap_Per_Group_Supercap->figure_tpye = 6;
    ui_default_Supercap_Per_Group_Supercap->layer = 0;
    ui_default_Supercap_Per_Group_Supercap->font_size = 40;
    ui_default_Supercap_Per_Group_Supercap->start_x = 1296;
    ui_default_Supercap_Per_Group_Supercap->start_y = 55;
    ui_default_Supercap_Per_Group_Supercap->color = 2;
    ui_default_Supercap_Per_Group_Supercap->number = 100;
    ui_default_Supercap_Per_Group_Supercap->width = 3;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Supercap_Per_Group_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Supercap_Per_Group_0, sizeof(ui_default_Supercap_Per_Group_0));
}

void _ui_update_default_Supercap_Per_Group_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Supercap_Per_Group_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Supercap_Per_Group_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Supercap_Per_Group_0, sizeof(ui_default_Supercap_Per_Group_0));
}

void _ui_remove_default_Supercap_Per_Group_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Supercap_Per_Group_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Supercap_Per_Group_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Supercap_Per_Group_0, sizeof(ui_default_Supercap_Per_Group_0));
}
