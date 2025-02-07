//
// Created by RM UI Designer
//

#include "ui_default_Chassis_Mode_Group_0.h"

#define FRAME_ID 0
#define GROUP_ID 2
#define START_ID 0
#define OBJ_NUM 4
#define FRAME_OBJ_NUM 5

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Chassis_Mode_Group_0;
ui_interface_round_t *ui_default_Chassis_Mode_Group_CHASSIS_SPIN = (ui_interface_round_t *)&(ui_default_Chassis_Mode_Group_0.data[0]);
ui_interface_rect_t *ui_default_Chassis_Mode_Group_CHASSIS_FRONT = (ui_interface_rect_t *)&(ui_default_Chassis_Mode_Group_0.data[1]);
ui_interface_rect_t *ui_default_Chassis_Mode_Group_CHASSIS_SIDE = (ui_interface_rect_t *)&(ui_default_Chassis_Mode_Group_0.data[2]);
ui_interface_line_t *ui_default_Chassis_Mode_Group_Shoot = (ui_interface_line_t *)&(ui_default_Chassis_Mode_Group_0.data[3]);

void _ui_init_default_Chassis_Mode_Group_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Chassis_Mode_Group_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_Chassis_Mode_Group_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_Chassis_Mode_Group_0.data[i].figure_name[2] = i + START_ID;
        ui_default_Chassis_Mode_Group_0.data[i].operate_tpyel = 1;
    }
//    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
//        ui_default_Chassis_Mode_Group_0.data[i].operate_tpyel = 0;
//    }

    ui_default_Chassis_Mode_Group_CHASSIS_SPIN->figure_tpye = 2;
    ui_default_Chassis_Mode_Group_CHASSIS_SPIN->layer = 4;
    ui_default_Chassis_Mode_Group_CHASSIS_SPIN->r = 72;
    ui_default_Chassis_Mode_Group_CHASSIS_SPIN->start_x = 972;
    ui_default_Chassis_Mode_Group_CHASSIS_SPIN->start_y = 151;
    ui_default_Chassis_Mode_Group_CHASSIS_SPIN->color = 0;
    ui_default_Chassis_Mode_Group_CHASSIS_SPIN->width = 4;

    ui_default_Chassis_Mode_Group_CHASSIS_FRONT->figure_tpye = 1;
    ui_default_Chassis_Mode_Group_CHASSIS_FRONT->layer = 0;
    ui_default_Chassis_Mode_Group_CHASSIS_FRONT->start_x = 881;
    ui_default_Chassis_Mode_Group_CHASSIS_FRONT->start_y = 92;
    ui_default_Chassis_Mode_Group_CHASSIS_FRONT->color = 0;
    ui_default_Chassis_Mode_Group_CHASSIS_FRONT->width = 4;
    ui_default_Chassis_Mode_Group_CHASSIS_FRONT->end_x = 1061;
    ui_default_Chassis_Mode_Group_CHASSIS_FRONT->end_y = 211;

    ui_default_Chassis_Mode_Group_CHASSIS_SIDE->figure_tpye = 1;
    ui_default_Chassis_Mode_Group_CHASSIS_SIDE->layer = 0;
    ui_default_Chassis_Mode_Group_CHASSIS_SIDE->start_x = 912;
    ui_default_Chassis_Mode_Group_CHASSIS_SIDE->start_y = 68;
    ui_default_Chassis_Mode_Group_CHASSIS_SIDE->color = 0;
    ui_default_Chassis_Mode_Group_CHASSIS_SIDE->width = 4;
    ui_default_Chassis_Mode_Group_CHASSIS_SIDE->end_x = 1030;
    ui_default_Chassis_Mode_Group_CHASSIS_SIDE->end_y = 241;

    ui_default_Chassis_Mode_Group_Shoot->figure_tpye = 0;
    ui_default_Chassis_Mode_Group_Shoot->layer = 1;
    ui_default_Chassis_Mode_Group_Shoot->start_x = 972;
    ui_default_Chassis_Mode_Group_Shoot->start_y = 151;
    ui_default_Chassis_Mode_Group_Shoot->end_x = 972;
    ui_default_Chassis_Mode_Group_Shoot->end_y = 244;
    ui_default_Chassis_Mode_Group_Shoot->color = 2;
    ui_default_Chassis_Mode_Group_Shoot->width = 20;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Chassis_Mode_Group_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Chassis_Mode_Group_0, sizeof(ui_default_Chassis_Mode_Group_0));
}

void _ui_update_default_Chassis_Mode_Group_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Chassis_Mode_Group_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Chassis_Mode_Group_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Chassis_Mode_Group_0, sizeof(ui_default_Chassis_Mode_Group_0));
}

void _ui_remove_default_Chassis_Mode_Group_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Chassis_Mode_Group_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Chassis_Mode_Group_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Chassis_Mode_Group_0, sizeof(ui_default_Chassis_Mode_Group_0));
}
