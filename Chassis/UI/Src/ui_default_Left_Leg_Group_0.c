//
// Created by RM UI Designer
//

#include "ui_default_Left_Leg_Group_0.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 0
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Left_Leg_Group_0;
ui_interface_line_t *ui_default_Left_Leg_Group_L5 = (ui_interface_line_t *)&(ui_default_Left_Leg_Group_0.data[0]);
ui_interface_line_t *ui_default_Left_Leg_Group_L4 = (ui_interface_line_t *)&(ui_default_Left_Leg_Group_0.data[1]);
ui_interface_round_t *ui_default_Left_Leg_Group_Joint2 = (ui_interface_round_t *)&(ui_default_Left_Leg_Group_0.data[2]);
ui_interface_round_t *ui_default_Left_Leg_Group_Joint1 = (ui_interface_round_t *)&(ui_default_Left_Leg_Group_0.data[3]);
ui_interface_line_t *ui_default_Left_Leg_Group_L3 = (ui_interface_line_t *)&(ui_default_Left_Leg_Group_0.data[4]);
ui_interface_line_t *ui_default_Left_Leg_Group_L1 = (ui_interface_line_t *)&(ui_default_Left_Leg_Group_0.data[5]);
ui_interface_line_t *ui_default_Left_Leg_Group_L2 = (ui_interface_line_t *)&(ui_default_Left_Leg_Group_0.data[6]);

void _ui_init_default_Left_Leg_Group_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Left_Leg_Group_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_Left_Leg_Group_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_Left_Leg_Group_0.data[i].figure_name[2] = i + START_ID;
        ui_default_Left_Leg_Group_0.data[i].operate_tpyel = 1;
    }
//    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
//        ui_default_Left_Leg_Group_0.data[i].operate_tpyel = 0;
//    }

    ui_default_Left_Leg_Group_L5->figure_tpye = 0;
    ui_default_Left_Leg_Group_L5->layer = 1;
    ui_default_Left_Leg_Group_L5->start_x = 674;
    ui_default_Left_Leg_Group_L5->start_y = 229;
    ui_default_Left_Leg_Group_L5->end_x = 740;
    ui_default_Left_Leg_Group_L5->end_y = 229;
    ui_default_Left_Leg_Group_L5->color = 3;
    ui_default_Left_Leg_Group_L5->width = 4;

    ui_default_Left_Leg_Group_L4->figure_tpye = 0;
    ui_default_Left_Leg_Group_L4->layer = 1;
    ui_default_Left_Leg_Group_L4->start_x = 604;
    ui_default_Left_Leg_Group_L4->start_y = 193;
    ui_default_Left_Leg_Group_L4->end_x = 674;
    ui_default_Left_Leg_Group_L4->end_y = 229;
    ui_default_Left_Leg_Group_L4->color = 6;
    ui_default_Left_Leg_Group_L4->width = 4;

    ui_default_Left_Leg_Group_Joint2->figure_tpye = 2;
    ui_default_Left_Leg_Group_Joint2->layer = 1;
    ui_default_Left_Leg_Group_Joint2->r = 7;
    ui_default_Left_Leg_Group_Joint2->start_x = 674;
    ui_default_Left_Leg_Group_Joint2->start_y = 229;
    ui_default_Left_Leg_Group_Joint2->color = 2;
    ui_default_Left_Leg_Group_Joint2->width = 3;

    ui_default_Left_Leg_Group_Joint1->figure_tpye = 2;
    ui_default_Left_Leg_Group_Joint1->layer = 1;
    ui_default_Left_Leg_Group_Joint1->r = 7;
    ui_default_Left_Leg_Group_Joint1->start_x = 740;
    ui_default_Left_Leg_Group_Joint1->start_y = 229;
    ui_default_Left_Leg_Group_Joint1->color = 2;
    ui_default_Left_Leg_Group_Joint1->width = 3;

    ui_default_Left_Leg_Group_L3->figure_tpye = 0;
    ui_default_Left_Leg_Group_L3->layer = 1;
    ui_default_Left_Leg_Group_L3->start_x = 604;
    ui_default_Left_Leg_Group_L3->start_y = 192;
    ui_default_Left_Leg_Group_L3->end_x = 706;
    ui_default_Left_Leg_Group_L3->end_y = 90;
    ui_default_Left_Leg_Group_L3->color = 1;
    ui_default_Left_Leg_Group_L3->width = 4;

    ui_default_Left_Leg_Group_L1->figure_tpye = 0;
    ui_default_Left_Leg_Group_L1->layer = 1;
    ui_default_Left_Leg_Group_L1->start_x = 740;
    ui_default_Left_Leg_Group_L1->start_y = 229;
    ui_default_Left_Leg_Group_L1->end_x = 811;
    ui_default_Left_Leg_Group_L1->end_y = 191;
    ui_default_Left_Leg_Group_L1->color = 6;
    ui_default_Left_Leg_Group_L1->width = 4;

    ui_default_Left_Leg_Group_L2->figure_tpye = 0;
    ui_default_Left_Leg_Group_L2->layer = 1;
    ui_default_Left_Leg_Group_L2->start_x = 706;
    ui_default_Left_Leg_Group_L2->start_y = 89;
    ui_default_Left_Leg_Group_L2->end_x = 812;
    ui_default_Left_Leg_Group_L2->end_y = 195;
    ui_default_Left_Leg_Group_L2->color = 1;
    ui_default_Left_Leg_Group_L2->width = 4;

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Left_Leg_Group_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Left_Leg_Group_0, sizeof(ui_default_Left_Leg_Group_0));
}

void _ui_update_default_Left_Leg_Group_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Left_Leg_Group_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Left_Leg_Group_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Left_Leg_Group_0, sizeof(ui_default_Left_Leg_Group_0));
}

void _ui_remove_default_Left_Leg_Group_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Left_Leg_Group_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Left_Leg_Group_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Left_Leg_Group_0, sizeof(ui_default_Left_Leg_Group_0));
}
