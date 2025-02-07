//
// Created by RM UI Designer
//

#include "ui_default_Mode_Group_0.h"

#define FRAME_ID 0
#define GROUP_ID 4
#define START_ID 0
#define OBJ_NUM 6
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Mode_Group_0;
ui_interface_round_t *ui_default_Mode_Group_Shoot = (ui_interface_round_t *)&(ui_default_Mode_Group_0.data[0]);
ui_interface_round_t *ui_default_Mode_Group_Cover = (ui_interface_round_t *)&(ui_default_Mode_Group_0.data[1]);
ui_interface_round_t *ui_default_Mode_Group_Week = (ui_interface_round_t *)&(ui_default_Mode_Group_0.data[2]);
ui_interface_rect_t  *ui_default_Mode_Group_Vision = (ui_interface_rect_t *)&(ui_default_Mode_Group_0.data[3]);
ui_interface_line_t *ui_default_Mode_Group_Left_Body_Line = (ui_interface_line_t *)&(ui_default_Mode_Group_0.data[4]);
ui_interface_line_t *ui_default_Mode_Group_Right_Body_Line = (ui_interface_line_t *)&(ui_default_Mode_Group_0.data[5]);

void _ui_init_default_Mode_Group_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Mode_Group_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_Mode_Group_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_Mode_Group_0.data[i].figure_name[2] = i + START_ID;
        ui_default_Mode_Group_0.data[i].operate_tpyel = 1;
    }
//    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
//        ui_default_Mode_Group_0.data[i].operate_tpyel = 0;
//    }

    ui_default_Mode_Group_Shoot->figure_tpye = 2;
    ui_default_Mode_Group_Shoot->layer = 0;
    ui_default_Mode_Group_Shoot->r = 17;
    ui_default_Mode_Group_Shoot->start_x = 161;
    ui_default_Mode_Group_Shoot->start_y = 826;
    ui_default_Mode_Group_Shoot->color = 8;
    ui_default_Mode_Group_Shoot->width = 7;

    ui_default_Mode_Group_Cover->figure_tpye = 2;
    ui_default_Mode_Group_Cover->layer = 0;
    ui_default_Mode_Group_Cover->r = 17;
    ui_default_Mode_Group_Cover->start_x = 161;
    ui_default_Mode_Group_Cover->start_y = 769;
    ui_default_Mode_Group_Cover->color = 8;
    ui_default_Mode_Group_Cover->width = 7;

    ui_default_Mode_Group_Week->figure_tpye = 2;
    ui_default_Mode_Group_Week->layer = 0;
    ui_default_Mode_Group_Week->r = 17;
    ui_default_Mode_Group_Week->start_x = 161;
    ui_default_Mode_Group_Week->start_y = 714;
    ui_default_Mode_Group_Week->color = 8;
    ui_default_Mode_Group_Week->width = 7;

    ui_default_Mode_Group_Vision->figure_tpye = 1;
    ui_default_Mode_Group_Vision->layer = 0;
    ui_default_Mode_Group_Vision->start_x = 558;
    ui_default_Mode_Group_Vision->start_y = 267;
    ui_default_Mode_Group_Vision->color = 8;
    ui_default_Mode_Group_Vision->width = 5;
    ui_default_Mode_Group_Vision->end_x = 1359;
    ui_default_Mode_Group_Vision->end_y = 816;
		
		ui_default_Mode_Group_Left_Body_Line->figure_tpye = 0;
    ui_default_Mode_Group_Left_Body_Line->layer = 0;
    ui_default_Mode_Group_Left_Body_Line->start_x = 705;
    ui_default_Mode_Group_Left_Body_Line->start_y = 267;
    ui_default_Mode_Group_Left_Body_Line->end_x = 849;
    ui_default_Mode_Group_Left_Body_Line->end_y = 530;
    ui_default_Mode_Group_Left_Body_Line->color = 5;
    ui_default_Mode_Group_Left_Body_Line->width = 4;
		
		ui_default_Mode_Group_Right_Body_Line->figure_tpye = 0;
    ui_default_Mode_Group_Right_Body_Line->layer = 0;
    ui_default_Mode_Group_Right_Body_Line->start_x = 1075;
    ui_default_Mode_Group_Right_Body_Line->start_y = 530;
    ui_default_Mode_Group_Right_Body_Line->end_x = 1220;
    ui_default_Mode_Group_Right_Body_Line->end_y = 267;
    ui_default_Mode_Group_Right_Body_Line->color = 5;
    ui_default_Mode_Group_Right_Body_Line->width = 4;
    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Mode_Group_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Mode_Group_0, sizeof(ui_default_Mode_Group_0));
}

void _ui_update_default_Mode_Group_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Mode_Group_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Mode_Group_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Mode_Group_0, sizeof(ui_default_Mode_Group_0));
}

void _ui_remove_default_Mode_Group_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Mode_Group_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Mode_Group_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Mode_Group_0, sizeof(ui_default_Mode_Group_0));
}
