/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : referee_info.c
  * @brief          : referee interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REFEREE_INFO_H
#define REFEREE_INFO_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* Exported defines ----------------------------------------------------------*/

#define REFEREE_RXFRAME_LENGTH  100

/**
 * @brief Referee Communication protocol format
 */
#define FrameHeader_Length    5U   /*!< the length of frame header */
#define CMDID_Length          2U   /*!< the length of CMD ID */
#define CRC16_Length          2U   /*!< the length of CRC ID */

/**
 * @brief Cmd id
 */
// #define GAME_STATUS_ID                    0x0001U  /*!< game status data */
// #define GAME_RESULT_ID                    0x0002U  /*!< game result data */
// #define GAME_ROBOTHP_ID                   0x0003U  /*!< robot HP data */

// #define EVENE_DATA_ID                     0x0101U  /*!< site event data */
// #define SUPPLY_ACTION_ID                  0x0102U  /*!< supply station action data */

//#define REFEREE_WARNING_ID                0x0104U  /*!< referee warning data */
//#define DART_INFO_ID                     0x0105U  /*!< dart shoot data */

#define ROBOT_STATUS_ID                   0x0201U  /*!< robot status data */

#define POWER_HEAT_ID                     0x0202U  /*!< real power heat data */

//#define ROBOT_POSITION_ID                 0x0203U  /*!< robot position data */
// #define ROBOT_BUFF_ID                     0x0204U  /*!< robot buff data */
// #define AIR_SUPPORT_ID                  0x0205U  /*!< aerial robot energy data */
// #define ROBOT_HURT_ID                     0x0206U  /*!< robot hurt data */
#define SHOOT_DATA_ID                       0x0207U  /*!< real robo t shoot data */
//#define PROJECTILE_ALLOWANCE_ID             0x0208U  /*!< bullet remain data */
//                
//#define RFID_STATUS_ID                    0x0209U  /*!< RFID status data */
//#define DART_CLIENT_CMD_ID                0x020AU  /*!< DART Client cmd data */
// #define GROUND_ROBOT_POSITION_ID                0x020BU  /*!< ground robot position */

//#define RADAR_MARAKING_ID                  0x020CU     /*!< Radar marking progress*/
//#define SENTRY_INFO_ID          0X020DU    /*!< SENTRY make autonomous decisions*/
//#define RADAR_INFO_ID             0X020EU   /*!< RADAR make autonomous decisions*/






#define INTERACTIVE_DATA_ID               0x0301U  /*!< robot interactive data */
#define CUSTOM_CONTROLLER_ID              0x0302U  /*!< custom controller data */
#define MINIMAP_INTERACTIVE_ID            0x0303U  /*!< mini map interactive data */
#define KEYMOUSE_INFO_ID                  0x0304U  /*!< key mouse data according the image transmission */
#define MINIMAP_RECV_ID                   0x0305U  /*!< mini map receive data */
#define CUSTOM_CONTROLLER_INTERACTIVE_ID  0x0306U  /*!< mini map receive data */
#define MAP_SENTRY_DATA_ID                0x0307U  /*!< mini map sentry path */
#define MAP_ROBOT_DATA_ID                 0x0308U  /*!< mini map robot  path */
/**
 * @brief Robot id
 */
#define ROBOT_RED_HERO_ID                 0x0001U
#define ROBOT_RED_ENGINEER_ID             0x0002U
#define ROBOT_RED_3_INFANTEY_ID           0x0003U
#define ROBOT_RED_4_INFANTEY_ID           0x0004U
#define ROBOT_RED_5_INFANTEY_ID           0x0005U
#define ROBOT_RED_AERIAL_INFANTEY_ID      0x0006U
#define ROBOT_RED_SENTEY_INFANTEY_ID      0x0007U
#define ROBOT_RED_DART_INFANTEY_ID        0x0008U
#define ROBOT_RED_RADAR_INFANTEY_ID       0x0009U
#define ROBOT_RED_OUTPOST_INFANTEY_ID     0x0010U
#define ROBOT_RED_BASE_INFANTEY_ID        0x0011U

#define ROBOT_BLUE_HERO_ID                0x0101U
#define ROBOT_BLUE_ENGINEER_ID            0x0102U
#define ROBOT_BLUE_3_INFANTEY_ID          0x0103U
#define ROBOT_BLUE_4_INFANTEY_ID          0x0104U
#define ROBOT_BLUE_5_INFANTEY_ID          0x0105U
#define ROBOT_BLUE_AERIAL_INFANTEY_ID     0x0106U
#define ROBOT_BLUE_SENTEY_INFANTEY_ID     0x0107U
#define ROBOT_BLUE_DART_INFANTEY_ID       0x0108U
#define ROBOT_BLUE_RADAR_INFANTEY_ID      0x0109U
#define ROBOT_BLUE_OUTPOST_INFANTEY_ID    0x0110U
#define ROBOT_BLUE_BASE_INFANTEY_ID       0x0111U

/**
 * @brief client id
 */
#define CLIENT_RED_HERO_ID                0x0101U
#define CLIENT_RED_ENGINEER_ID            0x0102U
#define CLIENT_RED_3_INFANTEY_ID          0x0103U
#define CLIENT_RED_4_INFANTEY_ID          0x0104U
#define CLIENT_RED_5_INFANTEY_ID          0x0105U
#define CLIENT_RED_AERIAL_INFANTEY_ID     0x0106U

#define CLIENT_BLUE_HERO_ID               0x0165U
#define CLIENT_BLUE_ENGINEER_ID           0x0166U
#define CLIENT_BLUE_3_INFANTEY_ID         0x0167U
#define CLIENT_BLUE_4_INFANTEY_ID         0x0168U
#define CLIENT_BLUE_5_INFANTEY_ID         0x0169U
#define CLIENT_BLUE_AERIAL_INFANTEY_ID    0x016AU

/* Exported types ------------------------------------------------------------*/
/* cancel byte alignment */
#pragma  pack(1)

/**
 * @brief typedef structure that contains the information of frame header
 */
typedef struct 
{
  uint8_t  SOF;           /*!< Data frame start byte, fixed value is 0xA5 */
  uint16_t data_length;   /*!< the length of data in the data frame */
  uint8_t  seq;           /*!< package serial number */
  uint8_t  CRC8;          /*!< Frame header CRC8 checksum */
}FrameHeader_TypeDef;

// /**
//  * @brief typedef structure that contains the information of Referee
//  */
// typedef struct
// {
//   FrameHeader_TypeDef FrameHeader;  /*!< frame header of referee received data */
//   uint16_t Cmd_id;                  /*!< cmd id  */
//   uint8_t  *Data;                   /*!< pointer to the receive data */
//   uint16_t FrameTail;               /*!< crc16 checksum , Whole package judgement */
// }Referee_Info_TypeDef;

/**
 * @brief typedef structure that contains the information of game status, id: 0x0001U
 */
typedef struct          
{
  /**
   * @brief the type of game,
            1:RMUC,
            2:RMUT,
            3:RMUA,
            4:RMUL,3v3,
            5:RMUL,1v1,
   */
	uint8_t game_type : 4;	        
	uint8_t game_progress : 4;	    /*!< the progress of game */
	uint16_t stage_remain_time;	    /*!< remain time of real progress */
  uint64_t SyncTimeStamp;
  // uint64_t SyncTimeStamp;         /*!< unix time */
} game_status_t;

/**
 * @brief typedef structure that contains the information of game result, id: 0x0002U
 */
typedef struct
{
  /**
   * @brief the result of game
            0:draw
            1:Red wins
            2:Blue wins
   */
 uint8_t winner;
} game_result_t;

/**
 * @brief typedef structure that contains the information of robot HP data, id: 0x0003U
 */
typedef struct
{
  uint16_t red_1_robot_HP;   /*!< Red Hero HP */
  uint16_t red_2_robot_HP;   /*!< Red Engineer HP */
  uint16_t red_3_robot_HP;   /*!< Red 3 Infantry HP */
  uint16_t red_4_robot_HP;   /*!< Red 4 Infantry HP */
  uint16_t red_5_robot_HP;   /*!< Red 5 Infantry HP */
  uint16_t red_7_robot_HP;   /*!< Red Sentry HP */
  uint16_t red_outpost_HP;   /*!< Red Outpost HP */
  uint16_t red_base_HP;      /*!< Red Base HP */

  uint16_t blue_1_robot_HP;   /*!< Blue Hero HP */
  uint16_t blue_2_robot_HP;   /*!< Blue Engineer HP */
  uint16_t blue_3_robot_HP;   /*!< Blue 3 Infantry HP */
  uint16_t blue_4_robot_HP;   /*!< Blue 4 Infantry HP */
  uint16_t blue_5_robot_HP;   /*!< Blue 5 Infantry HP */
  uint16_t blue_7_robot_HP;   /*!< Blue Sentry HP */
  uint16_t blue_outpost_HP;   /*!< Blue Outpost HP */
  uint16_t blue_base_HP;      /*!< Blue Base HP */
} game_robot_HP_t;

/**
 * @brief typedef structure that contains the information of site event data, id: 0x0101U
 */
typedef union
{
    /**
     * @brief the event of site
              bit 0:  status of supply station out recovery buff point, 1 is occupied
              bit 1:  status of supply station in recovery buff point, 1 is occupied
              bit 2:  status of supply Center gain point, 1 is occupied （Just RMUL）
              bit 3:  status of energy buff attack point, 1 is occupied
              bit 4:  status of little energy buff, 1 is activating
              bit 5:  status of large energy buff, 1 is activating
              bit 6-7:  status of Ring Heights land, 1 is our_side_occupied / 2  is opposite_side_occupied
              bit 8-9: status of R3 Trapezoidal Heights land,  1 is our side occupied / 2  is opposite side occupied
              bit 10-11  status of R4 Trapezoidal Heights land,  1 is our side occupied / 2  is opposite side occupied
              bit 12-18 status of Base virtual shield value Percentage of remaining  
              bit 19-27: The time when the dart last hit one's own outpost or base
              bit 28-29: The last time a dart hits a specific target at one's own outpost or base, 1 is outpost 2 is base fixed target, 3 is random target
              bit 30-31:  status of Center gain point , 0 is no occupied, 1 is our side occupied 2 is opposite side occupied （Just RMUL）
                        
    */
    uint32_t  event_data;

    uint32_t Out_recovery_point : 1;
    uint32_t In_recovery_point : 1;
    uint32_t recovery_point_RMUL : 1;
    uint32_t Energy_buff_attack_point : 1;
    uint32_t Little_energy_buff : 1;
    uint32_t Large_energy_buff : 1;
    uint32_t Ring_Heights_land : 2;
    uint32_t R3_Trapezoidal_Heights_land : 2;
    uint32_t R4_Trapezoidal_Heights_land : 2;
    uint32_t Base_shield_remaining_percentage: 7;
    uint32_t Dart_last_hit_time : 9;
    uint32_t Dart_last_hit_Target : 2;
    uint32_t Center_gain_point_situation_RMUL : 2; 

} event_data_t;

/**
 * @brief typedef structure that contains the information of supply station activation, id: 0x0102U
 */
typedef struct
{
  uint8_t reserved;   /*!< supply station id */
  /**
   * @brief supply robot id
   *             0: robot none
   *             1: red hero
   *             2: red engineer
   *         3/4/5: red infantry
   *           101: blue hero
   *           102: blue engineer
   *   103/104/105: blue infantry
   */
  uint8_t supply_robot_id;
  /**
   * @brief supply projectile status
   *        0: closed
   *        1: preparing
   *        2: open
   */
  uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;  /*!< the number of supply */
} supply_projectile_action_t;

/**
 * @brief typedef structure that contains the information of referee warning, id: 0x0104U
 */
typedef struct
{
   uint8_t level;
  /**
   * @brief warning level
   *        1: yellow card
   *        2: red card
   *        3: negative
   */
 
  uint8_t offending_robot_id;
    /**
   * @brief warning robot id
   *             0: robot none
   *             1: red hero
   *             2: red engineer
   *         3/4/5: red infantry
   *           101: blue hero
   *           102: blue engineer
   *   103/104/105: blue infantry
   */
  uint8_t count; /*!<The number of violations of the corresponding penalty level for the last violation robot that was penalized on our side.*/
} referee_warning_t;
/**
 * @brief typedef structure that contains the information of dart, id: 0x0105U
 */
typedef  struct
{
 
 uint8_t dart_remaining_time;
 
 uint16_t dart_info;
  /**
   * @brief dart_info
   *       bit 0-1 The latest target hit by our own dart.
   *       bit 2-4 The  hit count of the opponent's recently hit targets.
   *       bit 5-6 The selected target of the dart at this time, 1 is outpost 2 is base fixed target, 3 is random target;
   *       bit 7-15 reserved
   */
 uint16_t  Lastest_target_hit : 2;
 uint16_t  Hit_count : 3;
 uint16_t  Selected_target : 2 ; 
 uint16_t  reserved : 9 ;
}dart_info_t;



/**
 * @brief typedef structure that contains the information of robot status, id: 0x0201U
 */
typedef struct
{
  /**
   * @brief robot id
   *             0: robot none
   *             1: red hero
   *             2: red engineer
   *         3/4/5: red infantry
   *             6: red aerial
   *             7: red sentry
   *             8: red dart
   *             9: red radar station
   *           101: blue hero
   *           102: blue engineer
   *   103/104/105: blue infantry
   *           106: blue aerial
   *           107: blue sentry
   *           108: blue dart
   *           109: blue radar station
   */
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t current_HP;
  uint16_t maximum_HP;

  uint16_t shooter_barrel_cooling_value;
  uint16_t shooter_barrel_heat_limit;
  uint16_t chassis_power_limit;

  uint8_t mains_power_gimbal_output : 1;
  uint8_t mains_power_chassis_output : 1;
  uint8_t mains_power_shooter_output : 1;
} robot_status_t;

/**
 * @brief typedef structure that contains the information of power heat data, id: 0x0202U
 */
typedef struct
{
  uint16_t chassis_voltage;
  uint16_t chassis_current;
  float chassis_power;
  uint16_t buffer_energy;
  uint16_t shooter_17mm_1_barrel_heat;
  uint16_t shooter_17mm_2_barrel_heat;
  uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

/**
 * @brief typedef structure that contains the information of robot position data, id: 0x0203U
 */
typedef struct
{
  float x;    /*!< position x coordinate, unit: m */
  float y;    /*!< position y coordinate, unit: m */
  float  angle;  /*!< Position muzzle, unit: degrees */
} robot_pos_t;

/**
 * @brief typedef structure that contains the information of robot buff data, id: 0x0204U
 */
typedef union
{
  uint8_t recovery_buff;
  uint8_t cooling_buff;
  uint8_t defence_buff;
  uint8_t vulnerability_buff;
  uint16_t attack_buff;
}buff_t;

/**
 * @brief typedef structure that contains the information of aerial robot energy, id: 0x0205U
 */
typedef  struct
{
 uint8_t airforce_status;
 uint8_t time_remain;
}air_support_data_t;

/**
 * @brief typedef structure that contains the information of robot hurt, id: 0x0206U
 */
typedef struct
{
 uint8_t armor_id : 4; /*!< hurt armor id */
  /**
   * @brief hurt type
   *        0: armor hurt
   *        1: module offline
   *        2: over fire rate
   *        3: over fire heat
   *        4: over chassis power
   *        5: armor bump
   */
 uint8_t HP_deduction_reason : 4;
} hurt_data_t;

/**
 * @brief typedef structure that contains the information of real shoot data, id: 0x0207U
 */
typedef  struct
{
 uint8_t bullet_type;
 uint8_t shooter_number;
 uint8_t launching_frequency;
 float initial_speed;
}shoot_data_t;

/**
 * @brief typedef structure that contains the information of bullet remaining number, id: 0x0208U
 */
typedef  struct
{
 uint16_t projectile_allowance_17mm;
 uint16_t projectile_allowance_42mm;
 uint16_t remaining_gold_coin;
}projectile_allowance_t;;

/**
 * @brief typedef structure that contains the information of RFID status, id: 0x0209U
 */
typedef union
{
  /**
   * @brief RFID status
            bit 0:  status of RFID in base buff point
            bit 1:  status of RFID in heights buff point
            bit 2:  status of RFID in energy buff hit point
            bit 3:  status of RFID in flying slope buff point
            bit 4:  status of RFID in outpost buff point
            bit 6:  status of RFID in recovery buff point
            bit 7:  status of RFID in engineer resurrection card
            bit 8-31: reserved 
   */
 uint32_t rfid_status;
 uint32_t BaseBuff_status : 1;
 uint32_t Our_Ring_HeightBuff_status : 1;
 uint32_t Opposite_Ring_HeightBuff_status : 1;
 uint32_t Our_R3_Trapezoidal_HeightsBuff_status : 1;
 uint32_t Opposite_R3_Trapezoidal_HeightsBuff_status : 1;
 uint32_t Our_R4_Trapezoidal_HeightsBuff_status : 1;
 uint32_t Opposite_R4_Trapezoidal_HeightsBuff_status : 1;
 uint32_t EnergyhitBuff_status : 1;
 uint32_t Our_Front_FlyslopebBuff_status : 1;
 uint32_t Our_Before_FlyslopebBuff_status : 1;
 uint32_t Opposite_Front_FlyslopebBuff_status : 1;
 uint32_t Opposite_Before_FlyslopebBuff_status : 1;
 uint32_t OutpostBuff_status : 1;
 uint32_t HPrecoveryBuff_status : 1;
 uint32_t Our_Sentry_Patrol_status : 1;
 uint32_t Opposite_Sentry_Patrol_status : 1;
 uint32_t Our_Large_Resource_IslandBuff_status : 1 ;
 uint32_t Opposite_Large_Resource_IslandBuff_status : 1;
 uint32_t Our_Exchange_status : 1;
 uint32_t Center_Buff_point_status : 1;
 uint32_t reserved : 12;
} rfid_status_t;

/**
 * @brief typedef structure that contains the information of dart client data, id: 0x020AU
 */
typedef  struct
{
 uint8_t dart_launch_opening_status;
 uint8_t reserved;
 uint16_t target_change_time;
 uint16_t latest_launch_cmd_time;
}dart_client_cmd_t;

/**
 * @brief typedef structure that contains the information of robot position in mimi map, id: 0x020BU
 */
typedef struct
{
  float hero_x;
  float hero_y;
  float engineer_x;
  float engineer_y;
  float standard_3_x;
  float standard_3_y;
  float standard_4_x;
  float standard_4_y;
  float standard_5_x;
  float standard_5_y;
}ground_robot_position_t;

/**
 * @brief typedef structure that contains the information of robot mark, id: 0x020CU
 */
typedef struct
{
  uint8_t mark_hero_progress;
  uint8_t mark_engineer_progress;
  uint8_t mark_standard_3_progress;
  uint8_t mark_standard_4_progress;
  uint8_t mark_standard_5_progress;
  uint8_t mark_sentry_progress;
}radar_mark_data_t;
typedef  struct
{
 uint32_t sentry_info;
 uint32_t Sentry_Exchange_bullet : 11;
 uint32_t Sentry_Remote_Exchange_bullet_count : 4;
 uint32_t Sentry_Remote_Exchange_HP_count : 4;
 uint32_t reserved : 13;
} sentry_info_t;
typedef  struct
{
 uint8_t radar_info;
 uint8_t radar_double_damage_chance : 2;
 uint8_t opposite_radar_double_damage : 1;
 uint8_t reserved :5 ;  
} radar_info_t;
/**
 * @brief typedef structure that contains the information of custom controller interactive, id: 0x0301U
 */
typedef struct{ 
 uint16_t data_cmd_id;
 uint16_t sender_id;
 uint16_t receiver_id;
 uint8_t user_data[113];
}robot_interaction_data_t;
/**
 * @brief typedef structure that contains the information of custom controller interactive, id: 0x0302U
 */
typedef struct 
{
  uint8_t data[30];
} custom_robot_data_t;

/**
 * @brief typedef structure that contains the information of client transmit data, id: 0x0303U
 */
typedef struct
{
  /**
   * @brief target position coordinate, is 0 when transmit target robot id
   */
  float target_position_x;
  float target_position_y;
  float target_position_z;

  uint8_t commd_keyboard;
  uint16_t target_robot_ID;   /* is 0 when transmit position data */
} ext_robot_command_t;

/**
 * @brief typedef structure that contains the information of client receive data, id: 0x0305U
 */
typedef struct
{
  uint16_t target_robot_ID;
  float target_position_x;
  float target_position_y;
} ext_client_map_command_t;

/**
 * @brief typedef structure that contains the information of custom controller key mouse, id: 0x0306U
 */
typedef struct
{
 uint16_t key_value;
 uint16_t x_position:12;
 uint16_t mouse_left:4;
 uint16_t y_position:12;
 uint16_t mouse_right:4;
 uint16_t reserved;
}custom_client_data_t;

/**
 * @brief typedef structure that contains the information of sentry path, id: 0x0307U
 */
typedef struct
{
  /**
   * @brief  sentry status
   *         1: attack on target point
   *         2: defend on target point
   *         3: move to target point
   */
  uint8_t intention;
  uint16_t start_position_x;
  uint16_t start_position_y;
  int8_t delta_x[49];
  int8_t delta_y[49];
}map_sentry_data_t;

/**
 * @brief typedef structure that contains the information of Referee
 */
typedef struct 
{
  uint8_t index;
  uint16_t datalength;
  
#ifdef GAME_STATUS_ID
  game_status_t game_status;
#endif

#ifdef GAME_RESULT_ID
  game_result_t game_result;
#endif

#ifdef  GAME_ROBOTHP_ID
	game_robot_HP_t game_robot_HP;
#endif	
	
#ifdef EVENE_DATA_ID
  event_data_t event_data;
#endif

#ifdef SUPPLY_ACTION_ID   	
  supply_projectile_action_t  supply_projectile_action;
#endif

#ifdef REFEREE_WARNING_ID
    referee_warning_t referee_warning;
#endif

#ifdef DART_INFO_ID
  dart_info_t dart_info;
#endif

#ifdef ROBOT_STATUS_ID
  robot_status_t robot_status;
#endif

#ifdef POWER_HEAT_ID
  power_heat_data_t power_heat_data;
#endif

#ifdef ROBOT_POSITION_ID
  robot_pos_t  robot_pos;
#endif

#ifdef ROBOT_BUFF_ID
  buff_t  buff;
#endif

#ifdef AIR_SUPPORT_ID
 air_support_data_t air_support_data;
#endif
#ifdef ROBOT_HURT_ID
  hurt_data_t  hurt_data;
#endif

#ifdef SHOOT_DATA_ID
  shoot_data_t  shoot_data;
#endif

#ifdef PROJECTILE_ALLOWANCE_ID
  projectile_allowance_t projectile_allowance;
#endif
#ifdef  RFID_STATUS_ID
    rfid_status_t rfid_status;
#endif

#ifdef DART_CLIENT_CMD_ID
    dart_client_cmd_t  dart_client_cmd;
#endif

#ifdef GROUND_ROBOT_POSITION_ID
    ground_robot_position_t  ground_robot_position;
#endif 

#ifdef RADAR_MARAKING_ID
    radar_mark_data_t  radar_mark_data;
#endif

#ifdef SENTRY_INFO_ID
    sentry_info_t  sentry_info;
#endif

#ifdef RADAR_INFO_ID
    radar_info_t  radar_info;
#endif
}Referee_Info_TypeDef;

/* restore byte alignment */
#pragma  pack()

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief Referee_RxDMA MultiBuffer
 */
extern uint8_t REFEREE_MultiRx_Buf[2][100];

/**
 * @brief Referee structure variable
 */
extern Referee_Info_TypeDef Referee_Info;


/* Exported functions prototypes ---------------------------------------------*/
extern void Referee_Frame_Update(uint8_t *Buff);

#endif //REFEREE_INFO_H
