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

/* Includes ------------------------------------------------------------------*/
#include "referee_info.h"
#include "crc.h"
#include "string.h"

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief Referee_RxDMA MultiBuffer
 */
uint8_t REFEREE_MultiRx_Buf[2][REFEREE_RXFRAME_LENGTH];

/**
 * @brief Referee structure variable
 */
Referee_Info_TypeDef Referee_Info;
/* Private function prototypes -----------------------------------------------*/
static uint32_t bit8TObit32(uint8_t change_info[4]);
static uint32_t bit8TObit64(uint8_t change_info[8]);
static float bit8TOfloat32(uint8_t change_info[4]);
//static uint8_t bit32TObit8(uint8_t index_need,uint32_t bit32);
static int16_t bit8TObit16(uint8_t change_info[2]);
//static uint8_t bit16TObit8(uint8_t index_need,int16_t bit16);
static void Referee_Info_Update(uint8_t *Buff,Referee_Info_TypeDef *referee);

void Referee_Frame_Update(uint8_t *Buff)
{
	Referee_Info.index = 0;
	Referee_Info.datalength = 0;
	
  while (Buff[Referee_Info.index] == 0xA5)
	{
    if(verify_CRC8_check_sum(&Buff[Referee_Info.index],FrameHeader_Length) == true)
    {
      Referee_Info.datalength = (uint16_t)(Buff[Referee_Info.index+2]<<8 | Buff[Referee_Info.index+1]) + FrameHeader_Length + CMDID_Length + CRC16_Length;
      if(verify_CRC16_check_sum(&Buff[Referee_Info.index],Referee_Info.datalength) == true)
      {
        Referee_Info_Update(Buff,&Referee_Info);
      }
    }else{
		 break;
		}
		Referee_Info.index += Referee_Info.datalength;
  }
}

static void Referee_Info_Update(uint8_t *Buff,Referee_Info_TypeDef *referee)
{
  switch (bit8TObit16(&Buff[referee->index + FrameHeader_Length]))
  {
#ifdef GAME_STATUS_ID 
    case GAME_STATUS_ID:
      referee->game_status.game_type         = Buff[referee->index + FrameHeader_Length + CMDID_Length] & 0x0F  ;
      referee->game_status.game_progress     = (Buff[referee->index + FrameHeader_Length + CMDID_Length] & 0xF0) >> 4  ;
      referee->game_status.stage_remain_time = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 1]);
		  // referee->game_status.SyncTimeStamp = bit8TObit64(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 3]);
    break;
#endif

#ifdef GAME_RESULT_ID
		case GAME_RESULT_ID:
		 referee->game_result.winner = Buff[referee->index + FrameHeader_Length + CMDID_Length];
		break;
#endif

	#ifdef GAME_ROBOTHP_ID
    case GAME_ROBOTHP_ID:
      referee->game_robot_HP.red_1_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length]);
      referee->game_robot_HP.red_2_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 2]);
      referee->game_robot_HP.red_3_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 4]);
      referee->game_robot_HP.red_4_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 6]);
      referee->game_robot_HP.red_5_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 8]);
      referee->game_robot_HP.red_7_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 10]);
      referee->game_robot_HP.red_outpost_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 12]);
      referee->game_robot_HP.red_base_HP    = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 14]);

      referee->game_robot_HP.blue_1_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 16]);
      referee->game_robot_HP.blue_2_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 18]);
      referee->game_robot_HP.blue_3_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 20]);
      referee->game_robot_HP.blue_4_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 22]);
      referee->game_robot_HP.blue_5_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 24]);
      referee->game_robot_HP.blue_7_robot_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 26]);
      referee->game_robot_HP.blue_outpost_HP = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 28]);
      referee->game_robot_HP.blue_base_HP    = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 30]);
    break;
#endif

#ifdef EVENE_DATA_ID
    case EVENE_DATA_ID:
      referee->event_data.event_data = bit8TObit32(&Buff[referee->index + FrameHeader_Length + CMDID_Length]);
		  referee->event_data.Out_recovery_point =     (referee->event_data.event_data & 0x01);
		  referee->event_data.In_recovery_point =      (referee->event_data.event_data & 0x02) >> 1;
		  referee->event_data.recovery_point_RMUL =    (referee->event_data.event_data & 0x04) >>2;
		  referee->event_data.Energy_buff_attack_point =  (referee->event_data.event_data & 0x08) >> 3;
		  referee->event_data.Little_energy_buff =     (referee->event_data.event_data & 0x010) >> 4;
		  referee->event_data.Large_energy_buff =   (referee->event_data.event_data & 0x020) >> 5;
      referee->event_data.Ring_Heights_land	= 	(referee->event_data.event_data & 0xC0) >> 6;
		  referee->event_data.R3_Trapezoidal_Heights_land = (referee->event_data.event_data & 0x300) >> 8;
		  referee->event_data.R4_Trapezoidal_Heights_land = (referee->event_data.event_data & 0xC00) >> 10;
		  referee->event_data.Base_shield_remaining_percentage = (referee->event_data.event_data & 0x7F000) >> 12;
		  referee->event_data.Dart_last_hit_time =  (referee->event_data.event_data &0xFF80000 ) >> 19;
		  referee->event_data.Dart_last_hit_Target  =  (referee->event_data.event_data & 0x30000000)>> 28;
		  referee->event_data.Center_gain_point_situation_RMUL = (referee->event_data.event_data & 0xC0000000)>>30;
    break;
#endif

#ifdef  SUPPLY_ACTION_ID  
		case SUPPLY_ACTION_ID:
     referee->supply_projectile_action.reserved = Buff[referee->index + FrameHeader_Length + CMDID_Length];
		 referee->supply_projectile_action.supply_robot_id = Buff[referee->index + FrameHeader_Length + CMDID_Length+1];
		 referee->supply_projectile_action.supply_projectile_step =  Buff[referee->index + FrameHeader_Length + CMDID_Length+2];
		 referee->supply_projectile_action.supply_projectile_num =  Buff[referee->index + FrameHeader_Length + CMDID_Length+3];
		break;
		 
#endif

#ifdef REFEREE_WARNING_ID
    case REFEREE_WARNING_ID:
     referee->referee_warning.level = Buff[referee->index + FrameHeader_Length + CMDID_Length];
      referee->referee_warning.offending_robot_id = Buff[referee->index + FrameHeader_Length + CMDID_Length+1];
			referee->referee_warning.count = Buff[referee->index + FrameHeader_Length + CMDID_Length+2];
	  break;
#endif

#ifdef DART_INFO_ID
    case DART_INFO_ID:
      referee->dart_info.dart_remaining_time = Buff[referee->index + FrameHeader_Length + CMDID_Length];
		   referee->dart_info.dart_info = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length+1]);
		   referee->dart_info.Lastest_target_hit =  (referee->dart_info.dart_info & 0x03) ;
		   referee->dart_info.Hit_count = (referee->dart_info.dart_info & 0x0C) >> 2 ;
		   referee->dart_info.Selected_target = (referee->dart_info.dart_info & 0x30) >> 4 ;
    break;
#endif

#ifdef ROBOT_STATUS_ID
    case ROBOT_STATUS_ID:
		    referee->robot_status.robot_id  = 	 Buff[referee->index + FrameHeader_Length + CMDID_Length];
		    referee->robot_status.robot_level =  Buff[referee->index + FrameHeader_Length + CMDID_Length + 1];
		    referee->robot_status.current_HP =   bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 2]);
		    referee->robot_status.maximum_HP =   bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 4]);
		    referee->robot_status.shooter_barrel_cooling_value =  bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 6]);
		    referee->robot_status.shooter_barrel_heat_limit =     bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 8]);
		    referee->robot_status.chassis_power_limit =           bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 10]);
        referee->robot_status.mains_power_gimbal_output  = (Buff[referee->index + FrameHeader_Length + CMDID_Length + 12] & 0x01);
        referee->robot_status.mains_power_chassis_output = (Buff[referee->index + FrameHeader_Length + CMDID_Length + 12] & 0x02) >> 1;
        referee->robot_status.mains_power_shooter_output = (Buff[referee->index + FrameHeader_Length + CMDID_Length + 12] & 0x04) >> 2;
    break;
#endif

#ifdef POWER_HEAT_ID
    case POWER_HEAT_ID:
      referee->power_heat_data.chassis_voltage    = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length]);
      referee->power_heat_data.chassis_current = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 2]);
      referee->power_heat_data.chassis_power   = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 4]);
      referee->power_heat_data.buffer_energy = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 8]);
      referee->power_heat_data.shooter_17mm_1_barrel_heat = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 10]);
      referee->power_heat_data.shooter_17mm_2_barrel_heat = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 12]);
      referee->power_heat_data.shooter_42mm_barrel_heat = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 14]);
    break;
#endif

#ifdef ROBOT_POSITION_ID
    case ROBOT_POSITION_ID:
      referee->robot_pos.x   = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length]);
      referee->robot_pos.y   = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 4]);
      referee->robot_pos.angle = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 8]);
    break;
#endif

#ifdef ROBOT_BUFF_ID
  case ROBOT_BUFF_ID:
      referee->buff.recovery_buff = Buff[referee->index + FrameHeader_Length + CMDID_Length];
      referee->buff.cooling_buff  = Buff[referee->index + FrameHeader_Length + CMDID_Length + 1];
      referee->buff.defence_buff  = Buff[referee->index + FrameHeader_Length + CMDID_Length + 2];
	    referee->buff.vulnerability_buff  = Buff[referee->index + FrameHeader_Length + CMDID_Length + 3];
	    referee->buff.attack_buff = bit8TObit16 (&Buff[referee->index + FrameHeader_Length + CMDID_Length + 4]);
    break;
#endif


#ifdef AIR_SUPPORT_ID
    case AIR_SUPPORT_ID:
      referee->air_support_data.airforce_status = Buff[referee->index + FrameHeader_Length + CMDID_Length];
		  referee->air_support_data.time_remain = Buff[referee->index + FrameHeader_Length + CMDID_Length + 1];
    break;
#endif

#ifdef ROBOT_HURT_ID
    case ROBOT_HURT_ID:
      referee->hurt_data.armor_id  = Buff[referee->index + FrameHeader_Length + CMDID_Length] & 0x0F ;
      referee->hurt_data.HP_deduction_reason = (Buff[referee->index + FrameHeader_Length + CMDID_Length] & 0xF0) >>4;
    break;
#endif

#ifdef SHOOT_DATA_ID
    case SHOOT_DATA_ID:
      referee->shoot_data.bullet_type  = Buff[referee->index + FrameHeader_Length + CMDID_Length];
      referee->shoot_data.shooter_number   = Buff[referee->index + FrameHeader_Length + CMDID_Length + 1];
      referee->shoot_data.launching_frequency  = Buff[referee->index + FrameHeader_Length + CMDID_Length + 2];
      referee->shoot_data.initial_speed = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 3]);
    break;
#endif

#ifdef PROJECTILE_ALLOWANCE_ID
    case PROJECTILE_ALLOWANCE_ID:
      referee->projectile_allowance.projectile_allowance_17mm = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length]);
      referee->projectile_allowance.projectile_allowance_42mm = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 2]);
      referee->projectile_allowance.remaining_gold_coin       = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 4]);
    break;
#endif

#ifdef RFID_STATUS_ID
    case RFID_STATUS_ID:
      referee->rfid_status.rfid_status = bit8TObit32(&Buff[referee->index+FrameHeader_Length+CMDID_Length]);
    break;
#endif

#ifdef DART_CLIENT_CMD_ID
    case DART_CLIENT_CMD_ID:
      referee->dart_client_cmd.dart_launch_opening_status = Buff[referee->index + FrameHeader_Length + CMDID_Length];
      referee->dart_client_cmd.reserved = Buff[referee->index + FrameHeader_Length + CMDID_Length + 1];
      referee->dart_client_cmd.target_change_time = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 2]);
      referee->dart_client_cmd.latest_launch_cmd_time = bit8TObit16(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 4]);
    break;
#endif

#ifdef GROUND_ROBOT_POSITION_ID
    case GROUND_ROBOT_POSITION_ID:
      referee->ground_robot_position.hero_x       = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length]);
      referee->ground_robot_position.hero_y       = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 4]);
      referee->ground_robot_position.engineer_x   = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 8]);
      referee->ground_robot_position.engineer_y   = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 12]);
      referee->ground_robot_position.standard_3_x = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 16]);
      referee->ground_robot_position.standard_3_y = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 20]);
      referee->ground_robot_position.standard_4_x = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 24]);
      referee->ground_robot_position.standard_4_y = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 28]);
      referee->ground_robot_position.standard_5_x = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 32]);
      referee->ground_robot_position.standard_5_y = bit8TOfloat32(&Buff[referee->index + FrameHeader_Length + CMDID_Length + 36]);
    break;
#endif

#ifdef RADAR_MARAKING_ID
	 case RADAR_MARAKING_ID :
     referee->radar_mark_data.mark_hero_progress = Buff[referee->index + FrameHeader_Length + CMDID_Length];
     referee->radar_mark_data.mark_engineer_progress = Buff[referee->index + FrameHeader_Length + CMDID_Length+1];
	   referee->radar_mark_data.mark_standard_3_progress = Buff[referee->index + FrameHeader_Length + CMDID_Length+2];
	   referee->radar_mark_data.mark_standard_4_progress = Buff[referee->index + FrameHeader_Length + CMDID_Length+3];
	   referee->radar_mark_data.mark_standard_5_progress = Buff[referee->index + FrameHeader_Length + CMDID_Length+4];
	   referee->radar_mark_data.mark_sentry_progress     = Buff[referee->index + FrameHeader_Length + CMDID_Length+5];
   break;
#endif
	
#ifdef SENTRY_INFO_ID
	 case SENTRY_INFO_ID:
		 referee->sentry_info.sentry_info = bit8TObit32(&Buff[referee->index + FrameHeader_Length + CMDID_Length]);
	   referee->sentry_info.Sentry_Exchange_bullet =  (referee->sentry_info.sentry_info & 0x3FF);
	   referee->sentry_info.Sentry_Remote_Exchange_bullet_count =  (referee->sentry_info.sentry_info & 0x3C00)>>10;
	   referee->sentry_info.Sentry_Remote_Exchange_HP_count = (referee->sentry_info.sentry_info & 0x3C000)>>14;
	   
	 break;
#endif
#ifdef RADAR_INFO_ID
	 case RADAR_INFO_ID:
		 referee->radar_info.radar_info = Buff[referee->index + FrameHeader_Length + CMDID_Length];
	    referee->radar_info.radar_double_damage_chance =   (referee->radar_info.radar_info & 0x03);
	    referee->radar_info.opposite_radar_double_damage = (referee->radar_info.radar_info & 0x04)>>2;
	 break;
#endif	 
	 
	 
    default:break;
  }
}

/**
 * @brief  transform the bit8 to bit32
*/
static uint32_t bit8TObit32(uint8_t change_info[4])
{
	union
	{
    uint32_t bit32;
		uint8_t  byte[4];
	}u32val;

  u32val.byte[0] = change_info[0];
  u32val.byte[1] = change_info[1];
  u32val.byte[2] = change_info[2];
  u32val.byte[3] = change_info[3];

	return u32val.bit32;
}
//------------------------------------------------------------------------------
static uint32_t bit8TObit64(uint8_t change_info[8]){
 	union
	{
    uint64_t bit32;
		uint8_t  byte[4];
	}u64val;

  u64val.byte[0] = change_info[0];
  u64val.byte[1] = change_info[1];
  u64val.byte[2] = change_info[2];
  u64val.byte[3] = change_info[3];

	return u64val.bit32;

}
/**
 * @brief  transform the bit8 to float32
*/
static float bit8TOfloat32(uint8_t change_info[4])
{
	union
	{
    float float32;
		uint8_t  byte[4];
	}u32val;

  u32val.byte[0] = change_info[0];
  u32val.byte[1] = change_info[1];
  u32val.byte[2] = change_info[2];
  u32val.byte[3] = change_info[3];

	return u32val.float32;
}
//------------------------------------------------------------------------------

///**
// * @brief  transform the bit32 to bit8
//*/
//static uint8_t bit32TObit8(uint8_t index_need,uint32_t bit32)
//{
//	union
//	{
//    uint32_t  bit32;
//		uint8_t  byte[4];
//	}u32val;

//  u32val.bit32 = bit32;

//	return u32val.byte[index_need];
//}
//------------------------------------------------------------------------------

/**
 * @brief  transform the bit8 to bit16
*/
static int16_t bit8TObit16(uint8_t change_info[2])
{
	union
	{
    int16_t  bit16;
		uint8_t  byte[2];
	}u16val;

  u16val.byte[0] = change_info[0];
  u16val.byte[1] = change_info[1];

	return u16val.bit16;
}
//------------------------------------------------------------------------------

///**
// * @brief  transform the bit16 to bit8
//*/
//static uint8_t bit16TObit8(uint8_t index_need,int16_t bit16)
//{
//	union
//	{
//    int16_t  bit16;
//		uint8_t  byte[2];
//	}u16val;

//  u16val.bit16 = bit16;
//	return u16val.byte[index_need];
//}
//------------------------------------------------------------------------------





