/**
  ******************************************************************************
  * @file    app_bluenrg_2.c
  * @author  SRA Application Team
  * @brief   BlueNRG-2 initialization and applicative code
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_bluenrg_2.h"

#include "bluenrg_conf.h"
#include "central.h"

#include "hci.h"
#include "hci_tl.h"
#include "bluenrg1_events.h"
#include "bluenrg1_gap_aci.h"

#include "stm32f4xx_nucleo.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Defines */

/* USER CODE END Defines */

/* Private variables ---------------------------------------------------------*/
extern savedDevices_t   saved_devices;
extern nonConnDevices_t non_conn_devices;
extern serviceInfo_t    serv_info[MAX_NUM_OF_SERVICES];
extern centralStatus_t  central_status;

/* Primary Service UUID expected from peripherals (LSB-first) */
uint8_t GENERIC_ACCESS_PROFILE_UUID[]    = {0x00, 0x18};
uint8_t GENERIC_ATTRIBUTE_PROFILE_UUID[] = {0x01, 0x18};

/* BlueST main service UUID base */
uint8_t ST_HARDWARE_SERVICE_UUID[] = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xb4,0x9a,0xe1,0x11,0x01,0x00,0x00,0x00,0x00,0x00};

/* USER CODE BEGIN PV */

static uint16_t accel_value_handle = 0x0000;
static uint8_t  accel_serv_idx     = 0xFF;
static uint8_t  accel_char_idx     = 0xFF;

volatile float valori_ricevuti[3] = {0.0f, 0.0f, 0.0f};
volatile uint8_t ready_data = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void User_Init(void);
static void User_Process(void);
static void Central_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

void MX_BlueNRG_2_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN BlueNRG_2_Init_PreTreatment */

  /* USER CODE END BlueNRG_2_Init_PreTreatment */

  uint8_t ret;

  User_Init();

  /* LED ON durante l'init, OFF a fine boot. Si riaccenderà alla connessione. */
  BSP_LED_On(LED2);

  hci_init(APP_UserEvtRx, NULL);

  ret = CentralDevice_Init();
  if (ret == BLE_STATUS_SUCCESS) {
    Init_Saved_Devices();
    Init_NonConn_Devices();
    central_status = START_SCANNING;
    printf("[BOOT] BLE central pronto\r\n");
  } else {
    printf("[BOOT] CentralDevice_Init FALLITA 0x%02x\r\n", ret);
  }

  BSP_LED_Off(LED2);

  /* USER CODE BEGIN BlueNRG_2_Init_PostTreatment */

  /* USER CODE END BlueNRG_2_Init_PostTreatment */
}

/*
 * BlueNRG-2 background task
 */
void MX_BlueNRG_2_Process(void)
{
  /* USER CODE BEGIN BlueNRG_2_Process_PreTreatment */

  /* USER CODE END BlueNRG_2_Process_PreTreatment */

  hci_user_evt_proc();
  User_Process();

  /* USER CODE BEGIN BlueNRG_2_Process_PostTreatment */

  /* USER CODE END BlueNRG_2_Process_PostTreatment */
}

/**
 * @brief  Initialize User process.
 *
 * @param  None
 * @retval None
 */
static void User_Init(void)
{
  BSP_LED_Init(LED2);
  BSP_COM_Init(COM1);
}

/**
 * @brief  User Process — flow completamente automatico, niente console.
 */
static void User_Process(void)
{
  Central_Process();
}
/**
 * @brief  Manage all the BlueNRG actions a Central device can run
 */
void Central_Process(void)
{
  uint8_t dev_idx  = saved_devices.dev_idx;
  uint8_t serv_idx = saved_devices.dev_info[dev_idx].serv_idx;
  uint8_t char_idx;
  uint8_t prop_idx;

  switch (central_status)
  {
  case (START_SCANNING):
    printf("\r\n[SCAN] In ricerca della SensorTile...\r\n");
    central_status = SCANNING_STARTED;
    Start_Scanning();
    break;

  case (START_CONNECTION):
    printf("[CONN] Connessione in corso...\r\n");
    central_status = CONNECTION_STARTED;
    Start_Connection(dev_idx);
    break;

  case (SERVICE_DISCOVERY):
    printf("[GATT] Discovery servizi...\r\n");
    central_status = SERVICE_DISCOVERY_STARTED;
    Discover_Services(dev_idx);
    break;

  case (CHARACTERISTIC_DISCOVERY):
    central_status = CHARACTERISTIC_DISCOVERY_STARTED;
    Discover_Characteristics(dev_idx, serv_idx);
    break;

  case (UPDATE_CHARACTERISTIC):
    central_status = WAITING_UPDATE_CHARACTERISTIC;
    char_idx = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_idx;
    prop_idx = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].prop_idx;
    Update_Characteristic(dev_idx, serv_idx, char_idx, prop_idx);
    break;

  case (DISCONNECTION_COMPLETE):
    printf("[DISC] Disconnesso, riavvio scansione...\r\n");
    Init_Saved_Devices();
    Init_NonConn_Devices();
    accel_value_handle = 0;
    accel_serv_idx     = 0xFF;
    accel_char_idx     = 0xFF;
    central_status = START_SCANNING;
    break;

  default:
    /* Stati di transizione: SCANNING_STARTED, CONNECTION_STARTED, ecc. */
    break;
  }
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/
/*******************************************************************************
 * Function Name  : aci_gap_proc_complete_event.
 * Description    : This event indicates the end of a GAP procedure.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gap_proc_complete_event(uint8_t Procedure_Code,
                                 uint8_t Status,
                                 uint8_t Data_Length,
                                 uint8_t Data[])
{
	if (Procedure_Code == GAP_GENERAL_DISCOVERY_PROC)
	  {

	    if (central_status == START_CONNECTION ||
	        central_status == CONNECTION_STARTED ||
	        central_status == CONNECTION_COMPLETE) {
	      return;
	    }

	    if (saved_devices.dev_num > 0) {
	      central_status = SELECT_DEVICE;
	    }
	    else {
	      central_status = START_SCANNING;
	    }
	  }
}

/*******************************************************************************
 * Function Name  : aci_gatt_proc_complete_event.
 * Description    : This event indicates the end of a GATT procedure.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_proc_complete_event(uint16_t Connection_Handle,
                                  uint8_t Error_Code)
{
  uint8_t dev_idx  = saved_devices.dev_idx;

  if (Error_Code != BLE_STATUS_SUCCESS) {
    printf("[GATT] Procedura fallita 0x%02x\r\n", Error_Code);
  }

  switch(central_status)
  {
  case SERVICE_DISCOVERY_STARTED:
    printf("[GATT] %d servizi trovati\r\n", saved_devices.dev_info[dev_idx].serv_num);
    saved_devices.dev_info[dev_idx].serv_idx = 0;
    central_status = CHARACTERISTIC_DISCOVERY;
    break;

  case CHARACTERISTIC_DISCOVERY_STARTED:
    if (saved_devices.dev_info[dev_idx].serv_idx == saved_devices.dev_info[dev_idx].serv_num-1) {
      if (accel_value_handle != 0) {
        /* Punta ai target char accelerometro per Update_Characteristic */
        saved_devices.dev_info[dev_idx].serv_idx = accel_serv_idx;
        saved_devices.dev_info[dev_idx].serv_info[accel_serv_idx].char_idx = accel_char_idx;
        central_status = UPDATE_CHARACTERISTIC;
      } else {
        printf("[GATT] ERRORE: accelerometro non trovato\r\n");
        Close_Connection();
      }
    } else {
      saved_devices.dev_info[dev_idx].serv_idx++;
      central_status = CHARACTERISTIC_DISCOVERY;
    }
    break;

  case WAITING_UPDATE_CHARACTERISTIC:
    if (Error_Code != BLE_STATUS_SUCCESS) {
      printf("[GATT] Notify enable FALLITA\r\n");
      Close_Connection();
    } else {
      printf("[STREAM] Notifiche attive — formato CSV: X,Y,Z (mg)\r\n");
      central_status = RECEIVE_NOTIFICATIONS;
    }
    break;

  default:
    break;
  }

}

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates the end of a connection procedure.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{
  if (Status != BLE_STATUS_SUCCESS) {
    printf("[CONN] FALLITA, status=0x%02x — nuovo scan\r\n", Status);
    BSP_LED_Off(LED2);
    central_status = START_SCANNING;
    return;
  }

  saved_devices.connected = TRUE;
  saved_devices.dev_info[saved_devices.dev_idx].conn_handle = Connection_Handle;
  saved_devices.dev_info[saved_devices.dev_idx].serv_info   = serv_info;

  /* LED2 acceso = connesso */
  BSP_LED_On(LED2);

  printf("[CONN] OK — %02x:%02x:%02x:%02x:%02x:%02x  handle=0x%04x\r\n",
         saved_devices.dev_info[saved_devices.dev_idx].bdaddr[5],
         saved_devices.dev_info[saved_devices.dev_idx].bdaddr[4],
         saved_devices.dev_info[saved_devices.dev_idx].bdaddr[3],
         saved_devices.dev_info[saved_devices.dev_idx].bdaddr[2],
         saved_devices.dev_info[saved_devices.dev_idx].bdaddr[1],
         saved_devices.dev_info[saved_devices.dev_idx].bdaddr[0],
         Connection_Handle);

  central_status = SERVICE_DISCOVERY;

} /* hci_le_connection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event.
 * Description    : This event indicates the end of a disconnection procedure.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  (void)Status;
  (void)Connection_Handle;
  (void)Reason;

  saved_devices.connected = FALSE;
  BSP_LED_Off(LED2);             /* LED2 spento = non connesso */
  central_status = DISCONNECTION_COMPLETE;
}

/*******************************************************************************
 * Function Name  : hci_le_advertising_report_event.
 * Description    : An advertising report is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_advertising_report_event(uint8_t Num_Reports, Advertising_Report_t Advertising_Report[])
{
  /* MAC target in little-endian (visualizzato standard: FF:32:0C:B4:91:3D) */
  static const uint8_t target_mac[6] = {0x3D, 0x91, 0xB4, 0x0C, 0x32, 0xFF};
  uint8_t *adv_addr = Advertising_Report[0].Address;
  (void)Num_Reports;

  if (non_conn_devices.dev_num < MAX_NUM_OF_DEVICES &&
      Is_Device_Saved(adv_addr) == FALSE &&
      Is_Device_Scanned(adv_addr) == FALSE) {
    printf("[SCAN] %02X:%02X:%02X:%02X:%02X:%02X  rssi=%d\r\n",
           adv_addr[5], adv_addr[4], adv_addr[3], adv_addr[2], adv_addr[1], adv_addr[0],
           (int8_t)Advertising_Report[0].RSSI);
    Save_NonConn_Device(adv_addr);
  }

  if (saved_devices.connected || central_status == START_CONNECTION ||
      central_status == CONNECTION_STARTED) {
    return;
  }

  if (BLUENRG_memcmp(adv_addr, target_mac, 6) == 0)
  {
    printf("[SCAN] Target SensorTile trovato\r\n");

    aci_gap_terminate_gap_proc(GAP_GENERAL_DISCOVERY_PROC);

    Save_Found_Device(adv_addr, &Advertising_Report[0].Address_Type,
                      Advertising_Report[0].Length_Data, Advertising_Report[0].Data, 0);
    saved_devices.dev_num = 1;
    saved_devices.dev_idx = 0;

    central_status = START_CONNECTION;
  }
}
/*******************************************************************************
 * Function Name  : aci_att_read_by_group_type_resp_event.
 * Description    : A response event is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_att_read_by_group_type_resp_event(uint16_t Connection_Handle,
                                           uint8_t  Attribute_Data_Length,
                                           uint8_t  Data_Length,
                                           uint8_t  Attribute_Data_List[])
{
  uint8_t  *uuid, *uuid_length, *name, *name_length;
  serviceType_t *serv_type;
  uint8_t  i, offset, num_attr;
  uint8_t  dev_idx = saved_devices.dev_idx;
  uint8_t  serv_idx;
  uint16_t *start_handle, *end_handle;
  char const* serv_name;

  num_attr = (Data_Length / Attribute_Data_Length);
  offset = 0;

  for (i=0; i<num_attr; i++) {
    serv_idx     = saved_devices.dev_info[dev_idx].serv_idx;
    serv_type    = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].serv_type);
    uuid         = saved_devices.dev_info[dev_idx].serv_info[serv_idx].uuid;
    uuid_length  = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].uuid_length);
    name         = saved_devices.dev_info[dev_idx].serv_info[serv_idx].name;
    name_length  = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].name_length);
    start_handle = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].start_handle);
    end_handle   = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].end_handle);

    *start_handle = Attribute_Data_List[offset] | (Attribute_Data_List[offset+1] << 8);
    *end_handle   = Attribute_Data_List[offset+2] | (Attribute_Data_List[offset+3] << 8);

    *serv_type = CUSTOM_SERVICE_TYPE;
    serv_name = CUSTOM_SERVICE_NAME;

    if (Attribute_Data_Length == 6) {
      uuid[0] = Attribute_Data_List[offset+4];
      uuid[1] = Attribute_Data_List[offset+5];
      *uuid_length = UUID_MIN_LENGTH;
      if (BLUENRG_memcmp(uuid, GENERIC_ACCESS_PROFILE_UUID, (Attribute_Data_Length-4)) == 0) {
        *serv_type = GENERIC_ACCESS_PROFILE_TYPE;
        serv_name = GENERIC_ACCESS_PROFILE_NAME;
      }
      else if (BLUENRG_memcmp (uuid, GENERIC_ATTRIBUTE_PROFILE_UUID, (Attribute_Data_Length-4)) == 0) {
        *serv_type = GENERIC_ATTRIBUTE_PROFILE_TYPE;
        serv_name = GENERIC_ATTRIBUTE_PROFILE_NAME;
      }
    }
    else {
      *uuid_length = UUID_MAX_LENGTH;
      BLUENRG_memcpy(uuid, Attribute_Data_List+offset+4, *uuid_length);

      if (BLUENRG_memcmp(&uuid[4], &ST_HARDWARE_SERVICE_UUID[4], 12) == 0) {
        *serv_type = CUSTOM_SERVICE_TYPE;
        serv_name = "BlueST Custom Service";
      }
    }

    *name_length = (strlen(serv_name) > MAX_NAME_LENGTH) ? MAX_NAME_LENGTH : strlen(serv_name);
    BLUENRG_memcpy(name, serv_name, *name_length);

    saved_devices.dev_info[dev_idx].serv_idx++;
    offset += Attribute_Data_Length;
  }

  saved_devices.dev_info[dev_idx].serv_num += num_attr;
}

/*******************************************************************************
 * Function Name  : aci_att_read_by_type_resp_event.
 * Description    : A response event is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_att_read_by_type_resp_event(uint16_t Connection_Handle,
                                     uint8_t Handle_Value_Pair_Length,
                                     uint8_t Data_Length,
                                     uint8_t Handle_Value_Pair_Data[])
{
  uint8_t  *uuid, *uuid_length, *name, *name_length;
  uint16_t *decl_handle, *value_handle;
  uint8_t  *broadcast, *read, *write_wo_resp, *write, *notify, *indicate, *auth_signed_write;
  uint8_t  char_idx;
  characteristicType_t *char_type;
  char const* char_name;

  uint8_t dev_idx  = saved_devices.dev_idx;
  uint8_t serv_idx = saved_devices.dev_info[dev_idx].serv_idx;

  /* Calcola quante coppie handle/valore sono presenti nel pacchetto */
  uint8_t num_pairs = Data_Length / Handle_Value_Pair_Length;

  for (uint8_t i = 0; i < num_pairs; i++) {
    uint8_t offset = i * Handle_Value_Pair_Length;

    char_idx = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_idx;
    if (char_idx >= MAX_NUM_OF_CHARS) break;

    /* Mappatura puntatori alla struttura dati */
    char_type    = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].char_type);
    uuid         = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].uuid;
    uuid_length  = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].uuid_length);
    name         = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].name;
    name_length  = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].name_length);
    decl_handle  = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].decl_handle);
    value_handle = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].value_handle);

    broadcast         = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].broadcast);
    read              = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].read);
    write_wo_resp     = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].write_wo_resp);
    write             = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].write);
    notify            = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].notify);
    indicate          = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].indicate);
    auth_signed_write = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].auth_signed_write);

    /* Estrazione Handles e Proprietà */
    *decl_handle  = Handle_Value_Pair_Data[offset] | (Handle_Value_Pair_Data[offset+1] << 8);
    *value_handle = Handle_Value_Pair_Data[offset+3] | (Handle_Value_Pair_Data[offset+4] << 8);

    *broadcast         = (0x01 & Handle_Value_Pair_Data[offset+2]);
    *read              = (0x02 & Handle_Value_Pair_Data[offset+2]) >> 1;
    *write_wo_resp     = (0x04 & Handle_Value_Pair_Data[offset+2]) >> 2;
    *write             = (0x08 & Handle_Value_Pair_Data[offset+2]) >> 3;
    *notify            = (0x10 & Handle_Value_Pair_Data[offset+2]) >> 4;
    *indicate          = (0x20 & Handle_Value_Pair_Data[offset+2]) >> 5;
    *auth_signed_write = (0x40 & Handle_Value_Pair_Data[offset+2]) >> 6;

    /* Estrazione UUID (16 o 128 bit) */
    if (Handle_Value_Pair_Length == 7) {
      *uuid_length = 2;
      uuid[0] = Handle_Value_Pair_Data[offset+5];
      uuid[1] = Handle_Value_Pair_Data[offset+6];
    } else {
      *uuid_length = 16;
      BLUENRG_memcpy(uuid, Handle_Value_Pair_Data+offset+5, 16);
    }

    static const uint8_t BLUEST_V1_BASE[12] = {
        0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00};
    static const uint8_t BLUEST_V2_BASE[12] = {
        0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xb4,0x9a,0xe1,0x11,0x01,0x00};

    char_name = "Custom Char";
    *char_type = CUSTOM_CHAR_TYPE;

    if (*uuid_length == 16) {
      uint8_t is_bluest = (BLUENRG_memcmp(uuid, BLUEST_V1_BASE, 12) == 0) ||
                          (BLUENRG_memcmp(uuid, BLUEST_V2_BASE, 12) == 0);
      if (is_bluest) {

        uint8_t fm_b14 = uuid[14];

        if ((fm_b14 & 0x80) && accel_value_handle == 0) {
          accel_value_handle = *value_handle;
          accel_serv_idx     = serv_idx;
          accel_char_idx     = char_idx;
          *char_type = ST_ACC_GYRO_MAG_CHAR_TYPE;
          char_name = ((fm_b14 & 0xE0) == 0xE0) ? "ST Acc+Gyro+Mag" : "ST Acc";
          printf("[GATT] Accelerometro: handle=0x%04x\r\n", accel_value_handle);
          saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].prop_idx = 4; /* notify */
        }
      }
    }

    *name_length = strlen(char_name);
    if (*name_length > MAX_NAME_LENGTH) *name_length = MAX_NAME_LENGTH;
    BLUENRG_memcpy(name, char_name, *name_length);

    /* Incrementa contatori per la prossima caratteristica nel pacchetto */
    saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_idx++;
    saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_num++;
  }
}
/*******************************************************************************
 * Function Name  : aci_gatt_notification_event.
 * Description    : A response event is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 ******************************************************************************/
void aci_gatt_notification_event(uint16_t Connection_Handle,
                                 uint16_t Attribute_Handle,
                                 uint8_t Attribute_Value_Length,
                                 uint8_t Attribute_Value[])
{
  if ((accel_value_handle != 0) &&
      (Attribute_Handle == accel_value_handle) &&
      (Attribute_Value_Length >= 8))
  {
    /* Payload BlueST V1: [ts:u16 LE][X:i16 LE][Y:i16 LE][Z:i16 LE] in mg */
	  float x = 9.81f * ((float)((int16_t)(Attribute_Value[3] << 8 | Attribute_Value[2])) / 1000.0f);
	  float y = 9.81f * ((float)((int16_t)(Attribute_Value[5] << 8 | Attribute_Value[4])) / 1000.0f);
	  float z = 9.81f * ((float)((int16_t)(Attribute_Value[7] << 8 | Attribute_Value[6])) / 1000.0f);

	  valori_ricevuti[0] = y;
	  valori_ricevuti[1] = -x;
	  valori_ricevuti[2] = z;

	  ready_data = 1;

    // printf("[%d] %f,%f,%f\r\n", x, y, z);
  }
}
