/**
  ******************************************************************************
  * @file    App/central.c
  * @author  SRA Application Team
  * @brief   Central device init and state machine implementation
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "central.h"
#include "bluenrg_conf.h"
#include "bluenrg1_hal_aci.h"
#include "bluenrg1_gap.h"
#include "bluenrg1_gap_aci.h"
#include "bluenrg1_hci_le.h"
#include "hci_const.h"
#include "bluenrg1_gatt_aci.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define BDADDR_SIZE        6

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
centralStatus_t  central_status = INIT_STATUS;
savedDevices_t   saved_devices;
nonConnDevices_t non_conn_devices;
serviceInfo_t    serv_info[MAX_NUM_OF_SERVICES];

/* Private function prototypes -----------------------------------------------*/
static uint8_t Get_BLEFirmware_Details(void);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Get BlueNRG-2 hardware and firmware details
 *
 * @param  None
 * @retval Status
 */
uint8_t Get_BLEFirmware_Details(void)
{
  uint8_t status;

  uint8_t  DTM_version_major, DTM_version_minor, DTM_version_patch, DTM_variant;
  uint16_t DTM_Build_Number;
  uint8_t  BTLE_Stack_version_major, BTLE_Stack_version_minor, BTLE_Stack_version_patch,
           BTLE_Stack_development;
  uint16_t BTLE_Stack_variant, BTLE_Stack_Build_Number;
  uint8_t  BTLE_sv_patch, DTM_v_patch;
  uint8_t  alphabet[]={' ', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z'};

  status = aci_hal_get_firmware_details(&DTM_version_major, &DTM_version_minor,
                                        &DTM_version_patch, &DTM_variant, &DTM_Build_Number,
                                        &BTLE_Stack_version_major, &BTLE_Stack_version_minor,
                                        &BTLE_Stack_version_patch, &BTLE_Stack_development,
                                        &BTLE_Stack_variant, &BTLE_Stack_Build_Number);

  BTLE_sv_patch = alphabet[BTLE_Stack_version_patch];
  DTM_v_patch = alphabet[DTM_version_patch];

  printf("\r\n[BOOT] SensorTile Central v%d.%d.%d\r\n",
         CENTRAL_MAJOR_VERSION, CENTRAL_MINOR_VERSION, CENTRAL_PATCH_VERSION);

  if (status == BLE_STATUS_SUCCESS) {
    printf("[BOOT] BlueNRG-2 FW v%d.%d%c - DTM %s v%d.%d%c\r\n",
           BTLE_Stack_version_major, BTLE_Stack_version_minor, BTLE_sv_patch,
           DTM_variant == 0x01 ? "UART" : (DTM_variant == 0x02 ? "SPI" : "?"),
           DTM_version_major, DTM_version_minor, DTM_v_patch);
  }

  return status;
}

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  Init the Central device
 * @note
 * @param  None
 * @retval None
 */
uint8_t CentralDevice_Init(void)
{
  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  uint8_t device_name[] = {'B','L','E','2','_','C','e','n','t','r','a','l'};
  uint8_t bdaddr[BDADDR_SIZE];
  uint8_t bdaddr_len_out;
  uint8_t config_data_stored_static_random_address = 0x80; /* Offset of the static random address stored in NVM */

  /* Sw reset of the device */
  /* Nel file central.c, all'interno di CentralDevice_Init */

  /* 1. Reset standard dello stack BlueNRG-2 */
  ret = hci_reset();
  if (ret != BLE_STATUS_SUCCESS) return ret;

  /* Attendi un breve istante per il reset hardware */
  HAL_Delay(500);

  /* Cancella il database di sicurezza (bonding info) salvato in Flash */
  ret = aci_gap_clear_security_db();
  if (ret != BLE_STATUS_SUCCESS) {
      // Se fallisce qui, il database è già vuoto
  }
  /*
   *  To support both the BlueNRG-2 and the BlueNRG-2N a minimum delay of 2000ms is required at device boot
   */
  HAL_Delay(2000);

  /* get the BlueNRG HW and FW versions */
  ret = Get_BLEFirmware_Details();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINT_DBG("Get_BLEFirmware_Details() --> Failed 0x%02x\r\n", ret);
    return ret;
  }

  ret = aci_hal_read_config_data(config_data_stored_static_random_address,
                                 &bdaddr_len_out, bdaddr);

  if (ret != BLE_STATUS_SUCCESS) {
    PRINT_DBG("aci_hal_read_config_data() --> Read Static Random address failed 0x%02x\r\n", ret);
    return ret;
  }
  if ((bdaddr[5] & 0xC0) != 0xC0) {
    PRINT_DBG("Static Random address not well formed.\r\n");
    while(1);
  }

  /* Set the TX power -2 dBm */
  aci_hal_set_tx_power_level(1, 4);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_hal_set_tx_power_level() --> Failed 0x%04x\r\n", ret);
    return ret;
  }

  /* GATT Init */
  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gatt_init() failed: 0x%02x\r\n", ret);
    return ret;
  }

  /* GAP Init */
  ret = aci_gap_init(GAP_CENTRAL_ROLE, 0, sizeof(device_name), &service_handle, &dev_name_char_handle,
                     &appearance_char_handle);

  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gap_init() --> Failed 0x%02x\r\n", ret);
    return ret;
  }

  /* No-pairing / JustWorks: il firmware entry-expert della SensorTile.box PRO
     non usa PIN*/
  ret = aci_gap_set_io_capability(0x03); /* NoInputNoOutput */
  if (ret != BLE_STATUS_SUCCESS) return ret;

  ret = aci_gap_set_authentication_requirement(
        0x00,   /* Bonding_Mode: NO_BONDING */
        0x00,   /* MITM_Mode:    MITM_PROTECTION_NOT_REQUIRED */
        0x00,   /* SC_Support:   disabled */
        0x00,   /* KeyPress_Notification_Support */
        7, 16,
        0x00,   /* Use_Fixed_Pin: 0 (no fixed PIN) */
        0,
        0x00);
  if (ret != BLE_STATUS_SUCCESS) return ret;

  /* Update device name */
  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, sizeof(device_name),
                                   device_name);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gatt_update_char_value() --> Failed 0x%02x\r\n", ret);
    return ret;
  }

  return ret;
}

/**
 * @brief  Init struct containing all saved devices
 * @param  None
 * @retval None
 */
void Init_Saved_Devices(void)
{
  uint8_t i, j, t;

  for (i=0; i<MAX_NUM_OF_DEVICES; i++) {
    BLUENRG_memset(saved_devices.dev_info[i].bdaddr, 0 , sizeof(saved_devices.dev_info[i].bdaddr));
    saved_devices.dev_info[i].addr_type   = 0;
    saved_devices.dev_info[i].name_length = 0;
    BLUENRG_memset(saved_devices.dev_info[i].name, 0, MAX_NAME_LENGTH);
    saved_devices.dev_info[i].serv_idx    = 0;
    saved_devices.dev_info[i].serv_num    = 0;
    saved_devices.dev_info[i].conn_handle = 0;
    saved_devices.dev_info[i].serv_info   = NULL;
  }
  for (j=0; j<MAX_NUM_OF_SERVICES; j++) {
    serv_info[j].char_idx     = 0;
    serv_info[j].char_num     = 0;
    serv_info[j].start_handle = 0;
    serv_info[j].end_handle   = 0;
    serv_info[j].name_length  = 0;
    BLUENRG_memset(serv_info[j].name, 0, MAX_NAME_LENGTH);
    serv_info[j].serv_type    = NO_SERVICE_TYPE;
    serv_info[j].uuid_length  = 0;
    BLUENRG_memset(serv_info[j].uuid, 0, UUID_MAX_LENGTH);
    for (t=0; t<MAX_NUM_OF_CHARS; t++) {
      serv_info[j].char_info[t].uuid_length       = 0;
      BLUENRG_memset(serv_info[j].char_info[t].uuid, 0, UUID_MAX_LENGTH);
      serv_info[j].char_info[t].name_length       = 0;
      BLUENRG_memset(serv_info[j].char_info[t].name, 0, MAX_NAME_LENGTH);
      serv_info[j].char_info[t].uuid_length       = 0;
      serv_info[j].char_info[t].decl_handle       = 0;
      serv_info[j].char_info[t].value_handle      = 0;
      serv_info[j].char_info[t].prop_idx          = 0;
      serv_info[j].char_info[t].broadcast         = 0;
      serv_info[j].char_info[t].read              = 0;
      serv_info[j].char_info[t].write_wo_resp     = 0;
      serv_info[j].char_info[t].write             = 0;
      serv_info[j].char_info[t].notify            = 0;
      serv_info[j].char_info[t].indicate          = 0;
      serv_info[j].char_info[t].auth_signed_write = 0;
      serv_info[j].char_info[t].char_type = NO_CHARACTERISTIC_TYPE;
    }
  }

  saved_devices.dev_idx   = 0;
  saved_devices.connected = 0;
  saved_devices.dev_num   = 0;
}

/**
 * @brief  Init struct containing all non connectable devices
 * @param  None
 * @retval None
 */
void Init_NonConn_Devices()
{
  uint8_t i;

  for (i=0; i<MAX_NUM_OF_DEVICES; i++) {
    BLUENRG_memset(non_conn_devices.bdaddr[i], 0 , sizeof(non_conn_devices.bdaddr[i]));
  }

  non_conn_devices.dev_idx=0;
  non_conn_devices.dev_num=0;
}

/**
 * @brief  Close the connection with the peripheral device
 * @param  None
 * @retval None
 */
void Close_Connection(void)
{
  uint8_t i = saved_devices.dev_idx;
  int ret;

  ret = aci_gap_terminate(saved_devices.dev_info[i].conn_handle, BLE_ERROR_TERMINATED_LOCAL_HOST);

  if (ret != BLE_STATUS_SUCCESS){
    printf("aci_gap_terminate() failed: %02X\n",ret);
  }

  HAL_Delay(100); /* see comment @file bluenrg1_gap_aci.h */
}

/**
 * @brief  Start searching for a peripheral device
 * @param  None
 * @retval None
 */
void Start_Scanning(void)
{
  int ret;

  /* scanInterval, scanWindow, own_address_type, filterDuplicates */
  ret = aci_gap_start_general_discovery_proc(SCAN_P, SCAN_L, PUBLIC_ADDR, 1);

  if (ret != BLE_STATUS_SUCCESS){
    printf("aci_gap_start_general_discovery_proc() failed: %02X\n",ret);
  }
}

/**
 * @brief  Start connection with a peripheral device
 * @param  Device index to connect
 * @retval None
 */
void Start_Connection(uint8_t dev_index)
{
  uint8_t ret;

  ret = aci_gap_create_connection(SCAN_P, SCAN_L,
                                  saved_devices.dev_info[dev_index].addr_type,
                                  saved_devices.dev_info[dev_index].bdaddr,
                                  PUBLIC_ADDR, 40, 40, 0, 60, 2000 , 2000);

  if (ret != BLE_STATUS_SUCCESS) {
    printf("aci_gap_create_connection() failed: 0x%02x\n", ret);
  }
  else {
    PRINT_DBG("aci_gap_create_connection() OK\n");
  }
}

/**
 * @brief  Services discovery
 * @param  uint8_t The device index
 * @retval None
 */
void Discover_Services(uint8_t dev_index)
{
  uint8_t ret;

  PRINT_DBG(" connection handle 0x%04x\n", saved_devices.dev_info[dev_index].conn_handle);
  ret = aci_gatt_disc_all_primary_services(saved_devices.dev_info[dev_index].conn_handle);

  if (ret != BLE_STATUS_SUCCESS) {
    printf("aci_gatt_disc_all_primary_services() failed: 0x%02x\n", ret);
  }
  else {
    PRINT_DBG("aci_gatt_disc_all_primary_services() OK\n");
  }
}

/**
 * @brief  Discover for each service all characteristics
 * @param  uint8_t The device index
 * @param  uint8_t The service index
 * @retval None
 */
void Discover_Characteristics(uint8_t dev_idx, uint8_t serv_idx)
{
  uint16_t conn_handle  = saved_devices.dev_info[dev_idx].conn_handle;
  uint16_t start_handle = saved_devices.dev_info[dev_idx].serv_info[serv_idx].start_handle;
  uint16_t end_handle   = saved_devices.dev_info[dev_idx].serv_info[serv_idx].end_handle;
  uint8_t  ret;

  ret = aci_gatt_disc_all_char_of_service(conn_handle, start_handle, end_handle);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("aci_gatt_disc_all_char_of_service() failed: 0x%02x\n", ret);
  }
  else {
    PRINT_DBG("aci_gatt_disc_all_char_of_service() OK\n");
  }
}

/**
 * @brief  Update a characteristic property
 * @param  uint8_t The device index
 * @param  uint8_t The service index
 * @param  uint8_t The characteristic index
 * @param  uint8_t The property index
 * @retval None
 */
void Update_Characteristic (uint8_t dev_idx, uint8_t serv_idx, uint8_t char_idx,
                            uint8_t prop_idx)
{
  /* Nel flow automatico viene usato solo prop_idx = 4 (enable notify) */
  if (prop_idx == 4) {
    if (saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].notify) {
      Set_Notifications(dev_idx, serv_idx, char_idx, 0x01);
    } else {
      printf("[GATT] Notify non supportato\r\n");
    }
  }
}

/**
 * @brief  Enable/Disable notifications and indications
 * @param  uint8_t The device index
 * @param  uint8_t The service index
 * @param  uint8_t The characteristic index
 * @param  uint8_t The new status:
 *                 0x00 disable both notifications and indications
 *                 0x01 enable notifications
 *                 0x02 enable indications
 *                 0x03 enable both notifications and indications
 * @param  uint8_t The property index
 * @retval None
 */
void Set_Notifications(uint8_t dev_idx, uint8_t serv_idx, uint8_t char_idx, uint8_t status)
{
  uint16_t connection_handle = saved_devices.dev_info[dev_idx].conn_handle;
  uint8_t  attribute_val_length = 2;
  /**
   * status = 0x00 disable both notifications and indications
   * status = 0x01 enable notifications
   * status = 0x02 enable indications
   * status = 0x03 enable both notifications and indications
   */
  uint8_t attribute_val[] = {status, 0x00};
  uint8_t ret;

  uint16_t attr_handle = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].decl_handle + 2;
  ret = aci_gatt_write_char_desc(connection_handle, attr_handle,
                                 attribute_val_length, attribute_val);
  if (ret != BLE_STATUS_SUCCESS) {
    printf(" Unable to change notification/indication property on device %d (err 0x%02x, 0x%04x - 0x%04x)\r\n",
           dev_idx, ret, connection_handle, attr_handle);
    central_status = SELECT_ANOTHER_CHARACTERISTIC;
  }
}

/**
 * @brief  Save the found device in the struct containing all the found devices
 * @param  addr
 * @param  address type (0x00 Public Device Address, 0x01 Random Device Address)
 * @param  data_length
 * @param  data_value
 * @param  position in the struct
 * @retval None
 */
void Save_Found_Device(tBDAddr addr, uint8_t* addr_type, uint8_t data_length,
                       uint8_t* data_value, uint8_t index)
{
  uint8_t i = 0;

  BLUENRG_memcpy(saved_devices.dev_info[index].bdaddr, addr, 6);
  saved_devices.dev_info[index].addr_type   = *addr_type;
  saved_devices.dev_info[index].name_length = strlen("Unknown");
  BLUENRG_memcpy(saved_devices.dev_info[index].name, "Unknown",
                 (saved_devices.dev_info[index].name_length));

  while (i < data_length)
  {
    /* Advertising data fields: len, type, values */
    /* Check if field is a complete or a short local name */
    if ((data_value[i+1] == AD_TYPE_COMPLETE_LOCAL_NAME) ||
        (data_value[i+1] == AD_TYPE_SHORTENED_LOCAL_NAME))
    {
      saved_devices.dev_info[index].name_length = (data_value[i]-1);
      BLUENRG_memcpy(saved_devices.dev_info[index].name, &data_value[i+2],
                     (saved_devices.dev_info[index].name_length));
      break;
    }
    else
    {
      /* move to next advertising field */
      i += (data_value[i] + 1);
    }
  }

  return;
}

/**
 * @brief  Save the found device in the struct containing all the non connectable
 *         devices
 * @param  addr
 * @retval None
 */
void Save_NonConn_Device(tBDAddr addr)
{
  uint8_t i = non_conn_devices.dev_idx;

  BLUENRG_memcpy(non_conn_devices.bdaddr[i], addr, 6);

  non_conn_devices.dev_num++;
  non_conn_devices.dev_idx++;
}

/**
 * @brief  Search for an already saved device
 * @param  address of the device to search for
 * @retval TRUE if the device has been already saved, FALSE otherwise
 */
uint8_t Is_Device_Saved(tBDAddr addr)
{
  uint8_t i;

  for (i=0; i<saved_devices.dev_num; i++) {
    PRINT_DBG("Saved device %d: %02x:%02x:%02x:%02x:%02x:%02x\n", i,
           saved_devices.dev_info[i].bdaddr[5], saved_devices.dev_info[i].bdaddr[4],
           saved_devices.dev_info[i].bdaddr[3], saved_devices.dev_info[i].bdaddr[2],
           saved_devices.dev_info[i].bdaddr[1], saved_devices.dev_info[i].bdaddr[0]);
    if (BLUENRG_memcmp(addr, saved_devices.dev_info[i].bdaddr, sizeof(saved_devices.dev_info[i].bdaddr)) == 0)
    {
      PRINT_DBG("Current device %02x:%02x:%02x:%02x:%02x:%02x already saved at pos %d\n",
                 addr[5], addr[4], addr[3], addr[2], addr[1], addr[0], i);
      return TRUE;
    }
  }
  PRINT_DBG("Current device %02x:%02x:%02x:%02x:%02x:%02x not yet saved (%d)\n",
         addr[5], addr[4], addr[3], addr[2], addr[1], addr[0],i);
  return FALSE;
}

/**
 * @brief  Search for an already scanned device
 * @param  address of the device to search for
 * @retval TRUE if the device has been already scanned, FALSE otherwise
 */
uint8_t Is_Device_Scanned(tBDAddr addr)
{
  uint8_t i;

  for (i=0; i<non_conn_devices.dev_num; i++) {
    PRINT_DBG("Scanned device %d: %02x:%02x:%02x:%02x:%02x:%02x\n", i,
           non_conn_devices.bdaddr[i][5], non_conn_devices.bdaddr[i][4],
           non_conn_devices.bdaddr[i][3], non_conn_devices.bdaddr[i][2],
           non_conn_devices.bdaddr[i][1], non_conn_devices.bdaddr[i][0]);
    if (BLUENRG_memcmp(addr, non_conn_devices.bdaddr[i], sizeof(non_conn_devices.bdaddr[i])) == 0)
    {
      PRINT_DBG("Current device %02x:%02x:%02x:%02x:%02x:%02x already scanned (at pos %d in the scanned device array)\n",
                 addr[5], addr[4], addr[3], addr[2], addr[1], addr[0], i);
      return TRUE;
    }
  }
  PRINT_DBG("Current device %02x:%02x:%02x:%02x:%02x:%02x not yet scanned (%d)\n",
         addr[5], addr[4], addr[3], addr[2], addr[1], addr[0],i);
  return FALSE;
}

/**
 * @brief  Callback processing the ACI events
 * @note   Inside this function each event must be identified and correctly
 *         parsed
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void APP_UserEvtRx(void *pData)
{
  uint32_t i;

  hci_spi_pckt *hci_pckt = (hci_spi_pckt *)pData;

  if(hci_pckt->type == HCI_EVENT_PKT)
  {
    hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

    if(event_pckt->evt == EVT_LE_META_EVENT)
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;

      for (i = 0; i < (sizeof(hci_le_meta_events_table)/sizeof(hci_le_meta_events_table_type)); i++)
      {
        if (evt->subevent == hci_le_meta_events_table[i].evt_code)
        {
          hci_le_meta_events_table[i].process((void *)evt->data);
        }
      }
    }
    else if(event_pckt->evt == EVT_VENDOR)
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;

      for (i = 0; i < (sizeof(hci_vendor_specific_events_table)/sizeof(hci_vendor_specific_events_table_type)); i++)
      {
        if (blue_evt->ecode == hci_vendor_specific_events_table[i].evt_code)
        {
          hci_vendor_specific_events_table[i].process((void *)blue_evt->data);
        }
      }
    }
    else
    {
      for (i = 0; i < (sizeof(hci_events_table)/sizeof(hci_events_table_type)); i++)
      {
        if (event_pckt->evt == hci_events_table[i].evt_code)
        {
          hci_events_table[i].process((void *)event_pckt->data);
        }
      }
    }
  }
}
