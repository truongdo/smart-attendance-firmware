#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"

// Not declared in the public header in ESP-IDF's NimBLE packaging.
void ble_store_config_init(void);

static const char *TAG = "ble";

// 128-bit UUIDs generated for this project.
static const ble_uuid128_t gatt_svc_uuid =
    BLE_UUID128_INIT(0x7e, 0x2a, 0x3b, 0x4c, 0x5d, 0x6e, 0x7f, 0x80,
                     0x91, 0xa2, 0xb3, 0xc4, 0xd5, 0xe6, 0xf7, 0x08);

// Code characteristic UUID (read/notify).
static const ble_uuid128_t gatt_code_chr_uuid =
    BLE_UUID128_INIT(0x09, 0x18, 0x27, 0x36, 0x45, 0x54, 0x63, 0x72,
                     0x81, 0x90, 0xaf, 0xbe, 0xcd, 0xdc, 0xeb, 0xfa);

// Request characteristic UUID (write-only). Generated for this handshake.
static const ble_uuid128_t gatt_req_chr_uuid =
    // NOTE: BLE_UUID128_INIT expects bytes in little-endian order.
    // This is the byte-reversed form of: 5b7c0d8e-9fa1-b2c3-d4e5-f60718293a4b
    BLE_UUID128_INIT(0x4b, 0x3a, 0x29, 0x18, 0x07, 0xf6, 0xe5, 0xd4,
                     0xc3, 0xb2, 0xa1, 0x9f, 0x8e, 0x0d, 0x7c, 0x5b);

static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t g_code_val_handle = 0;
static uint8_t g_own_addr_type = BLE_OWN_ADDR_PUBLIC;

static uint8_t g_code_value[64] = "hello";
static uint16_t g_code_value_len = 5;

static void ble_app_advertise(void);

static int format_ble_addr(char *out, size_t out_len) {
  uint8_t addr_val[6] = {0};
  int rc = ble_hs_id_copy_addr(g_own_addr_type, addr_val, NULL);
  if (rc != 0) {
    return rc;
  }

  // Uppercase with colons: AA:BB:CC:DD:EE:FF
  int n = snprintf(out, out_len, "%02X:%02X:%02X:%02X:%02X:%02X", addr_val[5], addr_val[4],
                   addr_val[3], addr_val[2], addr_val[1], addr_val[0]);
  return (n > 0 && (size_t)n < out_len) ? 0 : -1;
}

static void ble_notify_code_if_connected(void) {
  if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE || g_code_val_handle == 0) {
    return;
  }

  struct os_mbuf *om = ble_hs_mbuf_from_flat(g_code_value, g_code_value_len);
  if (!om) {
    ESP_LOGW(TAG, "Failed to alloc mbuf for notify");
    return;
  }

  int rc = ble_gatts_notify_custom(g_conn_handle, g_code_val_handle, om);
  if (rc != 0) {
    ESP_LOGW(TAG, "Notify failed; rc=%d", rc);
  } else {
    ESP_LOGI(TAG, "Notified code; len=%u", (unsigned)g_code_value_len);
  }
}

static int gatt_code_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg) {
  (void)attr_handle;
  (void)arg;
  (void)conn_handle;

  switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR: {
      int rc = os_mbuf_append(ctxt->om, g_code_value, g_code_value_len);
      return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    default:
      return BLE_ATT_ERR_UNLIKELY;
  }
}

static int gatt_req_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg) {
  (void)attr_handle;
  (void)arg;

  if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
    return BLE_ATT_ERR_UNLIKELY;
  }

  const uint16_t pkt_len = (uint16_t)OS_MBUF_PKTLEN(ctxt->om);
  if (pkt_len == 0) {
    ESP_LOGI(TAG, "Request write from conn=%u len=0", (unsigned)conn_handle);
    return 0;
  }
  if (pkt_len > 256) {
    ESP_LOGW(TAG, "Write too long; conn=%u len=%u", (unsigned)conn_handle,
             (unsigned)pkt_len);
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  }

  uint8_t buf[256];
  uint16_t len = 0;
  int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len);
  if (rc != 0) {
    ESP_LOGW(TAG, "ble_hs_mbuf_to_flat failed; rc=%d pkt_len=%u", rc,
             (unsigned)pkt_len);
    return BLE_ATT_ERR_UNLIKELY;
  }

  ESP_LOGI(TAG, "Request write from conn=%u len=%u", (unsigned)conn_handle,
           (unsigned)len);
  ESP_LOG_BUFFER_HEXDUMP(TAG, buf, len, ESP_LOG_INFO);

  // Protocol: write ASCII "REQ" to request the device BLE MAC address.
  if (len == 3 && memcmp(buf, "REQ", 3) == 0) {
    char mac[32] = {0};
    rc = format_ble_addr(mac, sizeof(mac));
    if (rc != 0) {
      ESP_LOGW(TAG, "Failed to format BLE addr; rc=%d", rc);
      return BLE_ATT_ERR_UNLIKELY;
    }

    size_t mac_len = strlen(mac);
    if (mac_len > sizeof(g_code_value)) {
      return BLE_ATT_ERR_UNLIKELY;
    }

    memcpy(g_code_value, mac, mac_len);
    g_code_value_len = (uint16_t)mac_len;

    ble_notify_code_if_connected();
    return 0;
  }

  return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svc_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = &gatt_req_chr_uuid.u,
                    .access_cb = gatt_req_chr_access_cb,
                    .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC,
                },
                {
                    .uuid = &gatt_code_chr_uuid.u,
                    .access_cb = gatt_code_chr_access_cb,
                    .val_handle = &g_code_val_handle,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ_ENC |
                             BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC,
                },
                {0},
            },
    },
    {0},
};

static void ble_on_sync(void) {
  // Ensure we have a valid public / random address and then start advertising.
  int rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
  if (rc != 0) {
    ESP_LOGW(TAG, "ble_hs_id_infer_auto failed; rc=%d", rc);
    g_own_addr_type = BLE_OWN_ADDR_PUBLIC;
  }
  ble_app_advertise();
}

static void ble_on_reset(int reason) {
  ESP_LOGE(TAG, "Resetting state; reason=%d", reason);
}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
  (void)arg;

  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status == 0) {
        g_conn_handle = event->connect.conn_handle;
        ESP_LOGI(TAG, "Connected; conn_handle=%u", (unsigned)g_conn_handle);
      } else {
        ESP_LOGW(TAG, "Connect failed; status=%d", event->connect.status);
        ble_app_advertise();
      }
      return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
      if (event->enc_change.status == 0) {
        struct ble_gap_conn_desc desc;
        int rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
        if (rc == 0) {
          ESP_LOGI(TAG, "Encryption changed; encrypted=%d authenticated=%d bonded=%d",
                   desc.sec_state.encrypted, desc.sec_state.authenticated, desc.sec_state.bonded);
        } else {
          ESP_LOGI(TAG, "Encryption changed; conn_handle=%u", (unsigned)event->enc_change.conn_handle);
        }
      } else {
        ESP_LOGW(TAG, "Encryption change failed; status=%d", event->enc_change.status);
      }
      return 0;

    case BLE_GAP_EVENT_DISCONNECT:
      ESP_LOGI(TAG, "Disconnected; reason=%d", event->disconnect.reason);
      g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
      ble_app_advertise();
      return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
      ESP_LOGI(TAG, "Advertising complete; restarting");
      ble_app_advertise();
      return 0;

    default:
      return 0;
  }
}

static void ble_app_advertise(void) {
  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields fields;
  struct ble_hs_adv_fields rsp_fields;

  memset(&fields, 0, sizeof(fields));
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

  // Keep both ADV and scan response within 31 bytes.
  // Put the 128-bit UUID in ADV and the full name in scan response.
  const char *name = ble_svc_gap_device_name();
  fields.uuids128 = (ble_uuid128_t *)&gatt_svc_uuid;
  fields.num_uuids128 = 1;
  fields.uuids128_is_complete = 1;

  int rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_adv_set_fields failed; rc=%d", rc);
    return;
  }

  memset(&rsp_fields, 0, sizeof(rsp_fields));
  rsp_fields.name = (uint8_t *)name;
  rsp_fields.name_len = (uint8_t)strlen(name);
  rsp_fields.name_is_complete = 1;

  rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed; rc=%d", rc);
    return;
  }

  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

  rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                         ble_gap_event_cb, NULL);
  if (rc != 0) {
    ESP_LOGW(TAG, "ble_gap_adv_start failed; rc=%d (own_addr_type=%u)", rc,
             (unsigned)g_own_addr_type);
  }

  if (rc == 0) {
    ESP_LOGI(TAG, "Advertising as \"%s\"", name);
  } else {
    ESP_LOGE(TAG, "ble_gap_adv_start failed; rc=%d", rc);
  }
}

static void host_task(void *param) {
  (void)param;
  nimble_port_run();
  nimble_port_freertos_deinit();
}

void app_main(void) {
  ESP_LOGI(TAG, "Boot; reset_reason=%d", (int)esp_reset_reason());

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  } else {
    ESP_ERROR_CHECK(err);
  }

  ESP_ERROR_CHECK(nimble_port_init());

  ble_hs_cfg.reset_cb = ble_on_reset;
  ble_hs_cfg.sync_cb = ble_on_sync;

  // Security Manager: Just Works pairing with bonding and link encryption.
  // UX goal: minimize user interaction; trade-off: not MITM-resistant.
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_mitm = 0;
  ble_hs_cfg.sm_sc = 1;
  ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
  ble_store_config_init();

  ble_svc_gap_init();
  ble_svc_gatt_init();

  ble_svc_gap_device_name_set("SmartAttendance");

  int rc = ble_gatts_count_cfg(gatt_svcs);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gatts_count_cfg failed; rc=%d", rc);
    return;
  }
  rc = ble_gatts_add_svcs(gatt_svcs);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gatts_add_svcs failed; rc=%d", rc);
    return;
  }

  nimble_port_freertos_init(host_task);
}

