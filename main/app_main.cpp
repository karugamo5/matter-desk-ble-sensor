#include <esp_log.h>
#include <esp_matter.h>
#include <esp_timer.h>
#include <nvs_flash.h>
#include <setup_payload/OnboardingCodesUtil.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <platform/CHIPDeviceLayer.h>

#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <host/ble_hs_adv.h>

#include <mbedtls/aes.h>
#include <mbedtls/base64.h>

#include <cstdio>
#include <cstring>
#include <cmath>
#include <limits>

using namespace chip;
using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::endpoint;

namespace
{

    static const char *kTag = "desk_ble_sensor";
    static constexpr const char *kAppVersion = "v1.0";

    static constexpr uint32_t CLUSTER_ID = OccupancySensing::Id;
    static constexpr uint32_t ATTRIBUTE_ID = OccupancySensing::Attributes::Occupancy::Id;

    // ===== Tunable parameters =====
    // Target device name (for logs)
    static constexpr const char *kTargetName = "Apple Watch";
    // Optional Apple Watch IRK (Base64). Leave empty for BLE-only mode.
    static constexpr const char *kTargetIrkBase64 = "";
    // Presence timeout after last valid detection (ms)
    static constexpr uint32_t kPresenceTimeoutMs = 10000;
    // Scan duration per cycle (ms). Optimized for ~200ms beacon interval.
    static constexpr uint32_t kScanDurationMs = 600;
    // Gap before starting next scan cycle (ms)
    static constexpr uint32_t kScanRetryMs = 100;
    // Consider scan stuck if not completed within this timeout (ms)
    static constexpr uint32_t kScanStuckTimeoutMs = 12000;
    // Maximum distance considered "in-room" for analysis (m).
    // This is separate from presence threshold and filters far-away BLE sources.
    static constexpr float kRoomScopeMaxDistanceM = 7.0f;
    // Presence decision threshold (m)
    static constexpr float kPresenceDistanceThresholdM = 1.0f;
    // Distance EMA factor (0.0-1.0). Higher = faster response to new RSSI.
    static constexpr float kDistanceEmaAlpha = 0.35f;
    // Required consecutive near hits before turning present=true
    static constexpr uint8_t kNearHitRequired = 2;
    // Enable debug logs and debug LED
    static constexpr bool kDebugMode = false;
    // XIAO ESP32S3 built-in LED (D10 = GPIO21, active-low)
    static constexpr gpio_num_t kBuiltinLedGpio = GPIO_NUM_21;
    static constexpr bool kDebugLedActiveLow = true;
    // Health telemetry log interval for long-run operation (ms)
    static constexpr uint32_t kHealthLogIntervalMs = 60000;

    static uint16_t gSensorEndpointId = 0;

    static bool gCommissioned = false;
    static bool gScanRunning = false;
    static uint32_t gLastScanAttemptMs = 0;

    static bool gIrkValid = false;
    static bool gBleOnlyMode = true;
    static uint8_t gIrk[16] = {0};

    static bool gPresent = false;
    static uint32_t gLastSeenMs = 0;
    static uint32_t gLastStatusMs = 0;
    static bool gDebugLedReady = false;
    static bool gDistanceEmaValid = false;
    static float gDistanceEmaM = 0.0f;
    static uint8_t gNearHitCount = 0;

    static uint32_t gLastHealthLogMs = 0;
    static uint32_t gMinFreeHeapBytes = std::numeric_limits<uint32_t>::max();
    static UBaseType_t gRuntimeTaskMinStackWords = std::numeric_limits<UBaseType_t>::max();
    static uint32_t gScanStartOkCount = 0;
    static uint32_t gScanStartFailCount = 0;
    static uint32_t gScanCompleteCount = 0;
    static uint32_t gTargetMatchCount = 0;
    static uint32_t gOutOfRoomFilteredCount = 0;
    static uint32_t gOccupancyUpdateCount = 0;
    static uint32_t gMatterUpdateFailCount = 0;

    enum class NearbyState : uint8_t
    {
        kNone = 0,
        kActive,
        kLocked,
        kOther,
    };

    struct ScanCandidate
    {
        bool valid = false;
        float distanceRawM = 0.0f;
        float distanceEmaM = 0.0f;
        int rssi = -127;
        NearbyState nearbyState = NearbyState::kNone;
        bool irkMatched = false;
    };

    static ScanCandidate gBestCandidateThisScan = {};

    static uint32_t now_ms()
    {
        return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
    }

    static void debug_led_set(bool on)
    {
        if (!kDebugMode || !gDebugLedReady)
        {
            return;
        }
        int level = on ? 1 : 0;
        if (kDebugLedActiveLow)
        {
            level = on ? 0 : 1;
        }
        gpio_set_level(kBuiltinLedGpio, level);
    }

    static void debug_led_update()
    {
        // LED on while presence is true.
        debug_led_set(gPresent);
    }

    static float estimate_distance_m_from_rssi(int rssi)
    {
        // Simple RSSI-based distance estimate.
        // kTxPowerAt1m: expected RSSI at 1m (environment dependent).
        // kPathLossN: path-loss factor (typ. indoor 1.8-3.0).
        static constexpr float kTxPowerAt1m = -58.0f;
        static constexpr float kPathLossN = 2.2f;
        return powf(10.0f, (kTxPowerAt1m - static_cast<float>(rssi)) / (10.0f * kPathLossN));
    }

    static float smooth_distance_m(float distance_m)
    {
        if (!gDistanceEmaValid)
        {
            gDistanceEmaM = distance_m;
            gDistanceEmaValid = true;
            return gDistanceEmaM;
        }
        gDistanceEmaM = (kDistanceEmaAlpha * distance_m) + ((1.0f - kDistanceEmaAlpha) * gDistanceEmaM);
        return gDistanceEmaM;
    }

    static const char *nearby_state_to_str(NearbyState state)
    {
        switch (state)
        {
        case NearbyState::kActive:
            return "active";
        case NearbyState::kLocked:
            return "locked";
        case NearbyState::kOther:
            return "other";
        default:
            return "none";
        }
    }

    static void debug_log_nearby_detail(const ble_gap_event *event, const uint8_t *mfg, uint8_t mfg_len,
                                        NearbyState nearby_state, bool irk_matched)
    {
        if (!kDebugMode || !event || !mfg || mfg_len < 2)
        {
            return;
        }

        const ble_addr_t &a = event->disc.addr;
        const uint16_t company_id = static_cast<uint16_t>(mfg[0]) | (static_cast<uint16_t>(mfg[1]) << 8);
        if (company_id != 0x004C)
        {
            return;
        }
        bool nearby_matched = (nearby_state == NearbyState::kActive || nearby_state == NearbyState::kLocked);
        if (!(nearby_matched || irk_matched))
        {
            // Log only Apple Watch detections.
            return;
        }
        const float distance_m = estimate_distance_m_from_rssi(event->disc.rssi);

        ESP_LOGI(kTag,
                 "[nearby] target=%s rssi=%d distanceM=%.2f addrType=%u addr=%02X:%02X:%02X:%02X:%02X:%02X company=0x%04X len=%u matchedNearby=%u matchedIrk=%u nearbyState=%s active=%u locked=%u",
                 kTargetName, event->disc.rssi, distance_m, static_cast<unsigned>(a.type),
                 a.val[5], a.val[4], a.val[3], a.val[2], a.val[1], a.val[0],
                 company_id, static_cast<unsigned>(mfg_len),
                 nearby_matched ? 1 : 0, irk_matched ? 1 : 0,
                 nearby_state_to_str(nearby_state),
                 nearby_state == NearbyState::kActive ? 1 : 0,
                 nearby_state == NearbyState::kLocked ? 1 : 0);

        size_t pos = 2;
        while (pos + 1 < mfg_len)
        {
            uint8_t type = mfg[pos];
            uint8_t len = mfg[pos + 1];
            size_t value_pos = pos + 2;
            if (value_pos + len > mfg_len)
            {
                ESP_LOGI(kTag, "[nearby] tlv malformed type=0x%02X len=%u", type, static_cast<unsigned>(len));
                break;
            }

            if (type == 0x10)
            {
                uint8_t status = (len > 0) ? mfg[value_pos] : 0;
                uint8_t data_flags = (len > 1) ? mfg[value_pos + 1] : 0;
                NearbyState state = NearbyState::kOther;
                if (data_flags == 0x98)
                {
                    state = NearbyState::kActive;
                }
                else if (data_flags == 0x18)
                {
                    state = NearbyState::kLocked;
                }
                ESP_LOGI(kTag, "[nearby] tlv type=0x10 len=%u status=0x%02X dataFlags=0x%02X state=%s",
                         static_cast<unsigned>(len), status, data_flags, nearby_state_to_str(state));
            }
            else
            {
                ESP_LOGI(kTag, "[nearby] tlv type=0x%02X len=%u", type, static_cast<unsigned>(len));
            }
            pos = value_pos + len;
        }
    }

    static bool parse_base64_irk_16(const char *in, uint8_t out[16])
    {
        if (!in || !out)
        {
            return false;
        }
        const size_t in_len = strlen(in);
        if (in_len == 0)
        {
            return false;
        }
        uint8_t decoded[32] = {0};
        size_t out_len = 0;
        int rc = mbedtls_base64_decode(decoded, sizeof(decoded), &out_len,
                                       reinterpret_cast<const unsigned char *>(in), in_len);
        if (rc != 0 || out_len != 16)
        {
            return false;
        }
        memcpy(out, decoded, 16);
        return true;
    }

    static void aes_ecb_128(const uint8_t key[16], const uint8_t in[16], uint8_t out[16])
    {
        mbedtls_aes_context ctx;
        mbedtls_aes_init(&ctx);
        mbedtls_aes_setkey_enc(&ctx, key, 128);
        mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, in, out);
        mbedtls_aes_free(&ctx);
    }

    static bool ah_match_variant(const uint8_t key[16], const uint8_t hash[3], const uint8_t prand[3], bool compare_head)
    {
        if ((prand[0] & 0xC0) != 0x40)
        {
            return false;
        }

        uint8_t r_block[16] = {0};
        uint8_t out[16] = {0};
        r_block[13] = prand[0];
        r_block[14] = prand[1];
        r_block[15] = prand[2];

        aes_ecb_128(key, r_block, out);
        if (compare_head)
        {
            return out[0] == hash[0] && out[1] == hash[1] && out[2] == hash[2];
        }
        return out[13] == hash[0] && out[14] == hash[1] && out[15] == hash[2];
    }

    static bool resolve_rpa_by_irk(const uint8_t addr_be[6], const uint8_t irk[16])
    {
        uint8_t hash_a[3] = {addr_be[0], addr_be[1], addr_be[2]};
        uint8_t prand_a[3] = {addr_be[3], addr_be[4], addr_be[5]};
        uint8_t hash_b[3] = {addr_be[3], addr_be[4], addr_be[5]};
        uint8_t prand_b[3] = {addr_be[0], addr_be[1], addr_be[2]};

        uint8_t irk_rev[16];
        for (int i = 0; i < 16; ++i)
        {
            irk_rev[i] = irk[15 - i];
        }

        if (ah_match_variant(irk, hash_a, prand_a, false))
            return true;
        if (ah_match_variant(irk, hash_a, prand_a, true))
            return true;
        if (ah_match_variant(irk, hash_b, prand_b, false))
            return true;
        if (ah_match_variant(irk, hash_b, prand_b, true))
            return true;
        if (ah_match_variant(irk_rev, hash_a, prand_a, false))
            return true;
        if (ah_match_variant(irk_rev, hash_a, prand_a, true))
            return true;
        if (ah_match_variant(irk_rev, hash_b, prand_b, false))
            return true;
        if (ah_match_variant(irk_rev, hash_b, prand_b, true))
            return true;
        return false;
    }

    static NearbyState parse_apple_watch_nearby_state(const uint8_t *mfg, uint8_t mfg_len)
    {
        if (!mfg || mfg_len < 4)
        {
            return NearbyState::kNone;
        }

        const uint16_t company_id = static_cast<uint16_t>(mfg[0]) | (static_cast<uint16_t>(mfg[1]) << 8);
        if (company_id != 0x004C)
        {
            return NearbyState::kNone;
        }

        // Apple manufacturer payload TLV; Nearby info = type 0x10.
        size_t pos = 2;
        while (pos + 1 < mfg_len)
        {
            uint8_t type = mfg[pos];
            uint8_t len = mfg[pos + 1];
            size_t value_pos = pos + 2;
            if (value_pos + len > mfg_len)
            {
                break;
            }

            if (type == 0x10 && len >= 2)
            {
                uint8_t data_flags = mfg[value_pos + 1];
                if (data_flags == 0x98)
                {
                    return NearbyState::kActive;
                }
                if (data_flags == 0x18)
                {
                    return NearbyState::kLocked;
                }
                return NearbyState::kOther;
            }
            pos = value_pos + len;
        }

        return NearbyState::kNone;
    }

    static void set_presence_seen(const char *source)
    {
        gLastSeenMs = now_ms();
        if (gNearHitCount < 255)
        {
            gNearHitCount++;
        }
        if (!gPresent)
        {
            if (gNearHitCount >= kNearHitRequired)
            {
                gPresent = true;
                debug_led_update();
                ESP_LOGI(kTag, "[ble] present=1 source=%s nearHits=%u", source, gNearHitCount);
            }
        }
        else if (kDebugMode)
        {
            ESP_LOGI(kTag, "[ble] keep present source=%s nearHits=%u", source, gNearHitCount);
        }
    }

    static void evaluate_best_candidate_realtime()
    {
        if (!gBestCandidateThisScan.valid)
        {
            return;
        }
        if (gBestCandidateThisScan.distanceEmaM <= kPresenceDistanceThresholdM &&
            gBestCandidateThisScan.nearbyState == NearbyState::kActive)
        {
            set_presence_seen("NEAREST_ACTIVE<=1M_RT");
            return;
        }

        gNearHitCount = 0;
        if (kDebugMode)
        {
            ESP_LOGI(kTag,
                     "[ble] realtime nearest not eligible emaM=%.2f thresholdM=%.2f state=%s",
                     gBestCandidateThisScan.distanceEmaM,
                     kPresenceDistanceThresholdM,
                     nearby_state_to_str(gBestCandidateThisScan.nearbyState));
        }
    }

    static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
    {
        (void)arg;
        if (!event)
        {
            return 0;
        }

        switch (event->type)
        {
        case BLE_GAP_EVENT_DISC:
        {
            struct ble_hs_adv_fields fields;
            memset(&fields, 0, sizeof(fields));

            int rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
            if (rc != 0)
            {
                return 0;
            }

            NearbyState nearby_state = NearbyState::kNone;
            if (fields.mfg_data && fields.mfg_data_len > 0)
            {
                nearby_state = parse_apple_watch_nearby_state(fields.mfg_data, fields.mfg_data_len);
            }

            bool irk_matched = false;
            if (gIrkValid)
            {
                // Convert NimBLE LE addr to big-endian byte array for IRK check.
                uint8_t addr_be[6] = {
                    event->disc.addr.val[5], event->disc.addr.val[4], event->disc.addr.val[3],
                    event->disc.addr.val[2], event->disc.addr.val[1], event->disc.addr.val[0]};
                irk_matched = resolve_rpa_by_irk(addr_be, gIrk);
            }

            bool nearby_target = (nearby_state == NearbyState::kActive || nearby_state == NearbyState::kLocked);
            bool is_target_matched = (nearby_target || irk_matched);
            if (is_target_matched)
            {
                const float distance_m = estimate_distance_m_from_rssi(event->disc.rssi);
                const float distance_ema_m = smooth_distance_m(distance_m);
                if (distance_ema_m > kRoomScopeMaxDistanceM)
                {
                    gOutOfRoomFilteredCount++;
                    if (kDebugMode)
                    {
                        ESP_LOGI(kTag, "[ble] out-of-room filtered rawM=%.2f emaM=%.2f roomMaxM=%.2f",
                                 distance_m, distance_ema_m, kRoomScopeMaxDistanceM);
                    }
                    return 0;
                }

                gTargetMatchCount++;
                if (!gBestCandidateThisScan.valid || distance_ema_m < gBestCandidateThisScan.distanceEmaM)
                {
                    gBestCandidateThisScan.valid = true;
                    gBestCandidateThisScan.distanceRawM = distance_m;
                    gBestCandidateThisScan.distanceEmaM = distance_ema_m;
                    gBestCandidateThisScan.rssi = event->disc.rssi;
                    gBestCandidateThisScan.nearbyState = nearby_state;
                    gBestCandidateThisScan.irkMatched = irk_matched;
                    evaluate_best_candidate_realtime();
                }
            }
            if (fields.mfg_data && fields.mfg_data_len > 0)
            {
                debug_log_nearby_detail(event, fields.mfg_data, fields.mfg_data_len, nearby_state, irk_matched);
            }
            return 0;
        }

        case BLE_GAP_EVENT_DISC_COMPLETE:
            if (gBestCandidateThisScan.valid)
            {
                ESP_LOGI(kTag,
                         "[ble] nearest candidate rssi=%d rawM=%.2f emaM=%.2f nearbyState=%s irkMatched=%u",
                         gBestCandidateThisScan.rssi,
                         gBestCandidateThisScan.distanceRawM,
                         gBestCandidateThisScan.distanceEmaM,
                         nearby_state_to_str(gBestCandidateThisScan.nearbyState),
                         gBestCandidateThisScan.irkMatched ? 1 : 0);
            }
            gBestCandidateThisScan = {};
            gScanRunning = false;
            gScanCompleteCount++;
            ESP_LOGI(kTag, "[ble] scan complete reason=%d", event->disc_complete.reason);
            return 0;

        default:
            return 0;
        }
    }

    static bool start_watch_scan()
    {
        gBestCandidateThisScan = {};
        uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
        int rc = ble_hs_id_infer_auto(0, &own_addr_type);
        if (rc != 0)
        {
            ESP_LOGW(kTag, "[ble] ble_hs_id_infer_auto failed rc=%d", rc);
            return false;
        }

        ble_gap_disc_params disc_params;
        memset(&disc_params, 0, sizeof(disc_params));
        disc_params.passive = 1;
        disc_params.filter_duplicates = 1;
        disc_params.itvl = 0x30;
        disc_params.window = 0x30;

        rc = ble_gap_disc(own_addr_type, static_cast<int32_t>(kScanDurationMs), &disc_params, ble_gap_event_cb, nullptr);
        if (rc == 0)
        {
            gScanRunning = true;
            gScanStartOkCount++;
            ESP_LOGI(kTag, "[ble] scan started");
            return true;
        }

        gScanStartFailCount++;
        ESP_LOGW(kTag, "[ble] scan start failed rc=%d", rc);
        return false;
    }

    static void stop_watch_scan()
    {
        if (!gScanRunning)
        {
            return;
        }
        int rc = ble_gap_disc_cancel();
        if (rc != 0)
        {
            ESP_LOGW(kTag, "[ble] scan stop failed rc=%d", rc);
        }
        gScanRunning = false;
    }

    static bool is_commissioned()
    {
        if (lock::chip_stack_lock(portMAX_DELAY) != lock::SUCCESS)
        {
            ESP_LOGE(kTag, "chip_stack_lock failed in is_commissioned");
            return gCommissioned;
        }
        bool commissioned = chip::DeviceLayer::ConfigurationMgr().IsFullyProvisioned();
        lock::chip_stack_unlock();
        return commissioned;
    }

    static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id,
                                           uint8_t effect_id, uint8_t effect_variant, void *priv_data)
    {
        (void)type;
        (void)endpoint_id;
        (void)effect_id;
        (void)effect_variant;
        (void)priv_data;
        return ESP_OK;
    }

    static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id,
                                             uint32_t cluster_id, uint32_t attribute_id,
                                             esp_matter_attr_val_t *val, void *priv_data)
    {
        (void)type;
        (void)endpoint_id;
        (void)cluster_id;
        (void)attribute_id;
        (void)val;
        (void)priv_data;
        return ESP_OK;
    }

    static void app_device_event_cb(const ChipDeviceEvent *event, intptr_t arg)
    {
        (void)arg;
        if (!event)
        {
            return;
        }

        switch (event->Type)
        {
        case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
            ESP_LOGI(kTag, "[matter] commissioning started -> stop BLE watch scan");
            stop_watch_scan();
            break;

        case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
            ESP_LOGI(kTag, "[matter] commissioning complete");
            break;

        case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
            ESP_LOGI(kTag, "[matter] fabric committed");
            break;

        case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
            ESP_LOGI(kTag, "[matter] fabric removed");
            break;

        default:
            break;
        }
    }

    static bool update_occupancy(bool occupied)
    {
        esp_matter_attr_val_t val = esp_matter_uint8(occupied ? 1 : 0);

        if (lock::chip_stack_lock(portMAX_DELAY) != lock::SUCCESS)
        {
            ESP_LOGE(kTag, "chip_stack_lock failed");
            gMatterUpdateFailCount++;
            return false;
        }
        esp_err_t err = attribute::update(gSensorEndpointId, CLUSTER_ID, ATTRIBUTE_ID, &val);
        lock::chip_stack_unlock();
        if (err != ESP_OK)
        {
            gMatterUpdateFailCount++;
            ESP_LOGW(kTag, "[matter] occupancy update failed err=0x%x", static_cast<unsigned>(err));
            return false;
        }
        gOccupancyUpdateCount++;
        return true;
    }

    static void runtime_task(void *arg)
    {
        (void)arg;

        bool last_present = false;
        (void)update_occupancy(false);

        while (true)
        {
            uint32_t now = now_ms();
            bool commissioned = is_commissioned();

            if (commissioned != gCommissioned)
            {
                gCommissioned = commissioned;
                ESP_LOGI(kTag, "[matter] commissioned=%s", commissioned ? "true" : "false");
                if (!commissioned)
                {
                    stop_watch_scan();
                    gPresent = false;
                    gNearHitCount = 0;
                    gDistanceEmaValid = false;
                    debug_led_update();
                    update_occupancy(false);
                }
            }

            if (gCommissioned)
            {
                if (!gScanRunning && (now - gLastScanAttemptMs) >= kScanRetryMs)
                {
                    gLastScanAttemptMs = now;
                    (void)start_watch_scan();
                }
                if (gScanRunning && (now - gLastScanAttemptMs) > kScanStuckTimeoutMs)
                {
                    ESP_LOGW(kTag, "[ble] scan stuck detected -> force cancel");
                    stop_watch_scan();
                }
            }

            bool present_now = gPresent;
            if (present_now && (now - gLastSeenMs) > kPresenceTimeoutMs)
            {
                present_now = false;
                gPresent = false;
                gNearHitCount = 0;
                gDistanceEmaValid = false;
                debug_led_update();
                ESP_LOGI(kTag, "[ble] present timeout -> 0");
            }

            if (present_now != last_present)
            {
                if (update_occupancy(present_now))
                {
                    last_present = present_now;
                    ESP_LOGI(kTag, "[matter] occupancy=%u", present_now ? 1 : 0);
                }
            }

            uint32_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
            if (free_heap < gMinFreeHeapBytes)
            {
                gMinFreeHeapBytes = free_heap;
            }
            UBaseType_t stack_hw = uxTaskGetStackHighWaterMark(nullptr);
            if (stack_hw < gRuntimeTaskMinStackWords)
            {
                gRuntimeTaskMinStackWords = stack_hw;
            }

            if ((now - gLastStatusMs) > 5000)
            {
                gLastStatusMs = now;
                uint32_t ago = (gLastSeenMs == 0) ? 0 : (now - gLastSeenMs);
                ESP_LOGI(kTag, "[status] commissioned=%s scan=%s present=%s lastSeenAgoMs=%lu target=%s mode=%s",
                         gCommissioned ? "true" : "false",
                         gScanRunning ? "on" : "off",
                         gPresent ? "true" : "false",
                         static_cast<unsigned long>(ago), kTargetName,
                         gBleOnlyMode ? "BLE_ONLY" : "BLE_PLUS_IRK");
                if (kDebugMode)
                {
                    ESP_LOGI(kTag, "[status] nearHits=%u emaValid=%u emaDistanceM=%.2f presenceThresholdM=%.2f roomMaxM=%.2f",
                             gNearHitCount, gDistanceEmaValid ? 1 : 0, gDistanceEmaM,
                             kPresenceDistanceThresholdM, kRoomScopeMaxDistanceM);
                }
            }

            if ((now - gLastHealthLogMs) > kHealthLogIntervalMs)
            {
                gLastHealthLogMs = now;
                ESP_LOGI(kTag,
                         "[health] heapNow=%lu heapMin=%lu stackMinWords=%lu scanOk=%lu scanFail=%lu scanComplete=%lu targetMatch=%lu outOfRoomFiltered=%lu occUpdateOk=%lu occUpdateFail=%lu",
                         static_cast<unsigned long>(free_heap),
                         static_cast<unsigned long>(gMinFreeHeapBytes),
                         static_cast<unsigned long>(gRuntimeTaskMinStackWords),
                         static_cast<unsigned long>(gScanStartOkCount),
                         static_cast<unsigned long>(gScanStartFailCount),
                         static_cast<unsigned long>(gScanCompleteCount),
                         static_cast<unsigned long>(gTargetMatchCount),
                         static_cast<unsigned long>(gOutOfRoomFilteredCount),
                         static_cast<unsigned long>(gOccupancyUpdateCount),
                         static_cast<unsigned long>(gMatterUpdateFailCount));
            }

            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }

} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(kTag, "[init] appVersion=%s", kAppVersion);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    if (kDebugMode)
    {
        gpio_config_t led_cfg = {};
        led_cfg.pin_bit_mask = (1ULL << static_cast<uint64_t>(kBuiltinLedGpio));
        led_cfg.mode = GPIO_MODE_OUTPUT;
        led_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        led_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        led_cfg.intr_type = GPIO_INTR_DISABLE;
        if (gpio_config(&led_cfg) == ESP_OK)
        {
            gDebugLedReady = true;
            debug_led_set(false);
            ESP_LOGI(kTag, "[debug] mode=on builtinLedGpio=%d activeLow=%u",
                     static_cast<int>(kBuiltinLedGpio), kDebugLedActiveLow ? 1 : 0);
        }
        else
        {
            ESP_LOGW(kTag, "[debug] builtin led init failed gpio=%d", static_cast<int>(kBuiltinLedGpio));
        }
    }

    const size_t irk_cfg_len = strlen(kTargetIrkBase64);
    if (irk_cfg_len == 0)
    {
        gIrkValid = false;
        gBleOnlyMode = true;
        ESP_LOGI(kTag, "[init] target=%s mode=BLE_ONLY (IRK not configured)", kTargetName);
    }
    else
    {
        gIrkValid = parse_base64_irk_16(kTargetIrkBase64, gIrk);
        gBleOnlyMode = !gIrkValid;
        if (gIrkValid)
        {
            ESP_LOGI(kTag, "[init] target=%s mode=BLE_PLUS_IRK", kTargetName);
        }
        else
        {
            ESP_LOGW(kTag, "[init] target=%s IRK parse failed -> mode=BLE_ONLY", kTargetName);
        }
    }

    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    if (!node)
    {
        ESP_LOGE(kTag, "node::create failed");
        return;
    }

    occupancy_sensor::config_t sensor_config;
    sensor_config.occupancy_sensing.occupancy = 0;
    sensor_config.occupancy_sensing.occupancy_sensor_type =
        static_cast<uint8_t>(OccupancySensing::OccupancySensorTypeEnum::kPir);
    sensor_config.occupancy_sensing.occupancy_sensor_type_bitmap =
        static_cast<uint8_t>(OccupancySensing::OccupancySensorTypeBitmap::kPir);
    sensor_config.occupancy_sensing.feature_flags =
        static_cast<uint32_t>(OccupancySensing::Feature::kPassiveInfrared);

    endpoint_t *endpoint = occupancy_sensor::create(node, &sensor_config, ENDPOINT_FLAG_NONE, nullptr);
    if (!endpoint)
    {
        ESP_LOGE(kTag, "occupancy_sensor::create failed");
        return;
    }

    gSensorEndpointId = endpoint::get_id(endpoint);

    ESP_ERROR_CHECK(esp_matter::start(app_device_event_cb));

    ESP_LOGI(kTag, "[init] Matter occupancy sensor ready endpoint=%u", gSensorEndpointId);
    PrintOnboardingCodes(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));

    gCommissioned = is_commissioned();
    if (!gCommissioned)
    {
        ESP_LOGI(kTag, "[matter] waiting for commissioning, BLE watch scan disabled");
    }

    xTaskCreate(runtime_task, "runtime_task", 6144, nullptr, 5, nullptr);
}
