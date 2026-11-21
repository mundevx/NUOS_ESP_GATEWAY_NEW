#include "zigbee_device_classifier.h"
#include <string.h>
#include <ctype.h>

static bool has_cluster(const uint16_t *clusters, uint8_t count, uint16_t id)
{
    if (clusters == NULL) {
        return false;
    }
    if (count > 12) {
        count = 12;
    }
    for (uint8_t i = 0; i < count; ++i) {
        if (clusters[i] == id) {
            return true;
        }
    }
    return false;
}

static bool text_has(const char *text, const char *needle)
{
    if (text == NULL || needle == NULL || needle[0] == '\0') {
        return false;
    }

    const size_t nlen = strlen(needle);
    for (const char *p = text; *p; ++p) {
        size_t i = 0;
        while (i < nlen && p[i]) {
            if (toupper((unsigned char)p[i]) !=
                toupper((unsigned char)needle[i])) {
                break;
            }
            ++i;
        }
        if (i == nlen) {
            return true;
        }
    }
    return false;
}

static bool tuya_identity_is(const char *manufacturer,
                             const char *model,
                             const char *mfr,
                             const char *mdl)
{
    return manufacturer != NULL && model != NULL &&
           strcmp(manufacturer, mfr) == 0 &&
           strcmp(model, mdl) == 0;
}

static bool tuya_is_any(const char *manufacturer,
                        const char *model,
                        const char *mfr,
                        const char *mdl)
{
    return tuya_identity_is(manufacturer, model, mfr, mdl);
}

/*
 * Tuya product identities taken only from the supplied NUOS product source.
 *
 * IMPORTANT:
 * - Manufacturer + model are used together. Model alone is intentionally
 *   not sufficient because several TSxxxx models are reused by products.
 * - Cluster membership still wins for endpoint-level capability reporting.
 * - No Home Assistant / NUOS manufacturer identity is used here.
 */
static const char *tuya_product_hint(const char *manufacturer,
                                     const char *model)
{
    if (tuya_is_any(manufacturer, model, "_TZ3000_ydfvmon7", "TS0014")) return "4gang_onoff_or_group_dali";
    if (tuya_is_any(manufacturer, model, "_TZ3000_irptc32e", "TS0012")) return "2gang_onoff";
    if (tuya_is_any(manufacturer, model, "_TZ3000_vmmaf6q9", "TS0012")) return "2ch_curtain";
    if (tuya_is_any(manufacturer, model, "_TZ3002_kuzmsnrv", "TS130F")) return "curtain";
    if (tuya_is_any(manufacturer, model, "_TZ3000_ipumjqls", "TS130F")) return "curtain";
    if (tuya_is_any(manufacturer, model, "_TZ3000_fcfixupo", "TS130F")) return "curtain";
    if (tuya_is_any(manufacturer, model, "_TZ3000_pn01nze7", "TS0011")) return "curtain";
    if (tuya_is_any(manufacturer, model, "_TYZB01_60jbfyqy", "TS110F")) return "dimmer";

    if (tuya_is_any(manufacturer, model, "_TZE284_ojdbar5r", "TS0601")) return "light_fan";
    if (tuya_is_any(manufacturer, model, "_TZ3210_lzqq3u4r", "TS0501")) return "light_fan";
    if (tuya_is_any(manufacturer, model, "_TZ3000_bzcv6psl", "TS1101")) return "ir_fan_custom";
    if (tuya_is_any(manufacturer, model, "_TZ3000_cftyjrak", "TS1101")) return "ir_custom";
    if (tuya_is_any(manufacturer, model, "_TZE200_n4qc9fg7", "TS0601")) return "thermostat";

    if (tuya_is_any(manufacturer, model, "_TZ3000_m8ytq7be", "TS0202")) return "motion";
    if (tuya_is_any(manufacturer, model, "_TZ3000_2qzlhsne", "TS0202")) return "contact";
    if (tuya_is_any(manufacturer, model, "_TZ3000_xkl3yq9f", "TS0202")) return "gas";
    if (tuya_is_any(manufacturer, model, "_TYZB01_ikabtjsi", "TS0222")) return "illuminance";

    /* The supplied source has these two fields reversed for this product:
     * modelid = _TZ3000_xj1fk2vv, manufname = TS0202. Support the source as
     * written, and also the normal orientation if the actual device reports it. */
    if (tuya_is_any(manufacturer, model, "TS0202", "_TZ3000_xj1fk2vv") ||
        tuya_is_any(manufacturer, model, "_TZ3000_xj1fk2vv", "TS0202")) return "temperature_humidity";

    if (tuya_is_any(manufacturer, model, "_TZ3000_elv1x5d0", "TY0201")) return "temperature_humidity_custom";

    if (tuya_is_any(manufacturer, model, "_TZ3000_u3apvxay", "TS0044")) return "scene_switch";
    if (tuya_is_any(manufacturer, model, "_TZ3000_iiunlsf1", "TS0726")) return "scene_switch";
    if (tuya_is_any(manufacturer, model, "_TZ3000_unyg1lje", "TS0726")) return "scene_switch";
    if (tuya_is_any(manufacturer, model, "_TZ3000_unyg1lje", "TS0044")) return "scene_switch";
    if (tuya_is_any(manufacturer, model, "_TZ3000_hmrurcn7", "TS0044")) return "scene_switch";

    if (tuya_is_any(manufacturer, model, "_TZ3000_sgzohm9w", "TS0504B")) return "cct_light";
    if (tuya_is_any(manufacturer, model, "_TYZB01_lbdxarah", "TS110F")) return "scene_dali_dimmer";
    if (tuya_is_any(manufacturer, model, "_TZ3000_anxjonu1", "TS0014")) return "scene_dali";

    if (tuya_is_any(manufacturer, model, "_TZE200_6oarugsx", "TS0601")) return "rgb_custom";
    if (tuya_is_any(manufacturer, model, "_TZ3000_r7xbuwbw", "TS0503B")) return "rgb";
    if (tuya_is_any(manufacturer, model, "_TZ3000_ww5yena0", "TS0504B")) return "rgbw";
    if (tuya_is_any(manufacturer, model, "_TZ3000_s4zadi21", "TS0505B")) return "rgb_cw_ww";

    if (tuya_is_any(manufacturer, model, "_TZ3000_2qzlhsne", "TS0215A")) return "door_bell";

    return NULL;
}

void classify_zigbee_endpoint(uint16_t zigbee_device_type,
                              const uint16_t *clusters,
                              uint8_t cluster_count,
                              const char *manufacturer,
                              const char *model,
                              device_capability_t *cap)
{
    (void)zigbee_device_type;

    if (cap == NULL) {
        return;
    }

    memset(cap, 0, sizeof(*cap));
    cap->device_type = "unknown";

    const bool onoff       = has_cluster(clusters, cluster_count, ZCL_CLUSTER_ON_OFF);
    const bool level       = has_cluster(clusters, cluster_count, ZCL_CLUSTER_LEVEL_CONTROL);
    const bool color       = has_cluster(clusters, cluster_count, ZCL_CLUSTER_COLOR_CONTROL);
    const bool curtain     = has_cluster(clusters, cluster_count, ZCL_CLUSTER_WINDOW_COVERING);
    const bool fan         = has_cluster(clusters, cluster_count, ZCL_CLUSTER_FAN_CONTROL);
    const bool thermostat  = has_cluster(clusters, cluster_count, ZCL_CLUSTER_THERMOSTAT);
    const bool temperature = has_cluster(clusters, cluster_count, ZCL_CLUSTER_TEMPERATURE);
    const bool humidity    = has_cluster(clusters, cluster_count, ZCL_CLUSTER_HUMIDITY);
    const bool occupancy   = has_cluster(clusters, cluster_count, ZCL_CLUSTER_OCCUPANCY);
    const bool illuminance = has_cluster(clusters, cluster_count, ZCL_CLUSTER_ILLUMINANCE);
    const bool ias         = has_cluster(clusters, cluster_count, ZCL_CLUSTER_IAS_ZONE);

    const bool tuya_fan =
        has_cluster(clusters, cluster_count, ZCL_CLUSTER_NUOS_EF00) ||
        has_cluster(clusters, cluster_count, ZCL_CLUSTER_NUOS_ED00) ||
        has_cluster(clusters, cluster_count, ZCL_CLUSTER_NUOS_EE00);

    const bool scene_custom =
        has_cluster(clusters, cluster_count, ZCL_CLUSTER_NUOS_E000) ||
        has_cluster(clusters, cluster_count, ZCL_CLUSTER_NUOS_E001);

    const bool tuya_custom_config =
        has_cluster(clusters, cluster_count, ZCL_CLUSTER_NUOS_FC31);

    const char *hint = tuya_product_hint(manufacturer, model);

    cap->power = onoff;
    cap->brightness = level;
    cap->color = color;
    cap->curtain = curtain;
    cap->fan = fan;
    cap->thermostat = thermostat;
    cap->temperature = temperature;
    cap->humidity = humidity;
    cap->occupancy = occupancy;
    cap->illuminance = illuminance;

    if (ias) {
        if (hint != NULL && strcmp(hint, "motion") == 0) {
            cap->occupancy = true;
        } else if (hint != NULL && strcmp(hint, "contact") == 0) {
            cap->contact = true;
        } else if (hint != NULL && strcmp(hint, "gas") == 0) {
            cap->gas = true;
        } else if (text_has(model, "MOTION") || text_has(model, "PIR")) {
            cap->occupancy = true;
        }
    }

    /* Tuya identity resolves the otherwise ambiguous 0x0300 subtype. */
    if (color && hint != NULL &&
        (strcmp(hint, "cct_light") == 0)) {
        cap->color_temperature = true;
    } else if (color &&
               (text_has(model, "CCT") || text_has(model, "COLOR_TEMP") ||
                text_has(model, "CCT_DALI"))) {
        cap->color_temperature = true;
    }

    /* Identity-specific products where endpoint clusters alone are ambiguous. */
    if (hint != NULL) {
        if (strcmp(hint, "curtain") == 0 || strcmp(hint, "2ch_curtain") == 0) {
            cap->curtain = true;
        } else if (strcmp(hint, "thermostat") == 0 &&
                   (thermostat ||
                    has_cluster(clusters, cluster_count, ZCL_CLUSTER_NUOS_EE00) ||
                    has_cluster(clusters, cluster_count, ZCL_CLUSTER_NUOS_EF00))) {
            cap->thermostat = true;
        } else if ((strcmp(hint, "light_fan") == 0 ||
                    strcmp(hint, "ir_fan_custom") == 0) &&
                   (fan || tuya_fan)) {
            cap->fan = true;
        } else if (strcmp(hint, "temperature_humidity") == 0) {
            cap->temperature = temperature;
            cap->humidity = humidity;
        } else if (strcmp(hint, "temperature_humidity_custom") == 0) {
            cap->temperature = temperature;
            cap->humidity = humidity;
        } else if (strcmp(hint, "illuminance") == 0) {
            cap->illuminance = true;
        } else if (strcmp(hint, "scene_switch") == 0) {
            cap->device_type = "scene_switch";
            return;
        } else if (strcmp(hint, "cct_light") == 0 ||
                   strcmp(hint, "rgb") == 0 ||
                   strcmp(hint, "rgbw") == 0 ||
                   strcmp(hint, "rgb_cw_ww") == 0 ||
                   strcmp(hint, "rgb_custom") == 0) {
            cap->color = color || true;
        } else if (strcmp(hint, "dimmer") == 0) {
            cap->brightness = level || true;
            cap->power = onoff;
        }
    }

    if (cap->curtain) {
        cap->device_type = "curtain";
        return;
    }

    if (cap->thermostat) {
        cap->device_type = "thermostat";
        return;
    }

    /* For a light+fan product, preserve endpoint role: a standard fan/custom
     * Tuya fan endpoint is a fan; an OnOff/Level endpoint remains the light. */
    if (cap->fan && (fan || tuya_fan) && !onoff && !level && !color) {
        cap->device_type = "fan";
        return;
    }

    if (cap->occupancy) {
        cap->device_type = "motion";
        return;
    }

    if (cap->contact) {
        cap->device_type = "contact_sensor";
        return;
    }

    if (cap->gas) {
        cap->device_type = "gas_sensor";
        return;
    }

    if (cap->temperature && cap->humidity) {
        cap->device_type = "sensor";
        return;
    }

    if (cap->temperature) {
        cap->device_type = "temperature_sensor";
        return;
    }

    if (cap->humidity) {
        cap->device_type = "humidity_sensor";
        return;
    }

    if (cap->illuminance) {
        cap->device_type = "illuminance_sensor";
        return;
    }

    if (cap->color) {
        cap->device_type = cap->color_temperature ? "cct_light" : "color_light";
        return;
    }

    if (cap->brightness && cap->power) {
        cap->device_type = "dimmer";
        return;
    }

    if (cap->brightness) {
        cap->device_type = "dimmer";
        return;
    }

    if (cap->power) {
        /* TS0014 is also used by Group/Scene DALI and Scene DALI products;
         * cluster information decides the endpoint capability, not HA ID. */
        if (hint != NULL && strcmp(hint, "scene_dali") == 0) {
            cap->device_type = "scene_dali";
        } else {
            cap->device_type = "switch";
        }
        return;
    }

    if (scene_custom) {
        cap->device_type = "scene_switch";
        return;
    }

    if (tuya_custom_config) {
        cap->device_type = "config";
        return;
    }

    if (hint != NULL && strcmp(hint, "door_bell") == 0) {
        cap->device_type = "door_bell";
    }
}
