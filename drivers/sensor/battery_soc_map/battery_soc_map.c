/*
 * Copyright (c) 2026 Joe Love
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT raven_battery_soc_map

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <errno.h>
#include <stdint.h>

LOG_MODULE_REGISTER(raven_batt_soc, CONFIG_SENSOR_LOG_LEVEL);

struct soc_map_config {
    const struct device *voltage;
    const int32_t *voltage_mv;
    const int32_t *percent;
    size_t count;
};

struct soc_map_data {
    uint16_t millivolts;
    uint8_t state_of_charge;
};

static uint8_t mv_to_pct(const struct soc_map_config *cfg, int32_t mv)
{
    if (mv >= cfg->voltage_mv[0]) {
        return (uint8_t)cfg->percent[0];
    }

    const size_t last = cfg->count - 1;

    if (mv <= cfg->voltage_mv[last]) {
        return (uint8_t)cfg->percent[last];
    }

    for (size_t i = 0; i < last; i++) {
        const int32_t v0 = cfg->voltage_mv[i];
        const int32_t v1 = cfg->voltage_mv[i + 1];

        if (mv <= v0 && mv >= v1) {
            const int32_t p0 = cfg->percent[i];
            const int32_t p1 = cfg->percent[i + 1];

            return (uint8_t)(p0 + (p1 - p0) * (mv - v0) / (v1 - v0));
        }
    }

    return 0;
}

static int soc_map_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    if (chan != SENSOR_CHAN_GAUGE_STATE_OF_CHARGE && chan != SENSOR_CHAN_GAUGE_VOLTAGE &&
        chan != SENSOR_CHAN_VOLTAGE && chan != SENSOR_CHAN_ALL) {
        return -ENOTSUP;
    }

    const struct soc_map_config *cfg = dev->config;
    struct soc_map_data *data = dev->data;
    int rc = sensor_sample_fetch_chan(cfg->voltage, SENSOR_CHAN_ALL);

    if (rc == -ENOTSUP) {
        rc = sensor_sample_fetch_chan(cfg->voltage, SENSOR_CHAN_GAUGE_VOLTAGE);
    }

    if (rc != 0) {
        LOG_DBG("Failed to fetch voltage: %d", rc);
        return rc;
    }

    struct sensor_value voltage;

    rc = sensor_channel_get(cfg->voltage, SENSOR_CHAN_GAUGE_VOLTAGE, &voltage);
    if (rc != 0) {
        LOG_DBG("Failed to get voltage: %d", rc);
        return rc;
    }

    int32_t mv = voltage.val1 * 1000 + voltage.val2 / 1000;

    if (mv < 0) {
        mv = 0;
    } else if (mv > UINT16_MAX) {
        mv = UINT16_MAX;
    }

    data->millivolts = (uint16_t)mv;
    data->state_of_charge = mv_to_pct(cfg, data->millivolts);

    LOG_DBG("%d mV => %d%%", data->millivolts, data->state_of_charge);

    return 0;
}

static int soc_map_channel_get(const struct device *dev, enum sensor_channel chan,
                              struct sensor_value *val)
{
    const struct soc_map_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_VOLTAGE:
    case SENSOR_CHAN_GAUGE_VOLTAGE:
        val->val1 = data->millivolts / 1000;
        val->val2 = (data->millivolts % 1000) * 1000U;
        break;
    case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
        val->val1 = data->state_of_charge;
        val->val2 = 0;
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}

static const struct sensor_driver_api soc_map_api = {
    .sample_fetch = soc_map_sample_fetch,
    .channel_get = soc_map_channel_get,
};

static int soc_map_init(const struct device *dev)
{
    const struct soc_map_config *cfg = dev->config;

    if (!device_is_ready(cfg->voltage)) {
        LOG_ERR("Voltage sensor %s is not ready", cfg->voltage->name);
        return -ENODEV;
    }

    if (cfg->count < 2) {
        LOG_ERR("voltage-mv and percent need at least two points");
        return -EINVAL;
    }

    for (size_t i = 0; i + 1 < cfg->count; i++) {
        if (cfg->voltage_mv[i] <= cfg->voltage_mv[i + 1]) {
            LOG_ERR("voltage-mv must be strictly decreasing");
            return -EINVAL;
        }
    }

    return 0;
}

/* Init priority must be a literal. 91 is one after the voltage sensor (90). */
#define RAVEN_BATTERY_SOC_MAP_INIT(n)                                                              \
    BUILD_ASSERT(DT_INST_PROP_LEN(n, voltage_mv) == DT_INST_PROP_LEN(n, percent),                  \
                 "voltage-mv and percent must be the same length");                                \
    BUILD_ASSERT(DT_INST_PROP_LEN(n, voltage_mv) >= 2,                                             \
                 "voltage-mv and percent need at least two points");                               \
                                                                                                   \
    static const int32_t voltage_mv_##n[] = DT_INST_PROP(n, voltage_mv);                           \
    static const int32_t percent_##n[] = DT_INST_PROP(n, percent);                                 \
                                                                                                   \
    static const struct soc_map_config soc_map_config_##n = {                                      \
        .voltage = DEVICE_DT_GET(DT_INST_PHANDLE(n, voltage_sensor)),                              \
        .voltage_mv = voltage_mv_##n,                                                              \
        .percent = percent_##n,                                                                    \
        .count = DT_INST_PROP_LEN(n, voltage_mv),                                                  \
    };                                                                                             \
                                                                                                   \
    static struct soc_map_data soc_map_data_##n;                                                   \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, soc_map_init, NULL, &soc_map_data_##n, &soc_map_config_##n,           \
                          POST_KERNEL, 91, &soc_map_api);

DT_INST_FOREACH_STATUS_OKAY(RAVEN_BATTERY_SOC_MAP_INIT)
