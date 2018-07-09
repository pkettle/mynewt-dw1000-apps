/**
 * Copyright (C) 2017-2018, Decawave Limited, All Rights Reserved
 * 
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 * 
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include "os/mynewt.h"
#include "sysinit/sysinit.h"
#include "os/os.h"
#include "console/console.h"
#include "hal/hal_system.h"
#include "config/config.h"
#include "split/split.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#include "hal/hal_bsp.h"
#ifdef ARCH_sim
#include "mcu/mcu_sim.h"
#endif

/* BLE */
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

#include <log/log.h>
#include "dw1000/dw1000_dev.h"
#include "dw1000/dw1000_hal.h"
#include "dw1000/dw1000_phy.h"
#include "dw1000/dw1000_mac.h"
#include "dw1000/dw1000_rng.h"
#include "dw1000/dw1000_ftypes.h"

#if MYNEWT_VAL(DW1000_LWIP)
#include <dw1000/dw1000_lwip.h>
#endif
#if MYNEWT_VAL(DW1000_CLOCK_CALIBRATION)
#include <dw1000/dw1000_ccp.h>
#endif
#if MYNEWT_VAL(DW1000_PAN)
#include <dw1000/dw1000_pan.h>
#endif

#include <shell/shell.h>
#include <stats/stats.h>
#include <config/config.h>
#include "flash_map/flash_map.h"
#include <newtmgr/newtmgr.h>
#include <bootutil/image.h>
#include <bootutil/bootutil.h>
#include <imgmgr/imgmgr.h>
#include <reboot/log_reboot.h>
#include <id/id.h>
#include <mgmt/mgmt.h>
#include <hal/hal_flash.h>

/* Application-specified header. */
#include "bleprph.h"

/** Log data. */
struct log bleprph_log;

static int bleprph_gap_event(struct ble_gap_event *event, void *arg);
#define MAX_CBMEM_BUF 600

STATS_SECT_START(range)
STATS_SECT_ENTRY(counts)
STATS_SECT_END

static STATS_NAME_START(range)
STATS_NAME(range, counts)
STATS_NAME_END(range)

static uint8_t test8;
static uint8_t test8_shadow;
static char test_str[32];
static uint32_t cbmem_buf[MAX_CBMEM_BUF];
static uint32_t error_buf[MAX_CBMEM_BUF];
static uint32_t debug_buf[MAX_CBMEM_BUF];
static struct cbmem error_mem;
static struct cbmem cbmem;
static struct cbmem debug_mem;
static STATS_SECT_DECL(range) g_stats_range;

//struct log os_log;
//struct log os_debug_log;
//struct log os_error_log;
static char *test_conf_get(int argc, char **argv, char *val, int max_len);
static int test_conf_set(int argc, char **argv, char *val);
static int test_conf_commit(void);
static int test_conf_export(void (*export_func)(char *name, char *val),
  enum conf_export_tgt tgt);

static dwt_config_t mac_config = {
    .chan = 5,                          // Channel number. 
    .prf = DWT_PRF_64M,                 // Pulse repetition frequency. 
    .txPreambLength = DWT_PLEN_256,     // Preamble length. Used in TX only. 
    .rxPAC = DWT_PAC8,                 // Preamble acquisition chunk size. Used in RX only. 
    .txCode = 11,                        // TX preamble code. Used in TX only. 
    .rxCode = 11,                        // RX preamble code. Used in RX only. 
    .nsSFD = 0,                         // 0 to use standard SFD, 1 to use non-standard SFD. 
    .dataRate = DWT_BR_6M8,             // Data rate. 
    .phrMode = DWT_PHRMODE_STD,         // PHY header mode. 
    .sfdTO = (256 + 1 + 8 - 8)          // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. 
};

static struct conf_handler test_conf_handler = {
    .ch_name = "test",
    .ch_get = test_conf_get,
    .ch_set = test_conf_set,
    .ch_commit = test_conf_commit,
    .ch_export = test_conf_export
};

static char *
test_conf_get(int argc, char **argv, char *buf, int max_len)
{
    if (argc == 1) {
        if (!strcmp(argv[0], "8")) {
            return conf_str_from_value(CONF_INT8, &test8, buf, max_len);
        } else if (!strcmp(argv[0], "str")) {
            return test_str;
        }
    }
    return NULL;
}

static int
test_conf_set(int argc, char **argv, char *val)
{
    if (argc == 1) {
        if (!strcmp(argv[0], "8")) {
            return CONF_VALUE_SET(val, CONF_INT8, test8_shadow);
        } else if (!strcmp(argv[0], "str")) {
            return CONF_VALUE_SET(val, CONF_STRING, test_str);
        }
    }
    return OS_ENOENT;
}

static int
test_conf_commit(void)
{
    test8 = test8_shadow;

    return 0;
}

static int
test_conf_export(void (*func)(char *name, char *val), enum conf_export_tgt tgt)
{
    char buf[4];

    conf_str_from_value(CONF_INT8, &test8, buf, sizeof(buf));
    func("test/8", buf);
    func("test/str", test_str);
    return 0;
}

static dw1000_phy_txrf_config_t txrf_config = { 
        .PGdly = TC_PGDELAY_CH5,
        //.power = 0x25456585
        .BOOSTNORM = dw1000_power_value(DW1000_txrf_config_9db, 5),
        .BOOSTP500 = dw1000_power_value(DW1000_txrf_config_9db, 5),
        .BOOSTP250 = dw1000_power_value(DW1000_txrf_config_9db, 5),
        .BOOSTP125 = dw1000_power_value(DW1000_txrf_config_9db, 5)
};

static dw1000_rng_config_t rng_config = {
    .tx_holdoff_delay = 0x0600,         // Send Time delay in usec.
    .rx_timeout_period = 0x0800         // Receive response timeout in usec
};

#if MYNEWT_VAL(DW1000_PAN)
static dw1000_pan_config_t pan_config = {
    .tx_holdoff_delay = 0x0C00,         // Send Time delay in usec.
    .rx_timeout_period = 0x8000         // Receive response timeout in usec.
};
#endif

static twr_frame_t twr[] = {
    [0] = {
        .fctrl = 0x8841,                // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = 0xDECA,                // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID
    },
    [1] = {
        .fctrl = 0x8841,                // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = 0xDECA,                // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID
    }
};

/**
 * Logs information about a connection to the console.
 */
static void
bleprph_print_conn_desc(struct ble_gap_conn_desc *desc)
{
    BLEPRPH_LOG(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=",
                desc->conn_handle, desc->our_ota_addr.type);
    print_addr(desc->our_ota_addr.val);
    BLEPRPH_LOG(INFO, " our_id_addr_type=%d our_id_addr=",
                desc->our_id_addr.type);
    print_addr(desc->our_id_addr.val);
    BLEPRPH_LOG(INFO, " peer_ota_addr_type=%d peer_ota_addr=",
                desc->peer_ota_addr.type);
    print_addr(desc->peer_ota_addr.val);
    BLEPRPH_LOG(INFO, " peer_id_addr_type=%d peer_id_addr=",
                desc->peer_id_addr.type);
    print_addr(desc->peer_id_addr.val);
    BLEPRPH_LOG(INFO, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                "encrypted=%d authenticated=%d bonded=%d\n",
                desc->conn_itvl, desc->conn_latency,
                desc->supervision_timeout,
                desc->sec_state.encrypted,
                desc->sec_state.authenticated,
                desc->sec_state.bonded);
}

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void
bleprph_advertise(void)
{
    uint8_t own_addr_type;
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        BLEPRPH_LOG(ERROR, "error determining address type; rc=%d\n", rc);
        return;
}

/**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assiging the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids16 = (ble_uuid16_t[]){
        BLE_UUID16_INIT(GATT_SVR_SVC_ALERT_UUID)
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        BLEPRPH_LOG(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, bleprph_gap_event, NULL);
    if (rc != 0) {
        BLEPRPH_LOG(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleprph uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unuesd by
 *                                  bleprph.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        BLEPRPH_LOG(INFO, "connection %s; status=%d ",
                       event->connect.status == 0 ? "established" : "failed",
                       event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            bleprph_print_conn_desc(&desc);

#if MYNEWT_VAL(BLEPRPH_LE_PHY_SUPPORT)
            phy_conn_changed(event->connect.conn_handle);
#endif
        }
        BLEPRPH_LOG(INFO, "\n");

        if (event->connect.status != 0) {
            /* Connection failed; resume advertising. */
            bleprph_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        BLEPRPH_LOG(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        bleprph_print_conn_desc(&event->disconnect.conn);
        BLEPRPH_LOG(INFO, "\n");

#if MYNEWT_VAL(BLEPRPH_LE_PHY_SUPPORT)
        phy_conn_changed(CONN_HANDLE_INVALID);
#endif

        /* Connection terminated; resume advertising. */
        bleprph_advertise();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        BLEPRPH_LOG(INFO, "connection updated; status=%d ",
                    event->conn_update.status);
        rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        BLEPRPH_LOG(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        BLEPRPH_LOG(INFO, "advertise complete; reason=%d",
                    event->adv_complete.reason);
        bleprph_advertise();
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        BLEPRPH_LOG(INFO, "encryption change event; status=%d ",
                    event->enc_change.status);
        rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        BLEPRPH_LOG(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        BLEPRPH_LOG(INFO, "subscribe event; conn_handle=%d attr_handle=%d "
                          "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                    event->subscribe.conn_handle,
                    event->subscribe.attr_handle,
                    event->subscribe.reason,
                    event->subscribe.prev_notify,
                    event->subscribe.cur_notify,
                    event->subscribe.prev_indicate,
                    event->subscribe.cur_indicate);
        return 0;

    case BLE_GAP_EVENT_MTU:
        BLEPRPH_LOG(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* We already have a bond with the peer, but it is attempting to
         * establish a new secure link.  This app sacrifices security for
         * convenience: just throw away the old bond and accept the new link.
         */

        /* Delete the old bond. */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with the pairing operation.
         */
        return BLE_GAP_REPEAT_PAIRING_RETRY;

#if MYNEWT_VAL(BLEPRPH_LE_PHY_SUPPORT)
    case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
        /* XXX: assume symmetric phy for now */
        phy_update(event->phy_updated.tx_phy);
        return 0;
#endif
    }

    return 0;
}

static void
bleprph_on_reset(int reason)
{
    BLEPRPH_LOG(ERROR, "Resetting state; reason=%d\n", reason);
}

static void
bleprph_on_sync(void)
{
    int rc;

    /* Make sure we have proper identity address set (public preferred) */
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Begin advertising. */
    bleprph_advertise();
}

/* The timer callout */
static struct os_callout blinky_callout;

#define SAMPLE_FREQ 50.0
static void timer_ev_cb(struct os_event *ev) {
    float rssi;
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    hal_gpio_toggle(LED_BLINK_PIN);
    os_callout_reset(&blinky_callout, OS_TICKS_PER_SEC/SAMPLE_FREQ);
    
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_rng_instance_t * rng = inst->rng; 

    assert(inst->rng->nframes > 0);

    twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];
#if MYNEWT_VAL(DW1000_LOG)
    if (inst->status.start_rx_error)
    
        LOG_ERROR(&inst->os_error_log, LOG_MODULE_DEFAULT,"start_rx_error\"\n");

    if (inst->status.start_tx_error)

        LOG_ERROR(&inst->os_error_log, LOG_MODULE_DEFAULT,"start_tx_error\"\n");

    if (inst->status.rx_error)

        LOG_ERROR(&inst->os_error_log, LOG_MODULE_DEFAULT,"rx_error\"\n");

    if (inst->status.rx_timeout_error)

        LOG_ERROR(&inst->os_error_log, LOG_MODULE_DEFAULT,"rx_timeout_error\"\n");
#endif

    if (inst->status.start_tx_error || inst->status.start_rx_error || inst->status.rx_error 
        ||  inst->status.rx_timeout_error){
        dw1000_set_rx_timeout(inst, 0);
        dw1000_start_rx(inst); 
    }

    else if (frame->code == DWT_SS_TWR_FINAL) {
        dw1000_get_rssi(inst, &rssi);
        frame->code = DWT_SS_TWR_END;
        dw1000_set_rx_timeout(inst, 0);
        dw1000_start_rx(inst); 
    }

    else if (frame->code == DWT_DS_TWR_FINAL || frame->code == DWT_DS_TWR_EXT_FINAL) {
        uint32_t time_of_flight = (uint32_t) dw1000_rng_twr_to_tof(rng);
        float range = dw1000_rng_tof_to_meters(dw1000_rng_twr_to_tof(rng));
        dw1000_get_rssi(inst, &rssi);
        frame->code = DWT_DS_TWR_END;

#if MYNEWT_VAL(DW1000_LOG)
            LOG_INFO(&inst->os_log, LOG_MODULE_DEFAULT, "Range %lu Time_of_flight %lu ", 
                (uint32_t)(range*1000),time_of_flight);
            STATS_INC(g_stats_range, counts);
#endif
        dw1000_set_rx_timeout(inst, 0);
        dw1000_start_rx(inst); 
    }
 }


static void init_timer(dw1000_dev_instance_t * inst) {
    os_callout_init(&blinky_callout, os_eventq_dflt_get(), timer_ev_cb, inst);
    os_callout_reset(&blinky_callout, OS_TICKS_PER_SEC/100);
}

int main(int argc, char **argv){
    int rc;

    sysinit();
    
        /* Initialize the bleprph log. */
    log_register("bleprph", &bleprph_log, &log_console_handler, NULL,
                 LOG_SYSLEVEL);

    /* Initialize the NimBLE host configuration. */
    log_register("ble_hs", &ble_hs_log, &log_console_handler, NULL,
                 LOG_SYSLEVEL);
    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("nimble-bleprph");
    assert(rc == 0);

#if MYNEWT_VAL(BLEPRPH_LE_PHY_SUPPORT)
    phy_init();
#endif

    conf_load();

    /* If this app is acting as the loader in a split image setup, jump into
     * the second stage application instead of starting the OS.
     */
#if MYNEWT_VAL(SPLIT_LOADER)
    {
        void *entry;
        rc = split_app_go(&entry, true);
        if (rc == 0) {
            hal_system_start(entry);
        }
    }
#endif
    hal_gpio_init_out(LED_BLINK_PIN, 1);
    hal_gpio_init_out(LED_1, 1);
    hal_gpio_init_out(LED_3, 1);

    dw1000_dev_instance_t * inst = hal_dw1000_inst(0);
    dw1000_softreset(inst);
    dw1000_phy_init(inst, &txrf_config);    
 
    inst->PANID = 0xDECA;
    inst->my_short_address = MYNEWT_VAL(DEVICE_ID);
    inst->my_long_address = ((uint64_t) inst->device_id << 32) + inst->partID;

    dw1000_set_panid(inst,inst->PANID);
    dw1000_mac_init(inst, &mac_config);
    dw1000_rng_init(inst, &rng_config, sizeof(twr)/sizeof(twr_frame_t));
    dw1000_rng_set_frames(inst, twr, sizeof(twr)/sizeof(twr_frame_t));
#if MYNEWT_VAL(DW1000_CLOCK_CALIBRATION)
    dw1000_ccp_init(inst, 2, MYNEWT_VAL(UUID_CCP_MASTER));
#endif
#if MYNEWT_VAL(DW1000_PAN)
    dw1000_pan_init(inst, &pan_config); 
    dw1000_pan_start(inst, DWT_NONBLOCKING); // Don't block on the eventq_dflt
    while(inst->pan->status.valid != true){ 
        os_eventq_run(os_eventq_dflt_get());
        os_cputime_delay_usecs(5000);
    } 
#endif
    printf("device_id=%lX\n",inst->device_id);
    printf("PANID=%X\n",inst->PANID);
    printf("DeviceID =%X\n",inst->my_short_address);
    printf("partID =%lX\n",inst->partID);
    printf("lotID =%lX\n",inst->lotID);
    printf("xtal_trim =%X\n",inst->xtal_trim);
    
    rc = conf_register(&test_conf_handler);
    assert(rc == 0);
    cbmem_init(&cbmem, cbmem_buf, MAX_CBMEM_BUF);
    log_register("log", &inst->os_log, &log_cbmem_handler, &cbmem, LOG_LEVEL_INFO);    
    cbmem_init(&error_mem, error_buf, MAX_CBMEM_BUF);
    log_register("Error_log", &inst->os_error_log, &log_cbmem_handler,&error_mem, LOG_LEVEL_ERROR);
    cbmem_init(&debug_mem, debug_buf, MAX_CBMEM_BUF);
    log_register("Debug_log", &inst->os_debug_log, &log_cbmem_handler,&debug_mem, LOG_LEVEL_DEBUG);
    stats_init(STATS_HDR(g_stats_range),
               STATS_SIZE_INIT_PARMS(g_stats_range, STATS_SIZE_32),
               STATS_NAME_INIT_PARMS(range));

    stats_register("range_count", STATS_HDR(g_stats_range));

    conf_load();

    reboot_start(hal_reset_cause());    
    init_timer(inst);
    
    dw1000_set_rx_timeout(inst, 0);
    dw1000_start_rx(inst);
    while (1) {
        os_eventq_run(os_eventq_dflt_get());  
    }

    assert(0);

    return rc;
}

