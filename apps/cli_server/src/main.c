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
#include "bsp/bsp.h"
#include <imgmgr/imgmgr.h>
#include "hal/hal_gpio.h"
#include "hal/hal_bsp.h"
#ifdef ARCH_sim
#include "mcu/mcu_sim.h"
#endif

/* BLE */
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_uuid.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_l2cap.h"
#include "host/ble_sm.h"
#include "controller/ble_ll.h"

/* RAM HCI transport. */
#include "transport/ram/ble_hci_ram.h"

#include "dw1000/dw1000_dev.h"
#include "dw1000/dw1000_hal.h"
#include "dw1000/dw1000_phy.h"
#include "dw1000/dw1000_mac.h"
#include "dw1000/dw1000_rng.h"
#include "dw1000/dw1000_ftypes.h"

#if MYNEWT_VAL(DW1000_LWIP)
#include <dw1000/dw1000_lwip.h>
#endif
#if MYNEWT_VAL(DW1000_CCP_ENABLED)
#include <dw1000/dw1000_ccp.h>
#endif
#if MYNEWT_VAL(DW1000_PAN)
#include <dw1000/dw1000_pan.h>
#endif

/* Mandatory services. */
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* Newtmgr include */
#include "newtmgr/newtmgr.h"
#include "nmgrble/newtmgr_ble.h"
#include "bleuart/bleuart.h"

static int bleuart_gap_event(struct ble_gap_event *event, void *arg);
extern uint16_t g_console_conn_handle;
extern uint16_t g_bleuart_attr_read_handle;
static int bleuart_gap_event(struct ble_gap_event *event, void *arg);
static struct os_callout ble_callout;
int bleuart_write(void const* buffer, uint32_t size);
void ranging_start(void);
void ranging_stop(void);
int ble_tag_uart_gatt_svr_init(void);

static dw1000_rng_config_t rng_config = {
    .tx_holdoff_delay = 0x0380,      // Send Time delay in usec.
    .rx_timeout_period = 0x0         // timeout delta usec
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

void print_frame(const char * name, twr_frame_t *twr ){
    printf("%s{\n\tfctrl:0x%04X,\n", name, twr->fctrl);
    printf("\tseq_num:0x%02X,\n", twr->seq_num);
    printf("\tPANID:0x%04X,\n", twr->PANID);
    printf("\tdst_address:0x%04X,\n", twr->dst_address);
    printf("\tsrc_address:0x%04X,\n", twr->src_address);
    printf("\tcode:0x%04X,\n", twr->code);
    printf("\treception_timestamp:0x%08lX,\n", twr->reception_timestamp);
    printf("\ttransmission_timestamp:0x%08lX,\n", twr->transmission_timestamp);
    printf("\trequest_timestamp:0x%08lX,\n", twr->request_timestamp);
    printf("\tresponse_timestamp:0x%08lX\n}\n", twr->response_timestamp);
}

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void
bleuart_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /*
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o 128 bit UUID
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

    fields.uuids128 = BLE_UUID128(&gatt_svr_svc_uart_uuid.u);
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        return;
    }

    memset(&fields, 0, sizeof fields);
    fields.name = (uint8_t *)ble_svc_gap_device_name();
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&fields);
    if (rc != 0) {
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, bleuart_gap_event, NULL);
    if (rc != 0) {
        return;
    }
}
/* The timer callout */
static struct os_callout blinky_callout;

#define SAMPLE_FREQ 25.0
static void timer_ev_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    hal_gpio_toggle(LED_BLINK_PIN);
    
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_rng_instance_t * rng = inst->rng; 

    dw1000_rng_request(inst, 0x4321, DWT_DS_TWR);
    
    twr_frame_t * previous_frame = rng->frames[(rng->idx-1)%rng->nframes];
    twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];

    if (inst->status.start_rx_error)
        printf("{\"utime\": %lu,\"timer_ev_cb\": \"start_rx_error\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    if (inst->status.start_tx_error)
        printf("{\"utime\": %lu,\"timer_ev_cb\":\"start_tx_error\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    if (inst->status.rx_error)
        printf("{\"utime\": %lu,\"timer_ev_cb\":\"rx_error\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
    if (inst->status.rx_timeout_error)
        printf("{\"utime\": %lu,\"timer_ev_cb\":\"rx_timeout_error\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
   
    if (inst->status.start_tx_error || inst->status.start_rx_error || inst->status.rx_error 
        ||  inst->status.rx_timeout_error){
        inst->status.start_tx_error = inst->status.start_rx_error = inst->status.rx_error = inst->status.rx_timeout_error = 0;
    }

    else if (frame->code == DWT_SS_TWR_FINAL) {
        uint32_t time_of_flight = (uint32_t) dw1000_rng_twr_to_tof(rng);
        float range = dw1000_rng_tof_to_meters(dw1000_rng_twr_to_tof(rng));
        float rssi = dw1000_get_rssi(inst);
        print_frame("twr=", frame);
        frame->code = DWT_SS_TWR_END;
        printf("{\"utime\": %lu,\"tof\": %lu,\"range\": %lu,\"res_req\":\"%lX\","
               " \"rec_tra\":\"%lX\", \"rssi\": %d}\n",
            os_cputime_ticks_to_usecs(os_cputime_get32()),
            time_of_flight, 
            (uint32_t)(range * 1000), 
            (frame->response_timestamp - frame->request_timestamp),
            (frame->transmission_timestamp - frame->reception_timestamp),
            (int)(rssi)
        );         
    }

    else if (frame->code == DWT_DS_TWR_FINAL || frame->code == DWT_DS_TWR_EXT_FINAL) {
        uint32_t time_of_flight = (uint32_t) dw1000_rng_twr_to_tof(rng);
        float range = dw1000_rng_tof_to_meters(dw1000_rng_twr_to_tof(rng));
        float rssi = dw1000_get_rssi(inst);
        print_frame("1st=", previous_frame);
        print_frame("2nd=", frame);
        frame->code = DWT_DS_TWR_END;
            printf("{\"utime\": %lu,\"tof\": %lu,\"range\": %lu,\"res_req\":\"%lX\","
                   " \"rec_tra\":\"%lX\", \"rssi\": %d}\n",
            os_cputime_ticks_to_usecs(os_cputime_get32()),
            time_of_flight, 
            (uint32_t)(range * 1000), 
            (frame->response_timestamp - frame->request_timestamp),
            (frame->transmission_timestamp - frame->reception_timestamp),
            (int)(rssi)
        );
        char ran[5];
        sprintf(ran, "%lu\n",(uint32_t)(range*1000));
        bleuart_write(ran , 5);

    }
    os_callout_reset(&blinky_callout, OS_TICKS_PER_SEC/SAMPLE_FREQ);
}

static int count = 0;
static void timer_ev_cb_1(struct os_event *ev) {

    char str[25] ;
    sprintf(str,"%d %s %d",MYNEWT_VAL_BLE_DEVICE_ID,"Hello",count++);
    bleuart_write((void const*)str, sizeof(str));
}
void ranging_start(void)
{
    os_callout_reset(&blinky_callout, OS_TICKS_PER_SEC/SAMPLE_FREQ);
}

void ranging_stop(void)
{
    os_callout_stop(&blinky_callout);
}

static void init_timer(dw1000_dev_instance_t * inst) {
    os_callout_init(&blinky_callout, os_eventq_dflt_get(), timer_ev_cb, inst);
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleuart uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unuesd by
 *                                  bleuart.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
bleuart_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            bleuart_set_conn_handle(event->connect.conn_handle);
        }

        if (event->connect.status != 0) {
            /* Connection failed; resume advertising. */
            bleuart_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        /* Connection terminated; resume advertising. */
        bleuart_advertise();
        return 0;


    case BLE_GAP_EVENT_ADV_COMPLETE:
        /* Advertising terminated; resume advertising. */
        bleuart_advertise();
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
    }

    return 0;
}

static void
bleuart_on_sync(void)
{
    /* Begin advertising. */
    bleuart_advertise();
}

int bleuart_write(void const* buffer, uint32_t size)
{   
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buffer, size);
          
#if MYNEWT_VAL(BLEUART_STATS)
        STATS_INCN(g_bleuart_stats, txd_bytes, size);
#endif 

    return (0 == ble_gattc_notify_custom(g_console_conn_handle, g_bleuart_attr_read_handle, om)) ? size : 0;
}

int main(int argc, char **argv){
    int rc;

    sysinit();
    hal_gpio_init_out(LED_BLINK_PIN, 1);
    hal_gpio_init_out(LED_1, 1);
    hal_gpio_init_out(LED_3, 1);

    dw1000_dev_instance_t * inst = hal_dw1000_inst(0);
    
    inst->PANID = 0xDECA;
    inst->my_short_address = MYNEWT_VAL(DEVICE_ID);
    inst->my_long_address = ((uint64_t) inst->device_id << 32) + inst->partID;

    dw1000_set_panid(inst,inst->PANID);
    dw1000_mac_init(inst, NULL);
    dw1000_rng_init(inst, &rng_config, sizeof(twr)/sizeof(twr_frame_t));
    dw1000_rng_set_frames(inst, twr, sizeof(twr)/sizeof(twr_frame_t));
#if MYNEWT_VAL(DW1000_CCP_ENABLED)
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
    
    init_timer(inst);

    /* Set initial BLE device address. */
    memcpy(g_dev_addr, MYNEWT_VAL_BLE_PUBLIC_DEV_ADDR , 6);
    /* Initialize the BLE host. */
    log_register("ble_hs", &ble_hs_log, &log_console_handler, NULL,
                 LOG_SYSLEVEL);
    ble_hs_cfg.sync_cb = bleuart_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    rc = ble_tag_uart_gatt_svr_init();
    assert(rc == 0);
    
    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("DECAWAVE_BLE");
    assert(rc == 0);
    os_callout_init(&ble_callout, os_eventq_dflt_get(), timer_ev_cb_1, NULL);
    os_callout_reset(&ble_callout, OS_TICKS_PER_SEC/100);
    while (1) {
        os_eventq_run(os_eventq_dflt_get());   
    }

    assert(0);

    return rc;
}

