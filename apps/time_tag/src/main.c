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
#include "sysinit/sysinit.h"
#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#include "hal/hal_bsp.h"
#ifdef ARCH_sim
#include "mcu/mcu_sim.h"
#endif

#include "dw1000/dw1000_dev.h"
#include "dw1000/dw1000_hal.h"
#include "dw1000/dw1000_phy.h"
#include "dw1000/dw1000_mac.h"
#include "dw1000/dw1000_rng.h"
#include "dw1000/dw1000_ftypes.h"
#include "dw1000/dw1000_ccp.h"
#if MYNEWT_VAL(DW1000_TIME)
#include "dw1000/dw1000_time.h"
#endif
#if MYNEWT_VAL(DW1000_CLOCK_CALIBRATION)
#include <dw1000/dw1000_ccp.h>
#endif

static dw1000_rng_config_t rng_config = {
    .tx_holdoff_delay = 0x0600,         // Send Time delay in usec.
    .rx_timeout_period = 0x8000         // Receive response timeout in usec
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

/* The timer callout */
static struct os_callout blinky_callout;
static void timer_ev_cb(struct os_event* ev);

#define SAMPLE_FREQ 128.0
static uint32_t prev_cnt = 0;
static void timer_ev_cb(struct os_event* ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    hal_gpio_toggle(LED_BLINK_PIN);
    
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_rng_instance_t * rng = inst->rng; 

    assert(inst->rng->nframes > 0);
	
    os_callout_reset(&blinky_callout, OS_TICKS_PER_SEC*(MYNEWT_VAL(CCP_PERIOD) - MYNEWT_VAL(OS_LATENCY))*1e-6);

    dw1000_phy_forcetrxoff(inst);
    dw1000_mac_framefilter(inst,DWT_FF_DATA_EN);
    dw1000_rng_request_delay_start(inst, 0xabab, inst->txtimestamp, DWT_DS_TWR);
    uint32_t cur_cnt = os_cputime_ticks_to_usecs(os_cputime_get32());
    prev_cnt = cur_cnt;
    inst->txtimestamp = time_absolute(inst, (uint64_t)(inst->txtimestamp), MYNEWT_VAL(CCP_PERIOD)) & 0xFFFFFFFE00UL;

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
        inst->status.rx_timeout_error = inst->status.start_tx_error = inst->status.rx_error = 0;
        dw1000_set_rx_timeout(inst, 0);
        dw1000_start_rx(inst);
    }

    else if (frame->code == DWT_SS_TWR_FINAL) {
        uint32_t time_of_flight = (uint32_t) dw1000_rng_twr_to_tof(rng);
        float range = dw1000_rng_tof_to_meters(dw1000_rng_twr_to_tof(rng));
        float rssi = dw1000_get_rssi(inst);
        print_frame("trw=", frame);
        frame->code = DWT_SS_TWR_END;
        printf("{\"utime\": %lu,\"tof\": %lu,\"range\": %lu,\"res_req\": %lX,"
               " \"rec_tra\": %lX, \"rssi\": %d}\n",
            os_cputime_ticks_to_usecs(os_cputime_get32()),
            time_of_flight, 
            (uint32_t)(range * 1000), 
            (frame->response_timestamp - frame->request_timestamp),
            (frame->transmission_timestamp - frame->reception_timestamp),
            (int)(rssi)
        );         
        dw1000_set_rx_timeout(inst, 0);
        dw1000_start_rx(inst); 
    }

    else if (frame->code == DWT_DS_TWR_FINAL || frame->code == DWT_DS_TWR_EXT_FINAL) {
        uint32_t time_of_flight = (uint32_t) dw1000_rng_twr_to_tof(rng);
        float range = dw1000_rng_tof_to_meters(dw1000_rng_twr_to_tof(rng));
        float rssi = dw1000_get_rssi(inst);

        frame->code = DWT_DS_TWR_END;
            printf("{\"utime\": %lu, \"src\":0x%x, \"dst\":0x%x, \"tof\": %lu,\"range\": %lu,\"res_req\": %lX,"
                   " \"rec_tra\": %lX, \"rssi\": %d}\n",
            os_cputime_ticks_to_usecs(os_cputime_get32()),
            frame->src_address,
            frame->dst_address,
            time_of_flight, 
            (uint32_t)(range * 1000), 
            (frame->response_timestamp - frame->request_timestamp),
            (frame->transmission_timestamp - frame->reception_timestamp),
            (int)(rssi)
        );
        dw1000_set_rx_timeout(inst, 0);
        dw1000_start_rx(inst); 
    }
    dw1000_mac_framefilter(inst,DWT_FF_DATA_EN|DWT_FF_RSVD_EN);
}

static void init_timer(dw1000_dev_instance_t * inst) {
    os_callout_init(&blinky_callout, os_eventq_dflt_get(), timer_ev_cb, inst);
}

static void
ccp_postprocess(struct os_event * ev){

    os_callout_stop(&blinky_callout);

    assert(ev != NULL);
    assert(ev->ev_arg != NULL);
    printf("CCP post \n");
    dw1000_ccp_instance_t * ccp = (dw1000_ccp_instance_t *)ev->ev_arg;
    dw1000_dev_instance_t * inst = ccp->parent;
    //ccp_frame_t * previous_frame = ccp->frames[(ccp->idx-1)%ccp->nframes];
    ccp_frame_t * frame = ccp->frames[(ccp->idx)%ccp->nframes];

    uint32_t slot_delay = inst->slot_id * MYNEWT_VAL(TDMA_DELTA)/MYNEWT_VAL(TDMA_NSLOTS);
    uint32_t delay = 0;
    //Guard time calculation & total delay calculation
    //Guard time is calculated as the slot_id*clock_offset/no_of_slots
    //If the native timer is running fast then subtract the guard_time from the slot_delay
    uint32_t guard_time = rng_config.tx_holdoff_delay;
    delay = (slot_delay + guard_time) ;
    
    inst->txtimestamp = time_absolute(inst, (uint64_t)(frame->reception_timestamp), delay) & 0xFFFFFFFE00UL;
    os_callout_reset(&blinky_callout, OS_TICKS_PER_SEC*(delay - MYNEWT_VAL(OS_LATENCY))*1e-6);
}

int main(int argc, char **argv){
    int rc;

    sysinit();
    hal_gpio_init_out(LED_BLINK_PIN, 1);
    hal_gpio_init_out(LED_1, 1);
    hal_gpio_init_out(LED_3, 1);

    dw1000_dev_instance_t * inst = hal_dw1000_inst(0);
    dw1000_softreset(inst);
    dw1000_phy_init(inst, NULL);    
 
    inst->PANID = 0xDECA;
    inst->my_short_address = MYNEWT_VAL(DEVICE_ID);
    inst->my_long_address = ((uint64_t) inst->device_id << 32) + inst->partID;
    inst->slot_id = MYNEWT_VAL(SLOT_ID);

    dw1000_set_panid(inst,inst->PANID);
    dw1000_set_address16(inst,inst->my_short_address);

    dw1000_mac_init(inst, NULL);
    dw1000_mac_framefilter(inst,DWT_FF_DATA_EN|DWT_FF_RSVD_EN);
    dw1000_rng_init(inst, &rng_config, sizeof(twr)/sizeof(twr_frame_t));
    dw1000_rng_set_frames(inst, twr, sizeof(twr)/sizeof(twr_frame_t));
#if MYNEWT_VAL(DW1000_CCP_ENABLED)
    dw1000_ccp_init(inst, 2, MYNEWT_VAL(UUID_CCP_MASTER));
    dw1000_ccp_set_postprocess(inst->ccp, ccp_postprocess);
#endif
    printf("device_id=%lX\n",inst->device_id);
    printf("PANID=%X\n",inst->PANID);
    printf("DeviceID =%X\n",inst->my_short_address);
    printf("partID =%lX\n",inst->partID);
    printf("lotID =%lX\n",inst->lotID);
    printf("xtal_trim =%X\n",inst->xtal_trim);

    init_timer(inst);
    dw1000_set_rx_timeout(inst, 0);
    dw1000_start_rx(inst);

    while (1) {
        os_eventq_run(os_eventq_dflt_get());   
    }

    assert(0);
    return rc;
}
