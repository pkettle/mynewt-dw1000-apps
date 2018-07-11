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
#include <math.h>
#include "sysinit/sysinit.h"
#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#include "hal/hal_bsp.h"
#ifdef ARCH_sim
#include "mcu/mcu_sim.h"
#endif

#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_rng.h>
#include <dw1000/dw1000_ftypes.h>

#if MYNEWT_VAL(DW1000_CCP_ENABLED)
#include <dw1000/dw1000_ccp.h>
#endif
#if MYNEWT_VAL(TDMA_ENABLED)
#include <dw1000/dw1000_tdma.h>
#endif
#if MYNEWT_VAL(DW1000_LWIP)
#include <dw1000/dw1000_lwip.h>
#endif
#if MYNEWT_VAL(DW1000_PAN)
#include <dw1000/dw1000_pan.h>
#endif


static dw1000_rng_config_t rng_config = {
    .tx_holdoff_delay = 0x0500,         // Send Time delay in usec.
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
        .PANID = 0xDECA,                 // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID
    },
    [1] = {
        .fctrl = 0x8841,                // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = 0xDECA,                 // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID
    }
};


#define NSLOTS 4
static void timeout_cb(struct _dw1000_dev_instance_t * inst);
static void error_cb(struct _dw1000_dev_instance_t * inst);

/*! 
 * @fn frame_timer_cb(struct os_event * ev)
 *
 * @brief This function each 
 *
 * input parameters
 * @param inst - struct os_event *  
 *
 * output parameters
 *
 * returns none 
 */
static void 
frame_timer_cb(struct os_event * ev){
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());
    printf("{\"utime\": %lu,\"msg\": \"frame_timer_cb\"}\n",utime);

    hal_gpio_toggle(LED_BLINK_PIN);
    dw1000_dev_instance_t * inst = hal_dw1000_inst(0);
    dw1000_rng_instance_t * rng = inst->rng;
    clkcal_instance_t * clk = inst->ccp->clkcal;
    tdma_instance_t * tdma = inst->tdma;
    uint16_t slot = *(uint16_t *)ev->ev_arg;
    uint64_t delay = (clk->epoch 
                    + (uint64_t) (clk->skew * (slot * ((uint64_t)(tdma->period) << 16)/tdma->nslots))
                    + (uint64_t) (clk->skew * ((tdma->idx) * ((uint64_t)tdma->period << 16)))) & 0xFFFFFFFE00UL;

    dw1000_phy_forcetrxoff(inst);
    uint32_t tic = os_cputime_ticks_to_usecs(os_cputime_get32());
    if(dw1000_rng_request_delay_start(inst, 0x4321, delay, DWT_DS_TWR).start_tx_error){
        uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());
        printf("{\"utime\": %lu,\"frame_timer_cb\":\"start_tx_error\"}\n",utime);
        for (uint16_t i=1; i < tdma->nslots; i++) {
            if (tdma->timer_cb[i]){
                os_callout_stop(tdma->timer_cb[i]);
            }
        }
        dw1000_phy_forcetrxoff(inst);
        dw1000_set_rx_timeout(inst, 0);
        dw1000_start_rx(inst); 
    }else{
        uint32_t toc = os_cputime_ticks_to_usecs(os_cputime_get32());
        printf("{\"utime\": %lu,\"frame_timer_cb_tic_toc\": %lu}\n",toc,toc-tic);
    }
   // twr_frame_t * previous_frame = rng->frames[(rng->idx-1)%rng->nframes];
    twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];
 
    if (frame->code == DWT_SS_TWR_FINAL) {
        float time_of_flight = (float) dw1000_rng_twr_to_tof(rng);
        float range = dw1000_rng_tof_to_meters(dw1000_rng_twr_to_tof(rng));
        printf("{\"utime\": %lu,\"tof\": %lu,\"range\": %lu,\"res_req\": \"%lX\","
                " \"rec_tra\": \"%lX\"}\n",
                os_cputime_ticks_to_usecs(os_cputime_get32()),
                *(uint32_t *)(&time_of_flight), 
                *(uint32_t *)(&range),
                (frame->response_timestamp - frame->request_timestamp),
                (frame->transmission_timestamp - frame->reception_timestamp)
        );
        frame->code = DWT_SS_TWR_END;
    }

    if (frame->code == DWT_DS_TWR_FINAL) {
        float time_of_flight = dw1000_rng_twr_to_tof(rng);
        float range = dw1000_rng_tof_to_meters(dw1000_rng_twr_to_tof(rng));
        printf("{\"utime\": %lu,\"tof\": %lu,\"range\": %lu,\"azimuth\": %lu,\"res_req\":\"%lX\","
                " \"rec_tra\": \"%lX\"}\n",
                os_cputime_ticks_to_usecs(os_cputime_get32()), 
                *(uint32_t *)(&time_of_flight), 
                *(uint32_t *)(&range),
                *(uint32_t *)(&frame->spherical.azimuth),
                (frame->response_timestamp - frame->request_timestamp),
                (frame->transmission_timestamp - frame->reception_timestamp)
        );
        frame->code = DWT_DS_TWR_END;
    } 
    if (frame->code == DWT_DS_TWR_EXT_FINAL) {
        float time_of_flight = dw1000_rng_twr_to_tof(rng);
        printf("{\"utime\": %lu,\"tof\": %lu,\"range\": %lu,\"azimuth\": %lu,\"res_req\":\"%lX\","
                " \"rec_tra\": \"%lX\"}\n",
                os_cputime_ticks_to_usecs(os_cputime_get32()), 
                *(uint32_t *)(&time_of_flight), 
                *(uint32_t *)(&frame->spherical.range),
                *(uint32_t *)(&frame->spherical.azimuth),
                (frame->response_timestamp - frame->request_timestamp),
                (frame->transmission_timestamp - frame->reception_timestamp)
        );
        frame->code = DWT_DS_TWR_END;
    } 
}

/*! 
 * @fn timeout_cb(struct os_event *ev)
 *
 * @brief This callback is in the interrupt context and is called on timeout event.
 * In this example re enable rx.
 * Note interrupt context so overlapping IO is possible
 * input parameters
 * @param inst - dw1000_dev_instance_t * inst
 *
 * output parameters
 *
 * returns none 
 */
static void 
timeout_cb(struct _dw1000_dev_instance_t * inst) {
        if (inst->status.rx_timeout_error){
            printf("{\"utime\": %lu,\"timeout_cb\": \"rx_timeout_error\"}\n",os_cputime_ticks_to_usecs(os_cputime_get32()));
        }
}

/*! 
 * @fn error_cb(struct os_event *ev)
 *
 * @brief This callback is in the interrupt context and is called on error event.
 * In this example just log event. 
 * Note: interrupt context so overlapping IO is possible
 * input parameters
 * @param inst - dw1000_dev_instance_t * inst
 *
 * output parameters
 *
 * returns none 
 */
static void 
error_cb(struct _dw1000_dev_instance_t * inst) {
   
    uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());
    if (inst->status.start_rx_error)
        printf("{\"utime\": %lu,\"error_cb\": \"start_rx_error\"}\n",utime);
    if (inst->status.start_tx_error)
        printf("{\"utime\": %lu,\"error_cb\":\"start_tx_error\"}\n",utime);
    if (inst->status.rx_error)
        printf("{\"utime\": %lu,\"error_cb\":\"rx_error\"}\n",utime);
}


#if MYNEWT_VAL(TDMA_ENABLED)
static uint16_t g_slot[] = {1//,2,3,4,5,6,7,8,9,10,11,12,13,14,15,
                            //16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
                          //  32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,
                          //  48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63
                            };
#endif

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

    dw1000_set_panid(inst,inst->PANID);
    dw1000_mac_init(inst, &inst->config);
    dw1000_rng_init(inst, &rng_config, sizeof(twr)/sizeof(twr_frame_t));
    dw1000_rng_set_frames(inst, twr, sizeof(twr)/sizeof(twr_frame_t));
    dw1000_rng_set_error_extension_cb(inst, error_cb);
    dw1000_rng_set_rx_timeout_extension_cb(inst, timeout_cb);

#if MYNEWT_VAL(DW1000_CCP_ENABLED)
    dw1000_ccp_init(inst, 2, MYNEWT_VAL(UUID_CCP_MASTER));
#endif
    clkcal_set_postprocess(inst->ccp->clkcal, tdma_superframe_event_cb);
#if MYNEWT_VAL(DW1000_PAN)
    dw1000_pan_init(inst, &pan_config);   
    dw1000_pan_start(inst, DWT_NONBLOCKING);  
#endif
    printf("device_id = 0x%lX\n",inst->device_id);
    printf("PANID = 0x%X\n",inst->PANID);
    printf("DeviceID = 0x%X\n",inst->my_short_address);
    printf("partID = 0x%lX\n",inst->partID);
    printf("lotID = 0x%lX\n",inst->lotID);
    printf("xtal_trim = 0x%X\n",inst->xtal_trim);
#if MYNEWT_VAL(TDMA_ENABLED)
    tdma_init(inst, MYNEWT_VAL(CCP_PERIOD), NSLOTS); 
    tdma_assign_slot(inst->tdma, frame_timer_cb, g_slot[0], &g_slot[0]);
    for (uint16_t i=1 ;i < sizeof(g_slot)/sizeof(uint16_t); i++)
        tdma_assign_slot(inst->tdma, frame_timer_cb, g_slot[i], &g_slot[i]);
#endif
    dw1000_set_rx_timeout(inst, 0);
    dw1000_start_rx(inst); 
 
    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }
    assert(0);
    return rc;
}

