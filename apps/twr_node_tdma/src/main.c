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
#include <float.h>
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
#if MYNEWT_VAL(TIMESCALE)
#include <timescale/timescale.h> 
#endif
#include <clkcal/clkcal.h>  
#include "json_encode.h"

#define VERBOSE0
#define NSLOTS 16
#if MYNEWT_VAL(TDMA_ENABLED)
static uint16_t g_slot[NSLOTS] = {0};
#endif

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
        .PANID = 0xDECA,                 // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID
    },
    [1] = {
        .fctrl = 0x8841,                // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = 0xDECA,                 // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID,
    },
    [2] = {
        .fctrl = 0x8841,                // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = 0xDECA,                 // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID
    },
    [3] = {
        .fctrl = 0x8841,                // frame control (0x8841 to indicate a data frame using 16-bit addressing).
        .PANID = 0xDECA,                 // PAN ID (0xDECA)
        .code = DWT_TWR_INVALID,
    }
};

cir_t g_cir;

static void slot_ev_cb(struct os_event *ev)
{
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);
  
    hal_gpio_toggle(LED_BLINK_PIN);
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_rng_instance_t * rng = inst->rng; 
    twr_frame_t * frame = rng->frames[(rng->idx)%rng->nframes];

    if (frame->code == DWT_DS_TWR_FINAL || frame->code == DWT_DS_TWR_EXT_FINAL) {
        float time_of_flight = dw1000_rng_twr_to_tof(rng);
        uint32_t utime =os_cputime_ticks_to_usecs(os_cputime_get32()); 
        float rssi = dw1000_get_rssi(inst);

        printf("{\"utime\": %lu,\"tof\": %lu,\"range\": %lu,\"azimuth\": %lu,\"res_tra\":\"%lX\","
                    " \"rec_tra\":\"%lX\",\"rssi\":%lu}\n",
                utime,
                *(uint32_t *)(&time_of_flight), 
                *(uint32_t *)(&frame->spherical.range),
                *(uint32_t *)(&frame->spherical.azimuth),
                (frame->response_timestamp - frame->request_timestamp),
                (frame->transmission_timestamp - frame->reception_timestamp),
                *(uint32_t *)(&rssi)
        );
        //json_cir_encode(&g_cir, utime, "cir", CIR_SIZE);
        frame->code = DWT_DS_TWR_END;
    }    
    else if (frame->code == DWT_SS_TWR_FINAL) {
        uint32_t time_of_flight = (uint32_t) dw1000_rng_twr_to_tof(rng);
        uint32_t utime =os_cputime_ticks_to_usecs(os_cputime_get32()); 
        float range = dw1000_rng_tof_to_meters(time_of_flight);

        printf("{\"utime\": %lu,\"tof\": %lu,\"range\": %lu,\"res_tra\":\"%lX\","
                    " \"rec_tra\":\"%lX\"}\n",
                utime,
                *(uint32_t *)(&time_of_flight), 
                *(uint32_t *)(&range),
                (frame->response_timestamp - frame->request_timestamp),
                (frame->transmission_timestamp - frame->reception_timestamp)
        );
        frame->code = DWT_SS_TWR_END;
    }
}

/*! 
 * @fn timeout_cb(struct os_event *ev)
 *
 * @brief This callback is in the interrupt context and is called on timeout event.
 * input parameters
 * @param inst - dw1000_dev_instance_t * inst
 *
 * output parameters
 *
 * returns none 
 */
static void 
timeout_cb(dw1000_dev_instance_t * inst) {
  
  uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());
#ifdef VERBOSE
    if (inst->status.rx_timeout_error){
            printf("{\"utime\": %lu,\"msg\": \"timeout_cb::rx_timeout_error\"}\n",utime);
    }
#endif
    if (inst->tdma->status.awaiting_superframe){
        printf("{\"utime\": %lu,\"msg\": \"timeout_cb::awaiting_superframe\"}\n",utime);
        dw1000_set_rx_timeout(inst, 0);
        dw1000_start_rx(inst); 
    }
}

/*! 
 * @fn error_cb(struct os_event *ev)
 *
 * @brief This callback is in the interrupt context and is called on error event.
 * In this example just log event. Note the default config is .rxauto_enable = 1  //!< On error re-enable 
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

#ifdef VERBOSE
    uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());
    if (inst->status.start_rx_error)
        printf("{\"utime\": %lu,\"msg\": \"error_cb::start_rx_error\"}\n",utime);
    else if (inst->status.start_tx_error)
        printf("{\"utime\": %lu,\"msg\":\"error_cb::start_tx_error\"}\n",utime);
    else if (inst->status.rx_error)
        printf("{\"utime\": %lu,\"msg\":\"error_cb::rx_error\"}\n",utime);
    else 
        printf("{\"utime\": %lu,\"msg\":\"error_cb::unhandled event\"}\n",utime);
#endif
    if (inst->tdma->status.awaiting_superframe){
            uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());
            printf("{\"utime\": %lu,\"msg\":\"error_cb::awaiting_superframe\"}\n",utime);
            dw1000_set_rx_timeout(inst, 0);
            dw1000_start_rx(inst); 
    }
}

/*! 
 * @fn superres_complete_cb(dw1000_dev_instance_t * inst)
 *
 * @brief This callback is in the interrupt context and is uses to schedule an pdoa_complete event on the default event queue.  
 * Processing should be kept to a minimum giving the context. All algorithms should be deferred to a thread on an event queue. 
 * In this example all postprocessing is performed in the pdoa_ev_cb.
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * returns none 
 */
/* The timer callout */
static struct os_callout slot_callout;

static void 
complete_cb(struct _dw1000_dev_instance_t * inst){

    if (inst->tdma->status.awaiting_superframe){
            uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());
            printf("{\"utime\": %lu,\"complete_cb\":\"awaiting_superframe\"}\n",utime); 
            dw1000_set_rx_timeout(inst, 0);
            dw1000_start_rx(inst); 
    }else{     
        os_callout_init(&slot_callout, os_eventq_dflt_get(), slot_ev_cb, inst);
        os_eventq_put(os_eventq_dflt_get(), &slot_callout.c_ev);
    }
}


/*! 
 * @fn slot_timer_cb(struct os_event * ev)
 *
 * @brief In this example this timer callback is used to start_rx.
 *
 * input parameters
 * @param inst - struct os_event *  
 *
 * output parameters
 *
 * returns none 
 */
static void 
slot_timer_cb(struct os_event * ev){

    dw1000_dev_instance_t * inst = hal_dw1000_inst(0);
    clkcal_instance_t * clk = inst->ccp->clkcal;
    tdma_instance_t * tdma = inst->tdma;
    uint16_t slot = *(uint16_t *)ev->ev_arg;

#if MYNEWT_VAL(ADAPTIVE_TIMESCALE_ENABLED) 
    uint64_t dx_time = (clk->epoch + (uint64_t) roundf(clk->skew * (double)((slot * (uint64_t)tdma->period << 16)/tdma->nslots)));
#else
    uint64_t dx_time = (clk->epoch + (uint64_t) ((slot * ((uint64_t)tdma->period << 16)/tdma->nslots)));
#endif
    dx_time = (dx_time + 0xFFFFFFFE00UL + inst->rx_antenna_delay - (MYNEWT_VAL(DW1000_IDLE_TO_RX_LATENCY) << 16)) & 0xFFFFFFFE00UL;
    
    dw1000_set_delay_start(inst, dx_time);
    dw1000_set_rx_timeout(inst, rng_config.rx_timeout_period);

    if(dw1000_start_rx(inst).start_rx_error){
        uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());
        printf("{\"utime\": %lu,\"msg\": \"slot_timer_cb:start_rx_error\"}\n",utime);
    }

#ifdef VERBOSE
    uint32_t utime = os_cputime_ticks_to_usecs(os_cputime_get32());
    printf("{\"utime\": %lu,\"slot\": %d,\"dx_time\": %llu}\n",utime, slot, dx_time);
#endif
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

    dw1000_set_panid(inst,inst->PANID);
    dw1000_mac_init(inst, NULL);
    dw1000_rng_init(inst, &rng_config, sizeof(twr)/sizeof(twr_frame_t));
    dw1000_rng_set_frames(inst, twr, sizeof(twr)/sizeof(twr_frame_t));
    dw1000_rng_set_error_extension_cb(inst, error_cb);
    dw1000_rng_set_rx_timeout_extension_cb(inst, timeout_cb);
    dw1000_rng_set_complete_cb(inst, complete_cb);

#if MYNEWT_VAL(DW1000_PAN)
        dw1000_pan_init(inst, &pan_config);   
        dw1000_pan_start(inst, DWT_NONBLOCKING); 
        while(inst->pan->status.valid != true){ 
            os_eventq_run(os_eventq_dflt_get());
        }  
#endif
    printf("device_id = 0x%lX\n",inst->device_id);
    printf("PANID = 0x%X\n",inst->PANID);
    printf("DeviceID = 0x%X\n",inst->my_short_address);
    printf("partID = 0x%lX\n",inst->partID);
    printf("lotID = 0x%lX\n",inst->lotID);
    printf("xtal_trim = 0x%X\n",inst->xtal_trim);

#if MYNEWT_VAL(DW1000_CCP_ENABLED)
    dw1000_ccp_init(inst, 2, MYNEWT_VAL(UUID_CCP_MASTER));
#endif
    
#if MYNEWT_VAL(TDMA_ENABLED) 
    for (uint16_t i = 1; i < sizeof(g_slot)/sizeof(uint16_t); i++)
        g_slot[i] = i;

    tdma_init(inst, MYNEWT_VAL(CCP_PERIOD), NSLOTS); 
    for (uint16_t i = 1; i < sizeof(g_slot)/sizeof(uint16_t); i++)
        tdma_assign_slot(inst->tdma, slot_timer_cb,  g_slot[i] ,  &g_slot[i]);
#else
    dw1000_set_rx_timeout(inst, 0);
    dw1000_start_rx(inst);
#endif

    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }
    assert(0);
    return rc;
}

