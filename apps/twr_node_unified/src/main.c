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

#if MYNEWT_VAL(DW1000_LWIP)
#include <dw1000/dw1000_lwip.h>
#endif
#if MYNEWT_VAL(DW1000_CCP_ENABLED)
#include <dw1000/dw1000_ccp.h>
#endif
#if MYNEWT_VAL(DW1000_PAN)
#include <dw1000/dw1000_pan.h>
#endif
#include <dw1000/dw1000_provision.h>
#include "extension.h"

static void pan_phase(dw1000_dev_instance_t* );
static void range_phase(dw1000_dev_instance_t* );
#define TDMA_DELTA 500000
#define TDMA_NSLOTS 10
#define NUM_NODES 32

typedef enum _state_machine_t{
    DW1000_DISCOVERY_STATE,
    DW1000_PROVISION_STATE,
    DW1000_RANGE_STATE,
    DW1000_INVALID_STATE,
}state_machine_t;

typedef struct _state_func_t{
    state_machine_t states;
    void (*func)(dw1000_dev_instance_t*);
}state_func_t;

state_machine_t state = DW1000_INVALID_STATE;
bool node0 = false;
uint16_t g_node_addr[] = {0xffff, 0xffff};
uint32_t cur_ticks = 0;
bool provision_completed = false;

state_func_t statemachine_t[] = {
    { DW1000_DISCOVERY_STATE, pan_phase              },
    { DW1000_PROVISION_STATE, dw1000_provision_start },
    { DW1000_RANGE_STATE    , range_phase            },
    { DW1000_INVALID_STATE  , NULL                   },
};

static dw1000_rng_config_t rng_config = {
    .tx_holdoff_delay = 0x0600,         // Send Time delay in usec.
    .rx_timeout_period = 0xf000         // Receive response timeout in usec
};

#if MYNEWT_VAL(DW1000_PAN)
static dw1000_pan_config_t pan_config = {
    .tx_holdoff_delay = 0x0C00,         // Send Time delay in usec.
    .rx_timeout_period = 0xffff         // Receive response timeout in usec.
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
static struct os_callout range_callout, statemachine_callout;

#define SAMPLE_FREQ 50.0
static void timer_ev_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    hal_gpio_toggle(LED_BLINK_PIN);
    
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_rng_instance_t * rng = inst->rng; 

    assert(inst->rng->nframes > 0);

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
        dw1000_set_rx_timeout(inst, 0);
        dw1000_start_rx(inst); 
    }

    else if (frame->code == DWT_SS_TWR_FINAL) {
        uint32_t time_of_flight = (uint32_t) dw1000_rng_twr_to_tof(rng);
        float range = dw1000_rng_tof_to_meters(dw1000_rng_twr_to_tof(rng));
        float rssi = dw1000_get_rssi(inst);
        print_frame("trw=", frame);
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
        dw1000_set_rx_timeout(inst, 0);
        dw1000_start_rx(inst); 
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
        dw1000_set_rx_timeout(inst, 0);
        dw1000_start_rx(inst); 
    }
    os_callout_reset(&range_callout, OS_TICKS_PER_SEC/SAMPLE_FREQ);
}

void superframe_ev_cb(struct os_event* ev){
    dw1000_dev_instance_t* inst = (dw1000_dev_instance_t*)ev->ev_arg;
    (*statemachine_t[state].func)(inst);
}

/*************Start of post processes************************/
void ccp_postprocess(struct os_event* ev){
    printf("CCP Post \n");
    dw1000_ccp_instance_t* ccp = (dw1000_ccp_instance_t*)ev->ev_arg;
    dw1000_dev_instance_t* inst = ccp->parent;
    ccp_frame_t * frame = ccp->frames[(ccp->idx)%ccp->nframes];
    uint32_t slot_delay = inst->slot_id * TDMA_DELTA/TDMA_NSLOTS;
    uint32_t delay = 0;
    //Guard time calculation & total delay calculation
    //Guard time is calculated as the slot_id*clock_offset/no_of_slots
    //If the native timer is running fast then subtract the guard_time from the slot_delay
    uint32_t guard_time = rng_config.tx_holdoff_delay;
    delay = (slot_delay + guard_time);
    
    if(inst->pan->status.valid != true){
        state = DW1000_DISCOVERY_STATE;
        cur_ticks = os_cputime_ticks_to_usecs(os_cputime_get32());
        //Call immediately for discovery [Slot0 being used]
        os_callout_reset(&statemachine_callout,1);
    }else{
        if(node0 == true && provision_completed != true){
            state = DW1000_PROVISION_STATE;
            os_callout_reset(&statemachine_callout, OS_TICKS_PER_SEC*(delay-MYNEWT_VAL(OS_LATENCY))*1e-6);
        }
        else{
            inst->txtimestamp = frame->reception_timestamp + 2*((uint64_t)delay << 15);
            state = DW1000_RANGE_STATE;
            os_callout_reset(&statemachine_callout,1);
        }
    }
}

static void provision_postprocess(struct os_event * ev){
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    dw1000_provision_instance_t * provision = inst->provision;
    dw1000_provision_stop(inst);
    uint16_t count = 0;
    if(provision->status.provision_status == PROVISION_DONE){
        if(provision->num_node_count < 2)
            printf("Not enough nodes found in the network\n");
        else{
            printf("Provision Completed\n");
            for(int i=0; i < provision->num_node_count; i++){
                if(provision->dev_addr[i] != 0 && provision->dev_addr[i] != 0xffff){
                    g_node_addr[i] = provision->dev_addr[i]; 
                    printf("Provision Completed with %x \n",provision->dev_addr[i]);
                    count++;
                }
            }
            if(count >= 2)
                provision_completed = true;
        }
    }
    dw1000_set_rx_timeout(inst, 0);
    dw1000_start_rx(inst);
}

static void
pan_postprocess(struct os_event* ev){
    dw1000_dev_instance_t* inst = (dw1000_dev_instance_t*)ev->ev_arg;
    printf("NOt yet cmoplete \n");
    if(inst->pan->status.valid != true){
        os_sem_release(&inst->pan->sem_waitforsucess);
   }else{
        printf("Discovery Completed \n");
        printf("DeviceID =%X\n",inst->my_short_address);
        printf("SlotID =%X\n",inst->slot_id);
        if(inst->slot_id == 1){
            node0 = true;
        }
    }
    dw1000_set_rx_timeout(inst, 0);
    dw1000_start_rx(inst);
}
/*************End of post processes************************/

/*************StateMachine Function start*********************/
static void pan_phase(dw1000_dev_instance_t* inst){
    dw1000_pan_start(inst, DWT_NONBLOCKING);
}

static void range_phase(dw1000_dev_instance_t* inst){
    os_callout_stop(&range_callout);
    dw1000_set_rx_timeout(inst, 0);
    dw1000_start_rx(inst);
    os_callout_reset(&range_callout,1);
}
/*************StateMachine Function End*********************/


static void init_timer(dw1000_dev_instance_t * inst) {
    os_callout_init(&statemachine_callout, os_eventq_dflt_get(), superframe_ev_cb, inst);
    os_callout_init(&range_callout, os_eventq_dflt_get(), timer_ev_cb, inst);
}
int main(int argc, char **argv){
    int rc;
    dw1000_provision_config_t config;
    sysinit();
    hal_gpio_init_out(LED_BLINK_PIN, 1);
    hal_gpio_init_out(LED_1, 1);
    hal_gpio_init_out(LED_3, 1);

    dw1000_dev_instance_t * inst = hal_dw1000_inst(0);
    dw1000_softreset(inst);
    dw1000_phy_init(inst, NULL);    
 
    inst->PANID = 0xDECA;
    inst->my_long_address = ((uint64_t) inst->device_id << 32) + inst->partID;

    dw1000_set_panid(inst,inst->PANID);
    dw1000_mac_init(inst, NULL);
    dw1000_rng_init(inst, &rng_config, sizeof(twr)/sizeof(twr_frame_t));
    dw1000_rng_set_frames(inst, twr, sizeof(twr)/sizeof(twr_frame_t));
#if MYNEWT_VAL(DW1000_CCP_ENABLED)
    dw1000_ccp_init(inst, 2, MYNEWT_VAL(UUID_CCP_MASTER));
    dw1000_ccp_set_postprocess(inst->ccp, ccp_postprocess);
#endif
#if MYNEWT_VAL(DW1000_PAN)
    dw1000_extension_callbacks_t pan_cbs;
    dw1000_pan_init(inst, &pan_config);
    dw1000_pan_set_postprocess(inst, pan_postprocess);
    dw1000_remove_extension_callbacks(inst, DW1000_PAN);
    pan_cbs.tx_complete_cb = pan_tx_complete_cb;
    pan_cbs.rx_complete_cb = pan_rx_complete_cb;
    pan_cbs.rx_timeout_cb = pan_rx_timeout_cb;
    pan_cbs.rx_error_cb = pan_rx_error_cb;
    pan_cbs.tx_error_cb = pan_tx_error_cb;

    dw1000_pan_set_ext_callbacks(inst, pan_cbs);
#endif
#if MYNEWT_VAL(DW1000_PROVISION)
    config.tx_holdoff_delay = rng_config.tx_holdoff_delay;
    config.rx_timeout_period = rng_config.rx_timeout_period;
    config.period = MYNEWT_VAL(PROVISION_PERIOD)*1e-3;
    config.postprocess = false;
    config.max_node_count = NUM_NODES;
    dw1000_provision_init(inst,config);
    dw1000_provision_set_postprocess(inst, &provision_postprocess);
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

