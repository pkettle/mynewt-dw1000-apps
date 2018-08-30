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

#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_rng.h>
#include <dw1000/dw1000_lwip.h>
#include <dw1000/dw1000_ftypes.h>

#if MYNEWT_VAL(DW1000_CCP_ENABLED)
#include <dw1000/dw1000_ccp.h>
#endif

#include <lwip/init.h>
#include <lwip/ethip6.h>
#include <netif/lowpan6.h>

#define PING_ID	0xDDEE
#define RX_STATUS false

#if MYNEWT_VAL(DW1000_CCP_ENABLED)
static dw1000_rng_config_t rng_config = {
    .tx_holdoff_delay = 0x0C00,         // Send Time delay in usec.
    .rx_timeout_period = 0x0800         // Receive response timeout in usec.
};

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
    }
};
#endif

static
dw1000_lwip_config_t lwip_config = {
	.poll_resp_delay = 0x4800,
	.resp_timeout = 0xF000,
	.uwbtime_to_systime = 0
};

ip_addr_t my_ip_addr = {
	.addr[0] = MYNEWT_VAL(DEV_IP6_ADDR_1),
	.addr[1] = MYNEWT_VAL(DEV_IP6_ADDR_2),
	.addr[2] = MYNEWT_VAL(DEV_IP6_ADDR_3),
	.addr[3] = MYNEWT_VAL(DEV_IP6_ADDR_4)
};

ip_addr_t ip6_tgt_addr[4];

err_t error;
char *payload;
uint16_t seq_no;

struct ping_payload{
	uint16_t src_addr;
	uint16_t dst_addr;
	uint16_t ping_id;
	uint16_t seq_no;
	uint16_t data[5];
};


#if MYNEWT_VAL(DW1000_CCP_ENABLED)
static void lwip_ping_tx(dw1000_dev_instance_t * inst) {

    hal_gpio_toggle(LED_BLINK_PIN);

    uint16_t payload_size = (uint16_t)sizeof(struct ping_payload);
    struct ping_payload *ping_pl = (struct ping_payload*)payload;

    ping_pl->src_addr = MYNEWT_VAL(SHORT_ADDRESS);
    ping_pl->dst_addr = 0x4321;
    ping_pl->ping_id = PING_ID;
    ping_pl->seq_no = seq_no;

    dw1000_lwip_send(inst, payload_size, payload, ip6_tgt_addr);
}

static void ccp_postprocess(struct os_event * ev){
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

	dw1000_ccp_instance_t * ccp = (dw1000_ccp_instance_t *)ev->ev_arg;
    dw1000_dev_instance_t* inst = ccp->parent;

    ccp_frame_t * frame = ccp->frames[(ccp->idx-1)%ccp->nframes];

    seq_no = frame->seq_num;
    lwip_ping_tx(inst);
    printf("Seq Num : %d\n", frame->seq_num);
    dw1000_set_rx_timeout(inst, 0);
    dw1000_start_rx(inst); 
}
#else
static struct os_callout blinky_callout;

#define SAMPLE_FREQ 10.0

static void timer_ev_cb(struct os_event *ev) {
    assert(ev != NULL);
    assert(ev->ev_arg != NULL);

    hal_gpio_toggle(LED_BLINK_PIN);
    
    dw1000_dev_instance_t * inst = (dw1000_dev_instance_t *)ev->ev_arg;
    uint16_t payload_size = (uint16_t)sizeof(struct ping_payload);

    os_callout_reset(&blinky_callout, OS_TICKS_PER_SEC/SAMPLE_FREQ);

    struct ping_payload *ping_pl = (struct ping_payload*)payload;
    ping_pl->src_addr = MYNEWT_VAL(SHORT_ADDRESS);
    ping_pl->dst_addr = 0x4321;
    ping_pl->ping_id = PING_ID;
    ping_pl->seq_no  = seq_no++;

    dw1000_lwip_send(inst, payload_size, payload, ip6_tgt_addr);

    printf("\n\tSeq # - %d\n\n", seq_no);

    if (inst->lwip->status.start_rx_error)
        printf("timer_ev_cb:[start_rx_error]\n");
    if (inst->lwip->status.start_tx_error)
        printf("timer_ev_cb:[start_tx_error]\n");
    if (inst->lwip->status.rx_error)
        printf("timer_ev_cb:[rx_error]\n");
    if (inst->lwip->status.request_timeout){
        printf("timer_ev_cb:[request_timeout]\n");
        error = ERR_INPROGRESS;
    }
    if (inst->lwip->status.rx_timeout_error){
        printf("timer_ev_cb:[rx_timeout_error]\n");
        error = ERR_TIMEOUT;
    }

    if (inst->lwip->status.start_tx_error ||
            inst->lwip->status.rx_error ||
            inst->lwip->status.request_timeout ||
            inst->lwip->status.rx_timeout_error){

        inst->lwip->status.start_tx_error = inst->lwip->status.rx_error = inst->lwip->status.request_timeout = inst->lwip->status.rx_timeout_error = 0;
    }
    print_error(error);
}

static void init_timer(dw1000_dev_instance_t * inst) {
    os_callout_init(&blinky_callout, os_eventq_dflt_get(), timer_ev_cb, inst);
    os_callout_reset(&blinky_callout, OS_TICKS_PER_SEC/SAMPLE_FREQ);
}
#endif

int main(int argc, char **argv){
	int rc;

	dw1000_dev_instance_t * inst = hal_dw1000_inst(0);
	
	sysinit();
	hal_gpio_init_out(LED_BLINK_PIN, 1);

	dw1000_softreset(inst);
	inst->PANID = MYNEWT_VAL(DEVICE_PAN_ID);
	inst->my_short_address = MYNEWT_VAL(SHORT_ADDRESS);
    inst->my_long_address = MYNEWT_VAL(DEVICE_UUID);

	dw1000_set_panid(inst,inst->PANID);
	dw1000_low_level_init(inst, NULL, NULL);

#if MYNEWT_VAL(DW1000_CCP_ENABLED)
    dw1000_rng_init(inst, &rng_config, sizeof(twr)/sizeof(twr_frame_t));
    dw1000_rng_set_frames(inst, twr, sizeof(twr)/sizeof(twr_frame_t));  

    dw1000_ccp_init(inst, 2, inst->my_long_address);   
    dw1000_ccp_set_postprocess(inst->ccp, &ccp_postprocess);
#endif

	dw1000_lwip_init(inst, &lwip_config, MYNEWT_VAL(NUM_FRAMES), MYNEWT_VAL(BUFFER_SIZE));
    dw1000_netif_config(inst, &inst->lwip->lwip_netif, &my_ip_addr, RX_STATUS);
	lwip_init();
    lowpan6_if_init(&inst->lwip->lwip_netif);

    inst->lwip->lwip_netif.flags |= NETIF_FLAG_UP | NETIF_FLAG_LINK_UP;
	lowpan6_set_pan_id(MYNEWT_VAL(DEVICE_PAN_ID));

    dw1000_pcb_init(inst);

    inst->lwip->dst_addr = 0x4321;

    IP_ADDR6(ip6_tgt_addr, MYNEWT_VAL(TGT_IP6_ADDR_1), MYNEWT_VAL(TGT_IP6_ADDR_2), 
                            MYNEWT_VAL(TGT_IP6_ADDR_3), MYNEWT_VAL(TGT_IP6_ADDR_4));


    payload = (char *)malloc(sizeof(struct ping_payload));
    assert(payload != NULL);

#if 0
    printf("device_id = 0x%lX\n",inst->device_id);
    printf("PANID = 0x%X\n",inst->PANID);
    printf("DeviceID = 0x%X\n",inst->my_short_address);
    printf("partID = 0x%lX\n",inst->partID);
    printf("lotID = 0x%lX\n",inst->lotID);
    printf("xtal_trim = 0x%X\n",inst->xtal_trim);
#endif

#if MYNEWT_VAL(DW1000_CCP_ENABLED)
	dw1000_set_rx_timeout(inst, 0);
    dw1000_start_rx(inst); 
#else
    init_timer(inst);
#endif

	while (1) {
        os_eventq_run(os_eventq_dflt_get());
	}

	assert(0);
	return rc;
}