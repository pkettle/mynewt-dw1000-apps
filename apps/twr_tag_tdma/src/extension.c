/**
 * Copyright 2018, Decawave Limited, All Rights Reserved
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
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <os/os.h>
#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>
#include "bsp/bsp.h"

#include <dw1000/dw1000_regs.h>
#include <dw1000/dw1000_dev.h>
#include <dw1000/dw1000_hal.h>
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_phy.h>
#include <dw1000/dw1000_ftypes.h>
#include <dw1000/dw1000_rng.h>

#if MYNEWT_VAL(DW1000_PROVISION)
#include <dw1000/dw1000_provision.h>
#endif

#include <dw1000/dw1000_pan.h>

/*! 
 * @fn pan_rx_complete_cb(dw1000_dev_instance_t * inst)
 *
 * @brief This is an internal static function that executes on both the pan_master Node and the TAG/ANCHOR 
 * that initiated the blink. On the pan_master the postprecess function should allocate a PANID and a SLOTID, 
 * while on the TAG/ANCHOR the returned allocations are assigned and the PAN discover event is stopped. The pan 
 * discovery resources can be released. 
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * returns none
 */
bool
pan_rx_complete_cb(dw1000_dev_instance_t * inst){
     if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        return false;
    }else if(inst->pan->status.valid == true){
        dw1000_dev_control_t control = inst->control_rx_context;
        dw1000_restart_rx(inst, control);
        return true;
    }
    dw1000_pan_instance_t * pan = inst->pan;
    pan_frame_t * frame = pan->frames[(pan->idx)%pan->nframes];
    if(inst->frame_len == sizeof(struct _pan_frame_resp_t)) {
        dw1000_read_rx(inst, frame->array, 0, sizeof(struct _pan_frame_resp_t));
        if(frame->long_address == inst->my_long_address){
            // TAG/ANCHOR side

            inst->my_short_address = frame->short_address;
            inst->PANID = frame->pan_id;
            inst->slot_id = frame->slot_id;
            pan->status.valid = true;
            dw1000_pan_stop(inst);
            os_sem_release(&pan->sem);
            os_sem_release(&pan->sem_waitforsucess);
        }
    }
    // both pan_master and TAG/ANCHOR
    if (pan->control.postprocess)
        os_eventq_put(os_eventq_dflt_get(), &pan->pan_callout_postprocess.c_ev);
    return true;
}

/*! 
 * @fn pan_tx_complete_cb(dw1000_dev_instance_t * inst)
 *
 * @brief This is an internal static function that executes on the TAG/ANCHOR.
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * returns none
 */
bool
pan_tx_complete_cb(dw1000_dev_instance_t * inst){
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        return false;
    }
    dw1000_pan_instance_t * pan = inst->pan;
    os_sem_release(&inst->pan->sem);
    pan->idx++;
    return true;
}

/*!
 * @fn pan_tx_error_cb(dw1000_dev_instance_t * inst)
 *
 * @brief This is an internal static function that executes on the TAG/ANCHOR.
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t *
 *
 * output parameters
 *
 *
 * returns none
 */

bool
pan_tx_error_cb(dw1000_dev_instance_t * inst){
    /* Place holder */
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        return false;
    }
    return true;
}

/*!
 * @fn pan_rx_error_cb(dw1000_dev_instance_t * inst)
 *
 * @brief This is an internal static function that executes on the TAG/ANCHOR.
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t *
 *
 * output parameters
 *
 *
 * returns none
 */
bool
pan_rx_error_cb(dw1000_dev_instance_t * inst){
    /* Place holder */
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        return false;
    }
    os_sem_release(&inst->pan->sem);
    return true;
}

/*! 
 * @fn pan_rx_timeout_cb(dw1000_dev_instance_t * inst)
 *
 * @brief This is an internal static function that executes on the TAG/ANCHOR.
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * returns none
 */
bool
pan_rx_timeout_cb(dw1000_dev_instance_t * inst){
    //printf("pan_rx_timeout_cb\n");  
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        return false;
    }
    dw1000_pan_instance_t * pan = inst->pan;
    if (pan->control.postprocess){
        os_eventq_put(os_eventq_dflt_get(), &pan->pan_callout_postprocess.c_ev);
    }
    return true;
}

/*! 
 * @fn provision_rx_complete_cb(dw1000_dev_instance_t * inst)
 *
 * @brief This is an internal static function that executes on both the provision intiator and the TAG/ANCHOR 
 * that replies to the beacon. On the provision intiator the postprecess function can send the database to the application
 * and kick start some other task like ranging or so. In responder there is no need of post process and should just go to
 * rx mode again 
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * 
 *
 * output parameters
 *
 * returns none
 */
bool
provision_rx_complete_cb(dw1000_dev_instance_t* inst){
    assert(inst != NULL);
    if(inst->fctrl != FCNTL_IEEE_PROVISION_16){
        return false;
    }
    assert(inst->provision != NULL);
    uint16_t  frame_idx = inst->provision->idx;
    uint16_t code, dst_address;
    dw1000_provision_instance_t * provision = inst->provision;
    dw1000_provision_config_t config = provision->config;

    dw1000_read_rx(inst, (uint8_t *) &code, offsetof(ieee_rng_request_frame_t,code), sizeof(uint16_t));
    dw1000_read_rx(inst, (uint8_t *) &dst_address, offsetof(ieee_rng_request_frame_t,dst_address), sizeof(uint16_t));
    if ((dst_address != inst->my_short_address) && (dst_address != (uint16_t)0xFFFF)){
        if (dw1000_restart_rx(inst, inst->control).start_rx_error)
            inst->rng_rx_error_cb(inst);
        return true;
    }
    provision_frame_t * frame = &provision->frames[(frame_idx)%provision->nframes];
    switch(code)
    {
        case DWT_PROVISION_START:
            {
                if (dw1000_restart_rx(inst, inst->control).start_rx_error)
                    inst->rng_rx_error_cb(inst);
                break;
            }
        case DWT_PROVISION_RESP:
            {
                if (inst->frame_len >= sizeof(ieee_rng_response_frame_t))
                    dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
                else{
                    if (dw1000_restart_rx(inst,inst->control).start_rx_error)
                        inst->rng_rx_error_cb(inst);
                    break;
                }
                if(provision_add_node(inst,frame->src_address) == PROVISION_ERROR){
                    //If the addition fails when the number of nodes exceeds max allowed count callout the postprocess
                    os_error_t err = os_sem_release(&provision->sem);
                    assert(err == OS_OK);
                    provision->status.provision_status = PROVISION_DONE;
                    if (provision->config.postprocess){
                        os_eventq_put(os_eventq_dflt_get(), &provision->provision_callout_postprocess.c_ev);
                    }
                    break;
                }
                dw1000_dev_control_t control = inst->control_rx_context;
                if (dw1000_restart_rx(inst, control).start_rx_error)
                    inst->rng_rx_error_cb(inst);
                break;
            }
        default:
                if (inst->frame_len >= sizeof(ieee_rng_request_frame_t))
                    dw1000_read_rx(inst, frame->array, 0, sizeof(ieee_rng_request_frame_t));
                else{
                    if (dw1000_restart_rx(inst, inst->control).start_rx_error)
                        inst->rng_rx_error_cb(inst);
                    break;
                }
                uint8_t delay_factor = 1;  //Delay_factor for NODE_0
                if(inst->slot_id > 0) // if device is of NODE type
                   delay_factor = (inst->slot_id) * 4;  //Increase the delay factor for late response for Anchor provisioning
                uint64_t request_timestamp = dw1000_read_rxtime(inst);
                uint64_t response_tx_delay = request_timestamp + ((uint64_t)(config.tx_holdoff_delay*delay_factor) << 16);
                frame->dst_address = frame->src_address;
                frame->src_address = inst->my_short_address;
                frame->code = DWT_PROVISION_RESP;

                dw1000_write_tx(inst, frame->array, 0, sizeof(ieee_rng_response_frame_t));
                dw1000_write_tx_fctrl(inst, sizeof(ieee_rng_response_frame_t), 0, true);
                dw1000_set_wait4resp(inst,true);
                dw1000_set_delay_start(inst, response_tx_delay);
                dw1000_set_rx_timeout(inst,0);
                if (dw1000_start_tx(inst).start_tx_error){
                    printf("DWT_PROVISION_START tx error\n");
                }
                break;

            printf("Wrong request \n");
    }
    return true;
}

/*! 
 * @fn dw1000_rx_timeout_cb(dw1000_dev_instance_t * inst)
 *
 * @brief Handle the rx timeout case for provisioning
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * inst
 *
 * output parameters
 *
 * returns none
 */
bool
provision_rx_timeout_cb(dw1000_dev_instance_t * inst){
    assert(inst != NULL);

    if(inst->fctrl != FCNTL_IEEE_PROVISION_16){
		return false;
    }
    assert(inst->provision != NULL);
    dw1000_provision_instance_t *provision = inst->provision;
    if(provision->status.provision_status == PROVISION_START){
        printf("Finish \n");
        os_error_t err = os_sem_release(&provision->sem);
        assert(err == OS_OK);
		provision->status.provision_status = PROVISION_DONE;
        if (provision->config.postprocess){
            os_eventq_put(os_eventq_dflt_get(), &provision->provision_callout_postprocess.c_ev);
        }
    }else{
        inst->control = inst->control_rx_context;
        dw1000_start_rx(inst);
    }
    return true;
}

/*! 
 * @fn dw1000_rx_error_cb(dw1000_dev_instance_t * inst)
 *
 * @brief Handle the rx error case for provisioning
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * inst
 *
 * output parameters
 *
 * returns none
 */
bool
provision_rx_error_cb(dw1000_dev_instance_t * inst){
    assert(inst != NULL);
    if(inst->fctrl != FCNTL_IEEE_PROVISION_16){
     	return false;
    }
    assert(inst->provision != NULL);
    if(inst->provision->status.provision_status == PROVISION_START){
        os_error_t err = os_sem_release(&inst->provision->sem);
        assert(err == OS_OK);
        inst->provision->status.provision_status = PROVISION_DONE;
    }else{
        inst->control = inst->control_rx_context;
        dw1000_start_rx(inst);
    }
    return true;
}

/*! 
 * @fn provision_tx_error_cb(dw1000_dev_instance_t * inst)
 *
 * @brief Handle the tx error case for provisioning
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * inst
 *
 * output parameters
 *
 * returns none
 */
bool
provision_tx_error_cb(dw1000_dev_instance_t * inst){
    assert(inst != NULL);
    if(inst->fctrl != FCNTL_IEEE_PROVISION_16){
        return false;
    }
    return true;
}

/*! 
 * @fn dw1000_tx_complete_cb(dw1000_dev_instance_t * inst)
 *
 * @brief Handle the tx complete case. If required add the support
 *
 * input parameters
 * @param inst - dw1000_dev_instance_t * inst
 *
 * output parameters
 *
 * returns none
 */
bool
provision_tx_complete_cb(dw1000_dev_instance_t * inst){
    //Place holder
	if(inst->fctrl != FCNTL_IEEE_PROVISION_16){
        return false;
	}
    return true;
}
