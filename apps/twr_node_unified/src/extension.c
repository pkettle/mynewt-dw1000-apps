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
void
pan_rx_complete_cb(dw1000_dev_instance_t * inst){
     printf("Rx Complete = %x \n",inst->fctrl_array[0]);
     if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        if(inst->extension_cb->next != NULL){
            inst->extension_cb = inst->extension_cb->next;
            if(inst->extension_cb->rx_complete_cb != NULL)
                inst->extension_cb->rx_complete_cb(inst);
        //For the range service the fctrl is same as FCNTL_IEEE_RANGE_16
        //In he range case the decision is always taken by application
        //So put it back to receive only if the intended packet doesn't match
        //any of the reserved or range packet
        }else if(inst->fctrl != FCNTL_IEEE_RANGE_16){
            dw1000_dev_control_t control = inst->control_rx_context;
            dw1000_restart_rx(inst, control);
        }
        return;
    }else if(inst->pan->status.valid == true){
        dw1000_dev_control_t control = inst->control_rx_context;
        dw1000_restart_rx(inst, control);
        return;
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
void
pan_tx_complete_cb(dw1000_dev_instance_t * inst){
   if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        if(inst->extension_cb->next != NULL){
            inst->extension_cb = inst->extension_cb->next;
            if(inst->extension_cb->tx_complete_cb != NULL)
                inst->extension_cb->tx_complete_cb(inst);
        }
    return;
   }
   dw1000_pan_instance_t * pan = inst->pan;
   os_sem_release(&inst->pan->sem);
   pan->idx++;
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

void
pan_tx_error_cb(dw1000_dev_instance_t * inst){
    /* Place holder */
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        if(inst->extension_cb->next != NULL){
            inst->extension_cb = inst->extension_cb->next;
            if(inst->extension_cb->tx_error_cb != NULL)
                inst->extension_cb->tx_error_cb(inst);
        }
        return;
    }
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
void
pan_rx_error_cb(dw1000_dev_instance_t * inst){
    /* Place holder */
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        if(inst->extension_cb->next != NULL){
            inst->extension_cb = inst->extension_cb->next;
            if(inst->extension_cb->rx_error_cb != NULL)
                inst->extension_cb->rx_error_cb(inst);
        }
        return;
    }
    os_sem_release(&inst->pan->sem);
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
void
pan_rx_timeout_cb(dw1000_dev_instance_t * inst){
    //printf("pan_rx_timeout_cb\n");  
    if(inst->fctrl_array[0] != FCNTL_IEEE_BLINK_TAG_64){
        if(inst->extension_cb->next != NULL){
            inst->extension_cb = inst->extension_cb->next;
            if(inst->extension_cb->rx_timeout_cb != NULL)
                inst->extension_cb->rx_timeout_cb(inst);
        }
        return;
    }
    dw1000_pan_instance_t * pan = inst->pan;
    if (pan->control.postprocess){
        os_eventq_put(os_eventq_dflt_get(), &pan->pan_callout_postprocess.c_ev);
    }
}

