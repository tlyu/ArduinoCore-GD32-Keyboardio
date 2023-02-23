/*!
    \file    usbd_pwr.c
    \brief   USB device power management driver

    \version 2020-08-01, V3.0.0, firmware for GD32F30x
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "usbd_pwr.h"

/*!
    \brief      start to remote wakeup
    \param[in]  udev: pointer to USB core instance
    \param[out] none
    \retval     none
*/
void usbd_remote_wakeup_active(usb_dev *udev)
{
    resume_mcu(udev);

#ifdef LPM_ENABLED
    if(1 == udev->lpm.L1_remote_wakeup){
        udev->drv_handler->resume(udev);

        udev->lpm.L1_resume = 1U;
    }
#endif /* LPM_ENABLED */

    if(1U == udev->pm.remote_wakeup){
        udev->pm.remote_wakeup_on = 1U;
        /*
         * bugfix: send remote wakeup for minimum, not maximum time
         *
         * The USB 2.0 specification says that the minimum length of a remote
         * wakeup sent by a device is 1ms, with a maximum of 15ms. The ISR
         * handles this by counting ESOF events, so we need to see two of them
         * to ensure that at least 1ms has elapsed.
         *
         * We add another 3 ESOFs just in case the peripheral doesn't enforce
         * the total minimum bus idle period of 5ms prior to sending a remote
         * wakeup. (3ms of idle are consumed by the suspend detection process)
         *
         * We don't want to use the maximum, because we might exceed it if we
         * count 15 ESOFs. Also, we don't want to miss any traffic by sending
         * the resume for too long, because there might be noncompliant hosts
         * or hubs that don't reflect the resume for the full 20ms required by
         * the spec.
         */
        udev->pm.esof_count = 5U;
        udev->drv_handler->resume(udev);
    }
}
