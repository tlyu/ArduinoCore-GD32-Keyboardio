/*!
    \file    usbd_transc.c
    \brief   USBD transaction function

    \version 2020-08-01, V3.0.0, firmware for GD32F30x
    \version 2022-06-10, V3.1.0, firmware for GD32F30x
*/

/*
    Copyright (c) 2022, GigaDevice Semiconductor Inc.

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

#include "usbd_enum.h"
#include "usbd_transc.h"

/* local function prototypes ('static') */
static inline void usb_stall_transc (usb_dev *udev);
static inline void usb_ctl_data_in(usb_dev *udev);
static inline void usb_ctl_status_in (usb_dev *udev);
static inline void usb_ctl_data_out (usb_dev *udev);
static inline void usb_ctl_status_out (usb_dev *udev);
static inline void usb_0len_packet_send (usb_dev *udev);

/*!
    \brief      USB setup stage processing
    \param[in]  udev: pointer to USB device instance
    \param[out] none
    \retval     none
*/
void _usb_setup_transc (usb_dev *udev, uint8_t ep_num)
{
    (void)ep_num;

    usb_reqsta reqstat = REQ_NOTSUPP;

    /* Force to IDLE state, to cancel actions by any pending completion handlers. */
    udev->control.ctl_state = USBD_CTL_IDLE;

    switch (udev->control.req.bmRequestType & USB_REQTYPE_MASK) {
    /* standard device request */
    case USB_REQTYPE_STRD:
        reqstat = usbd_standard_request(udev, &udev->control.req);
        break;

    /* device class request */
    case USB_REQTYPE_CLASS:
        reqstat = usbd_class_request(udev, &udev->control.req);
        break;

    /* vendor defined request */
    case USB_REQTYPE_VENDOR:
        reqstat = usbd_vendor_request(udev, &udev->control.req);
        break;

    default:
        break;
    }

    if (REQ_SUPP == reqstat) {
        if (0U == udev->control.req.wLength) {
            /* USB control transfer status in stage */
            usb_ctl_status_in(udev);
        } else {
            if (udev->control.req.bmRequestType & 0x80U) {
                usb_ctl_data_in(udev);
            } else {
                /* USB control transfer data out stage */
                usb_ctl_data_out(udev);
            }
        }
    } else {
        usb_stall_transc(udev);
    }
}

/*!
    \brief      data out stage processing
    \param[in]  udev: pointer to USB device instance
    \param[in]  ep_num: endpoint identifier(0..7)
    \param[out] none
    \retval     none
*/
void _usb_out0_transc (usb_dev *udev, uint8_t ep_num)
{
    if (((uint8_t)USBD_CONFIGURED == udev->cur_status) && (udev->class_core->ctlx_out != NULL)) {
        /* device class handle */
        /*
         * bugfix: actually check return value from ctlx_out. This allows
         * the Arduino core to defer the OUT setup validation to ctlx_out,
         * and for the Arduino core's recvControl to work as expected.
         */
        if (USBD_OK != udev->class_core->ctlx_out(udev)) {
            usb_stall_transc(udev);
            return;
        };
    }

    if (USBD_CTL_DATA_OUT == udev->control.ctl_state) {
        /* enter the control transaction status IN stage */
        usb_ctl_status_in(udev);
    } else if (USBD_CTL_STATUS_OUT == udev->control.ctl_state) {
        usb_transc_config(&udev->transc_out[ep_num], NULL, 0U, 0U);

        udev->control.ctl_state = USBD_CTL_IDLE;
    } else {
        /* no operation */
    }
}

/*!
    \brief      data in stage processing
    \param[in]  udev: pointer to USB device instance
    \param[in]  ep_num: endpoint identifier(0..7)
    \param[out] none
    \retval     none
*/
void _usb_in0_transc (usb_dev *udev, uint8_t ep_num)
{
    (void)ep_num;

    if (udev->control.ctl_zlp) {
        /* send 0 length packet */
        usb_0len_packet_send(udev);

        udev->control.ctl_zlp = 0U;
    } 

    if (((uint8_t)USBD_CONFIGURED == udev->cur_status) && (udev->class_core->ctlx_in != NULL)) {
        (void)udev->class_core->ctlx_in(udev);
    }

    if (USBD_CTL_DATA_IN == udev->control.ctl_state) {
        /* USB control transfer status OUT stage */
        usb_ctl_status_out(udev);
    } else if (USBD_CTL_STATUS_IN == udev->control.ctl_state) {
        udev->control.ctl_state = USBD_CTL_IDLE;
    }

    if (0U != udev->dev_addr) {
        udev->drv_handler->set_addr(udev);

        udev->dev_addr = 0U;
    }
}
