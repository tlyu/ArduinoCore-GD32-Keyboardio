#ifdef USBCON
#include "USBCore.h"

extern "C" {
#include "gd32/usb.h"
#include "usbd_enum.h"
#include "usbd_lld_regs.h"
#include "usbd_pwr.h"
#include "usbd_transc.h"
}

#include <cassert>

// bMaxPower in Configuration Descriptor
#define USB_CONFIG_POWER_MA(mA)                ((mA)/2)
#ifndef USB_CONFIG_POWER
#define USB_CONFIG_POWER                      (500)
#endif

// TODO: make the device descriptor a member variable which can be
// overridden by subclasses.
static usb_desc_dev devDesc = {
    .header = {
        .bLength          = USB_DEV_DESC_LEN,
        .bDescriptorType  = USB_DESCTYPE_DEV
    },
    .bcdUSB                = 0x0200,
    .bDeviceClass          = 0xef,
    .bDeviceSubClass       = 0x02,
    .bDeviceProtocol       = 0x01,
    // TODO: this depends on what the mcu can support, but this is
    // device dependent code, so nevermind?
    .bMaxPacketSize0       = USB_EP_SIZE,
    .idVendor              = USB_VID,
    .idProduct             = USB_PID,
    .bcdDevice             = 0x0100,
    // Can set these to 0 so they’ll be ignored.
    .iManufacturer         = STR_IDX_MFC,
    .iProduct              = STR_IDX_PRODUCT,
    .iSerialNumber         = STR_IDX_SERIAL,
    .bNumberConfigurations = 1
};

usb_desc_config configDesc = {
    .header = {
        .bLength = sizeof(usb_desc_config),
        .bDescriptorType = USB_DESCTYPE_CONFIG
    },
    .wTotalLength = 0,
    .bNumInterfaces = 0,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
#ifdef USBD_IS_SELF_POWERED
#ifdef USBD_REMOTE_WAKEUP
    .bmAttributes = 0b11100000,
#else
    .bmAttributes = 0b11000000,
#endif // USBD_REMOTE_WAKEUP
#else
#ifdef USBD_REMOTE_WAKEUP
    .bmAttributes = 0b10100000,
#else
    .bmAttributes = 0b10000000,
#endif // USBD_REMOTE_WAKEUP
#endif // USBD_SELF_POWERED
    .bMaxPower = USB_CONFIG_POWER_MA(USB_CONFIG_POWER)
};

#pragma pack(1)
/* String descriptor with char16_t[], to use UTF-16 string literals */
typedef struct _usb_desc_utf16 {
    usb_desc_header header;
    char16_t unicode_string[];
} usb_desc_utf16;
#pragma pack()

/* Turn an ordinary string literal into a UTF-16 one */
#define XUSTR(s) u ## s
#define USTR(s) XUSTR(s)
#define USTRLEN(s) (2 * ((sizeof(s) - 1)))
#define U16DESC(s)                                                          \
    {                                                                       \
        .header = {                                                         \
            .bLength = sizeof(usb_desc_header) + USTRLEN(s),                \
            .bDescriptorType = USB_DESCTYPE_STR                             \
        },                                                                  \
        /* Can't use designated initializer here because G++ complains */   \
        USTR(s)                                                             \
    }

/* USB language ID Descriptor */
static usb_desc_LANGID usbd_language_id_desc =
{
    .header =
     {
         .bLength         = sizeof(usb_desc_LANGID),
         .bDescriptorType = USB_DESCTYPE_STR
     },
    .wLANGID              = ENG_LANGID
};

static usb_desc_utf16 mfcDesc = U16DESC(USB_MANUFACTURER);
static usb_desc_utf16 prodDesc = U16DESC(USB_PRODUCT);

/* USBD serial string */
static usb_desc_str serialDesc = {
    .header = {
        .bLength         = USB_STRING_LEN(12),
        .bDescriptorType = USB_DESCTYPE_STR,
    },
    .unicode_string = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

static uint8_t* stringDescs[] = {
    [STR_IDX_LANGID]  = (uint8_t *)&usbd_language_id_desc,
    [STR_IDX_MFC]     = (uint8_t *)&mfcDesc,
    [STR_IDX_PRODUCT] = (uint8_t *)&prodDesc,
    [STR_IDX_SERIAL]  = (uint8_t *)&serialDesc
};

usb_desc desc = {
    .dev_desc    = (uint8_t *)&devDesc,
    .config_desc = nullptr,
    .bos_desc    = nullptr,
    .strings     = stringDescs
};

// Must be called with interrupts disabled
template<size_t L>
void EPBuffer<L>::init(uint8_t ep)
{
    this->ep = ep;
    this->reset();
    this->rxWaiting = false;
    this->txWaiting = false;
}

template<size_t L>
size_t EPBuffer<L>::push(const void *d, size_t len)
{
    usb_disable_interrupts();
    size_t w = min(this->sendSpace(), len);
    const uint8_t* d8 = (const uint8_t*)d;
    for (size_t i = 0; i < w; i++) {
        *this->p++ = *d8++;
    }
    assert(this->p >= this->buf);
    auto doflush = (this->sendSpace() == 0);
    usb_enable_interrupts();
    if (doflush) {
        this->flush();
    }
    return w;
}

template<size_t L>
size_t EPBuffer<L>::pop(void* d, size_t len)
{
    usb_disable_interrupts();
    size_t r = min(this->available(), len);
    uint8_t* d8 = (uint8_t*)d;
    for (size_t i = 0; i < r; i++) {
        *d8++ = *this->p++;
    }
    assert(this->p <= this->tail);

    if (this->available() == 0) {
        this->enableOutEndpoint();
    }
    usb_enable_interrupts();
    return r;
}

// Must be called with interrupts disabled
template<size_t L>
void EPBuffer<L>::reset()
{
    this->p = this->buf;
    this->tail = this->buf;
}

// Must be called with interrupts disabled
template<size_t L>
size_t EPBuffer<L>::len()
{
    assert(this->p >= this->buf);
    return this->p - this->buf;
}

// Must be called with interrupts disabled
template<size_t L>
size_t EPBuffer<L>::available()
{
    assert(this->p <= this->tail);
    return this->tail - this->p;
}

// Must be called with interrupts disabled
template<size_t L>
size_t EPBuffer<L>::sendSpace()
{
    return L - this->len();
}

template<size_t L>
void EPBuffer<L>::flush()
{
    usb_disable_interrupts();
    // Don't flush an empty buffer
    if (this->len() == 0) {
        usb_enable_interrupts();
        return;
    }
    /*
     * Bounce out if a flush is already occurring. This is only
     * possible when ‘flush’ is called from an interrupt, so the
     * check-and-set must be done with interrupts disabled.
     *
     * This flag is still necessary, because interrupts can get
     * reenabled while waiting to transmit.
     */
    if (this->currentlyFlushing) {
        usb_enable_interrupts();
        return;
    }
    this->currentlyFlushing = true;
    USBCore().logEP('_', this->ep, '>', this->len());

    // Only attempt to send if the device is configured enough.
    switch (USBCore().usbDev().cur_status) {
    case USBD_DEFAULT:
    case USBD_ADDRESSED:
        if (this->ep != 0) {
            break;
        }
    // fall through
    case USBD_CONFIGURED:
    case USBD_SUSPENDED: {
        // This will temporarily reenable and disable interrupts
        auto canWrite = this->waitForWriteComplete();
        if (canWrite) {
            // In case of an uncaught reset or configuration event
            if (this->len() == 0) {
                break;
            }
            // Only start the next transmission if the device hasn't been
            // reset.
            this->txWaiting = true;
            usbd_ep_send(&USBCore().usbDev(), this->ep, (uint8_t *)this->buf, this->len());
            USBCore().logEP('>', this->ep, '>', this->len());
        }
        break;
    }
    default:
        break;
    }
    this->reset();
    this->currentlyFlushing = false;
    usb_enable_interrupts();
}

// Must be called with interrupts disabled
template<size_t L>
void EPBuffer<L>::enableOutEndpoint()
{
    // Don’t attempt to read from the endpoint buffer until it’s
    // ready.
    if (this->rxWaiting) return;
    this->rxWaiting = true;

    this->reset();
    usb_transc_config(&USBCore().usbDev().transc_out[this->ep],
                      (uint8_t*)this->buf, sizeof(this->buf), 0);
    USBCore().usbDev().drv_handler->ep_rx_enable(&USBCore().usbDev(), this->ep);
}

// Must be called via ISR
template<size_t L>
void EPBuffer<L>::transcOut()
{
    this->tail = this->buf + USBCore().usbDev().transc_out[this->ep].xfer_count;;
    this->rxWaiting = false;
}

// Must be called via ISR
template<size_t L>
void EPBuffer<L>::transcIn()
{
    this->txWaiting = false;
}

// Unused?
template<size_t L>
uint8_t* EPBuffer<L>::ptr()
{
    return this->buf;
}

// Busy loop until the latest IN packet has been sent. Returns ‘true’
// if a new packet can be queued when this call completes.
// Must run with interrupts disabled, which will be temporarily reenabled
template<size_t L>
bool EPBuffer<L>::waitForWriteComplete()
{
    // auto start = getCurrentMillis();
    auto ok = true;
    do {
        usb_enable_interrupts();
        switch (USBCore().usbDev().cur_status) {
        case USBD_DEFAULT:
        case USBD_ADDRESSED:
            ok = false;
            break;
        default:
            break;
        }
        // if (getCurrentMillis() - start > 5) {
        //     EPBuffers().buf(ep).transcIn();
        //     ok = false;
        // }
        usb_disable_interrupts();
    } while (ok && this->txWaiting);
    return ok;
}

template<size_t L, size_t C>
EPBuffers_<L, C>::EPBuffers_()
{
    this->init();
}

template<size_t L, size_t C>
void EPBuffers_<L, C>::init()
{
    for (uint8_t ep = 0; ep < C; ep++) {
        this->buf(ep).init(ep);
    }
}

template<size_t L, size_t C>
EPBuffer<L>& EPBuffers_<L, C>::buf(uint8_t ep)
{
    return this->epBufs[ep];
}

template<size_t L, size_t C>
EPDesc* EPBuffers_<L, C>::desc(uint8_t ep)
{
    assert(ep < C);
    static EPDesc descs[C];
    return &descs[ep];
}

EPBuffers_<USB_EP_SIZE, EP_COUNT>& EPBuffers()
{
    static EPBuffers_<USB_EP_SIZE, EP_COUNT> obj;
    return obj;
}

class ClassCore
{
    public:
        static usb_class *structPtr()
        {
            static usb_class rc = {
                .req_cmd     = 0xff,
                .req_altset  = 0x0,
                .init        = ClassCore::init,
                .deinit      = ClassCore::deinit,
                .req_process = ClassCore::reqProcess,
                .ctlx_in     = ClassCore::ctlIn,
                .ctlx_out    = ClassCore::ctlOut,
                .data_in     = ClassCore::dataIn,
                .data_out    = ClassCore::dataOut
            };
            return &rc;
        }

        // Called after device configuration is set.
        static uint8_t init(usb_dev* usbd, uint8_t config_index)
        {
            (void)config_index;

            /*
             * Endpoint 0 is configured during startup, so skip it and only
             * handle what’s configured by ‘PluggableUSB’.
             */
            uint32_t buf_offset = EP0_RX_ADDR + USB_EP_SIZE;
            for (uint8_t ep = 1; ep < PluggableUSB().epCount(); ep++) {
                auto desc = *(EPDesc*)epBuffer(ep);
                usb_desc_ep ep_desc = {
                    .header = {
                        .bLength = sizeof(ep_desc),
                        .bDescriptorType = USB_DESCTYPE_EP,
                    },
                    .bEndpointAddress = (uint8_t)(desc.dir() | ep),
                    .bmAttributes = desc.type(),
                    .wMaxPacketSize = desc.maxlen(),
                    .bInterval = 0
                };
                // Don’t overflow the hardware buffer table.
                assert((buf_offset + ep_desc.wMaxPacketSize) <= 512);

                // Reinit EPBuffer, in case a packet got queued after reset
                // but before configuration
                EPBuffers().buf(ep).init(ep);
                usbd->ep_transc[ep][TRANSC_IN] = USBCore_::transcInHelper;
                usbd->ep_transc[ep][TRANSC_OUT] = USBCore_::transcOutHelper;
                usbd->drv_handler->ep_setup(usbd, EP_BUF_SNG, buf_offset, &ep_desc);

                /*
                 * Allow data to come in to OUT buffers immediately, as it
                 * will be copied out as it comes in.
                 */
                if (desc.dir() == 0) {
                    EPBuffers().buf(ep).enableOutEndpoint();
                }

                buf_offset += ep_desc.wMaxPacketSize;
            }
            return USBD_OK;
        }

        // Called when SetConfiguration setup packet sets the
        // configuration to 0.
        static uint8_t deinit(usb_dev* usbd, uint8_t config_index)
        {
            (void)usbd;
            (void)config_index;
            return USBD_OK;
        }

        // Called when ep0 gets a SETUP packet after configuration.
        static uint8_t reqProcess(usb_dev* usbd, usb_req* req)
        {
            USBCore().logStatus("ClassCore");
            (void)usbd;

            // TODO: remove this copy.
            arduino::USBSetup setup;
            memcpy(&setup, req, sizeof(setup));
            USBCore().setupClass(req->wLength);
            if (setup.bRequest == USB_GET_DESCRIPTOR) {
                auto sent = PluggableUSB().getDescriptor(setup);
                if (sent > 0) {
                    USBCore().flush(0);
                } else if (sent < 0) {
                    return REQ_NOTSUPP;
                }
            } else if ((setup.bmRequestType & USB_RECPTYPE_MASK) == USB_RECPTYPE_EP) {
                uint8_t ep = EP_ID(setup.wIndex);
                // Reset endpoint state on ClearFeature(EndpointHalt)
                EPBuffers().buf(ep).init(ep);
                return REQ_SUPP;
            } else {
#ifdef USBD_USE_CDC
                if (CDCACM().setup(setup))
                    return REQ_SUPP;
#endif
                if (PluggableUSB().setup(setup)) {
                    return REQ_SUPP;
                }

                return REQ_NOTSUPP;
            }

            return REQ_SUPP;
        }

        // Called when ep0 is done sending all data from an IN stage.
        static uint8_t ctlIn(usb_dev* usbd)
        {
            (void)usbd;
            return REQ_SUPP;
        }

        // Called when ep0 is done receiving all data from an OUT stage.
        static uint8_t ctlOut(usb_dev* usbd)
        {
            USBCore().ctlOut(usbd);
            return REQ_SUPP;
        }

        // Appears to be unused in usbd library, but used in usbfs.
        static void dataIn(usb_dev* usbd, uint8_t ep)
        {
            (void)usbd;
            (void)ep;
            return;
        }

        // Appears to be unused in usbd library, but used in usbfs.
        static void dataOut(usb_dev* usbd, uint8_t ep)
        {
            (void)usbd;
            (void)ep;
            return;
        }
};

void (*oldResetHandler)(usb_dev *usbd);
void handleReset(usb_dev *usbd)
{
    USBCore().logStatus("Reset");
    USBCore().nreset++;
    USBCore().setupClass(0);
    EPBuffers().init();
    oldResetHandler(usbd);
}

#ifdef USBD_ERROR_HOOKS
static void handleSetupErr(uint8_t len)
{
    USBCore().logEP('!', 0, '<', len);
    usbd_ep_ram *btable_ep = (usbd_ep_ram *)(USBD_RAM + 2 * (BTABLE_OFFSET & 0xFFF8));
    uint32_t *p = (uint32_t *)(USBD_RAM + 2U * EP0_RX_ADDR);
    uint16_t buf[USBD_EP0_MAX_SIZE/2];
    for (int i = 0; i < USBD_EP0_MAX_SIZE/2; i++) {
        buf[i] = p[i];
    }
    USBCore().hexDump('!', (uint8_t *)buf, USBD_EP0_MAX_SIZE);
    auto bytes = (uint16_t)(btable_ep[0].rx_count & EPRCNT_CNT);
    USBCore().logEP('.', 0, '!', bytes);
}

static void handleErr()
{
    USBCore().logStatus("Error");
    USBCore().nerror++;
}
#endif

void (*oldSuspendHandler)();
void handleSuspend()
{
    USBCore().logStatus("Suspend");
    USBCore().nsusp++;
    oldSuspendHandler();
}

void (*oldResumeHandler)();
void handleResume()
{
    usb_disable_interrupts();
    USBCore().logStatus("Resume");
    usb_enable_interrupts();
    USBCore().nresume++;
    oldResumeHandler();
}

USBCore_::USBCore_()
{
#ifdef USBCORE_TRACE
    Serial1.begin(115200);
#endif
    /*
     * Use global ‘usbd’ here, instead of wrapped version, to avoid
     * initialization loop.
     */
    usb_init(&desc, ClassCore::structPtr());
    usbd.user_data = this;

    oldResetHandler = usbd.drv_handler->ep_reset;
    usbd.drv_handler->ep_reset = handleReset;

#ifdef USBD_ERROR_HOOKS
    usbd.drv_handler->err = handleErr;
    usbd.drv_handler->setup_err = handleSetupErr;
#endif

    oldSuspendHandler = usbd.drv_handler->suspend;
    usbd.drv_handler->suspend = handleSuspend;

    oldResumeHandler = usbd.drv_handler->suspend_leave;
    usbd.drv_handler->suspend_leave = handleResume;

    this->oldTranscSetup = usbd.ep_transc[0][TRANSC_SETUP];
    usbd.ep_transc[0][TRANSC_SETUP] = USBCore_::transcSetupHelper;

    this->oldTranscOut = usbd.ep_transc[0][TRANSC_OUT];
    usbd.ep_transc[0][TRANSC_OUT] = USBCore_::transcOutHelper;

    this->oldTranscIn = usbd.ep_transc[0][TRANSC_IN];
    usbd.ep_transc[0][TRANSC_IN] = USBCore_::transcInHelper;
}

void USBCore_::logEP(char kind, uint8_t ep, char dir, size_t len)
{
#ifdef USBCORE_TRACE
    Serial1.print(USBD_EPxCS(ep), 16);
    Serial1.print(kind);
    Serial1.print(ep);
    Serial1.print(dir);
    Serial1.print(len);
    Serial1.print(' ');
    Serial1.println(USBD_EPxCS(ep), 16);
    Serial1.flush();
#endif
}

void USBCore_::hexDump(char prefix, const uint8_t *buf, size_t len)
{
#ifdef USBCORE_TRACE
    Serial1.print(prefix);
    for (size_t i = 0; i < len; i++) {
        Serial1.print(buf[i] >> 4, 16);
        Serial1.print((buf[i] & 0x0f), 16);
        Serial1.print(' ');
    }
    Serial1.println();
    Serial1.flush();
#endif
}

void USBCore_::logStatus(const char *status)
{
#ifdef USBCORE_TRACE
    Serial1.println(status);
    Serial1.flush();
#endif
}

void USBCore_::connect()
{
    USBCore().buildDeviceConfigDescriptor();
    usb_connect();
}

void USBCore_::disconnect()
{
    usb_disconnect();
}

void USBCore_::setupClass(uint16_t wLength)
{
    this->ctlIdx = 0;
    this->ctlOutBuf = NULL;
    this->ctlOutLen = 0;
    this->maxWrite = wLength;
    auto usbd = &USBCore().usbDev();
    usb_transc_config(&usbd->transc_in[0], NULL, 0, 0);
    usb_transc_config(&usbd->transc_out[0], NULL, 0, 0);
}

// Send ‘len’ octets of ‘d’ through the control pipe (endpoint 0).
// Configures the low-level API's transfer buffer if TRANSFER_RELEASE
// is set, or when flushed.
//
// Limitations: There is a fixed maximum buffer size of CTL_BUFSZ, which
// must be adjusted per-application, in an attempt to avoid dynamic
// allocation.
//
// Returns the number of octets sent, or -1 on error.
//
// Must be called via ISR, or when the endpoint isn't in VALID status.
int USBCore_::sendControl(uint8_t flags, const void* data, int len)
{
    USBCore().logEP('+', 0, '>', len);

    uint8_t* d = (uint8_t*)data;
    auto usbd = &USBCore().usbDev();
    auto l = min(len, this->maxWrite);
    assert(l <= CTL_BUFSZ - this->ctlIdx);
    if (flags & TRANSFER_ZERO) {
        memset(&this->ctlBuf[this->ctlIdx], 0, l);
    } else {
        memcpy(&this->ctlBuf[this->ctlIdx], data, l);
    }
    ctlIdx += l;
    this->maxWrite -= l;
    if ((l != 0) && (flags & TRANSFER_RELEASE)) {
        USBCore().flush(0);
    }

    // Return ‘len’, rather than ‘wrote’, because PluggableUSB
    // calculates descriptor sizes by first having them write to an
    // empty buffer (setting ‘this->maxWrite’ to 0). To accomodate
    // that, we always just pretend we sent the entire buffer.
    //
    // TODO: this may cause issues when /actually/ sending buffers
    // larger than ‘this->maxWrite’, since we will have claimed to
    // send more data than we did.
    return len;
}

// Configure a Control receive (Data OUT stage) buffer
//
// Limitations: data must point to storage that has a lifetime long
// enough to be written to by the completion handler ctlOut. Only a
// single buffer can be used per control transfer.
//
// Must be called via ISR, or when the endpoint isn't in VALID status.
int USBCore_::recvControl(void* data, int len)
{
    this->ctlOutBuf = (uint8_t *)data;
    this->ctlOutLen = len;
    usb_transc_config(&USBCore().usbDev().transc_out[0], this->ctlBuf, len, 0);
    USBCore().logEP('_', 0, '<', len);
    return len;
}

// Completion handler for Control Data OUT stage.
//
// Copies the read data into the destination set by recvControl.
void USBCore_::ctlOut(usb_dev* usbd)
{
    auto transc = &usbd->transc_out[0];
    if (this->ctlOutBuf) {
        USBCore().hexDump('<', this->ctlBuf, transc->xfer_count);

        memcpy(this->ctlOutBuf, this->ctlBuf, this->ctlOutLen);
        this->ctlOutBuf = NULL;
    }
}

// TODO: no idea? this isn’t in the avr 1.8.2 library, although it has
// the function prototype.
int USBCore_::recvControlLong(void* data, int len)
{
    (void)data;
    (void)len;
    return -1;
}

// Number of octets available on OUT endpoint.
uint8_t USBCore_::available(uint8_t ep)
{
    usb_disable_interrupts();
    auto r =  EPBuffers().buf(ep).available();
    usb_enable_interrupts();
    return r;
}

// Space left in IN endpoint buffer.
uint8_t USBCore_::sendSpace(uint8_t ep)
{
    usb_disable_interrupts();
    auto r = EPBuffers().buf(ep).sendSpace();
    usb_enable_interrupts();
    return r;
}

// Blocking send of data to an endpoint. Returns the number of octets
// sent, or -1 on error.
int USBCore_::send(uint8_t ep, const void* data, int len)
{
    uint8_t* d = (uint8_t*)data;
    // Top nybble is used for flags.
    auto flags = ep & 0xf0;
    ep &= 0x7;
    auto wrote = 0;
    auto usbd = &USBCore().usbDev();

    usb_disable_interrupts();
    // USBCore().logEP('+', ep, '>', len);
    usb_enable_interrupts();
#ifdef USBD_REMOTE_WAKEUP
    usb_disable_interrupts();
    if (usbd->cur_status == USBD_SUSPENDED && usbd->pm.remote_wakeup) {
        USBCore().logStatus("Remote wakeup");
        usb_enable_interrupts();
        usbd_remote_wakeup_active(usbd);
    } else {
        usb_enable_interrupts();
    }
#endif
    // Make sure any transactions made outside of PluggableUSB are
    // cleaned up.
    auto transc = &usbd->transc_in[ep];
    usb_transc_config(transc, nullptr, 0, 0);

    // TODO: query the endpoint for its max packet length.
    while (wrote < len) {
        auto w = 0;
        auto toWrite = len - wrote;
        if (flags & TRANSFER_ZERO) {
            // TODO: handle writing zeros instead of ‘d’.
            return -1;
        } else {
            w = EPBuffers().buf(ep).push(d, toWrite);
        }
        d += w;
        wrote += w;
    }

    if (flags & TRANSFER_RELEASE) {
        this->flush(ep);
    }

    return wrote;
}

// Non-blocking receive. Returns the number of octets read, or -1 on
// error.
int USBCore_::recv(uint8_t ep, void* data, int len)
{
    uint8_t* d = (uint8_t*)data;
    return EPBuffers().buf(ep).pop(d, len);
}

// Receive one octet from OUT endpoint ‘ep’. Returns -1 if no bytes
// available.
int USBCore_::recv(uint8_t ep)
{
    uint8_t c;
    auto rc = this->recv(ep, &c, sizeof(c));
    if (rc < 0) {
        return rc;
    } else if (rc == 0) {
        return -1;
    }
    return c;
}

// Flushes an outbound transmission as soon as possible.
int USBCore_::flush(uint8_t ep)
{
    if (ep == 0) {
        auto usbd = &USBCore().usbDev();
        usbd->transc_in[0].xfer_buf = ctlBuf;
        usbd->transc_in[0].xfer_len = ctlIdx;
        USBCore().logEP('_', 0, '>', ctlIdx);
        // USBCore().hexDump('>', ctlBuf, ctlIdx);
    } else {
        EPBuffers().buf(ep).flush();
    }
    return 0;
}

void USBCore_::transcSetupHelper(usb_dev* usbd, uint8_t ep)
{
    USBCore_* core = (USBCore_*)usbd->user_data;
    core->transcSetup(usbd, ep);
}

void USBCore_::transcOutHelper(usb_dev* usbd, uint8_t ep)
{
    USBCore_* core = (USBCore_*)usbd->user_data;
    core->transcOut(usbd, ep);
}

void USBCore_::transcInHelper(usb_dev* usbd, uint8_t ep)
{
    USBCore_* core = (USBCore_*)usbd->user_data;
    core->transcIn(usbd, ep);
}

usb_dev& USBCore_::usbDev()
{
    return usbd;
}

/* Log the raw Setup stage data packet */
void USBCore_::transcSetup(usb_dev* usbd, uint8_t ep)
{
    (void)ep;
#ifdef USBCORE_TRACE
    Serial1.println((uint16_t)USBD_EPxCS(0), 16);
#endif
    USBCore().hexDump('^', (uint8_t *)&usbd->control.req, USB_SETUP_PACKET_LEN);

    this->oldTranscSetup(usbd, ep);
}

// Called in interrupt context.
void USBCore_::transcOut(usb_dev* usbd, uint8_t ep)
{
    auto transc = &usbd->transc_out[ep];
    auto count = transc->xfer_count;
    USBCore().logEP(':', ep, '<', count);
    if (ep == 0) {
        this->oldTranscOut(usbd, ep);
    } else {
        EPBuffers().buf(ep).transcOut();
    }
    USBCore().logEP('.', ep, '<', count);
}

// Called in interrupt context.
void USBCore_::transcIn(usb_dev* usbd, uint8_t ep)
{
    auto transc = &usbd->transc_in[ep];
    USBCore().logEP(':', ep, '>', transc->xfer_count);
    if (ep == 0) {
        this->oldTranscIn(usbd, ep);
    } else {
        EPBuffers().buf(ep).transcIn();
    }
    if (usbd->control.ctl_state != USBD_CTL_STATUS_OUT) {
        USBCore().logEP('.', ep, '>', transc->xfer_count);
    }
    transc->xfer_count = 0;
}

void USBCore_::buildDeviceConfigDescriptor()
{
    this->ctlIdx = 0;
    this->maxWrite = 0;
    uint8_t interfaceCount = 0;
    uint16_t len = 0;
#ifdef USBD_USE_CDC
    interfaceCount += 2;
    len += CDCACM().getInterface();
#endif
    len += PluggableUSB().getInterface(&interfaceCount);

    configDesc.wTotalLength = sizeof(configDesc) + len;
    configDesc.bNumInterfaces = interfaceCount;
    this->maxWrite = this->CTL_BUFSZ;
    this->ctlIdx = 0;
    this->sendControl(0, &configDesc, sizeof(configDesc));
    interfaceCount = 0;
#ifdef USBD_USE_CDC
    interfaceCount += 2;
    CDCACM().getInterface();
#endif
    PluggableUSB().getInterface(&interfaceCount);
    memcpy(this->cfgDesc, this->ctlBuf, this->ctlIdx);
    USBCore().usbDev().desc->config_desc = this->cfgDesc;
}

void USBCore_::sendZLP(usb_dev* usbd, uint8_t ep)
{
    usbd->drv_handler->ep_write(nullptr, ep, 0);
}

USBCore_& USBCore()
{
    static USBCore_ core;
    return core;
}

// -> returns a pointer to the Nth element of the EP buffer structure
void* epBuffer(unsigned int n)
{
    return (void*)EPBuffers().desc(n);
}

bool USBCore_::isSuspended()
{
    return USBCore().usbDev().cur_status == USBD_SUSPENDED;
}
#endif
