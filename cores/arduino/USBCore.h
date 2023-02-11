#pragma once
#include "Arduino.h"
#include "USBDefs.h"

extern "C" {
#include "usbd_core.h"
#include "usb_ch9_std.h"
}

/*
 * Default size of fixed-size control transfer buffer. Application can
 * redefine at compile time. This is set to 255 due to a bug in the
 * vendor firmware that only reads a single byte for config descriptor
 * set total length.
 */
#ifndef USBCORE_CTL_BUFSZ
#define USBCORE_CTL_BUFSZ 255
#endif

/*
 * Default speed for trace logging serial port
 */
#ifndef USBCORE_TRACE_SPEED
#define USBCORE_TRACE_SPEED 230400
#endif

/*
 * Descriptor for storing an endpoint’s direction, type, and max
 * packet length.
 */
union EPDesc {
    struct {
        uint8_t maxlen;
        unsigned int type:3;
        unsigned int dir:5;
    } parts;
    unsigned int val;

    // Default descriptor, in case, I dunno, you need to initialize an
    // array or something.
    constexpr EPDesc() : EPDesc(USB_TRX_OUT, USB_EP_ATTR_CTL, USB_EP_SIZE) {}

    // Encode a direction, type, and max packet length in an endpoint
    // descriptor.
    constexpr EPDesc(uint8_t dir, uint8_t type) : EPDesc(dir, type, USB_EP_SIZE) {}

    // Encode a direction and type in an endpoint descriptor, using
    // the device’s max packet length as the endpoint’s.
    constexpr EPDesc(uint8_t dir, uint8_t type, uint8_t maxlen) : val((dir|type) << 8 | maxlen) {}

    // Extract the direction from an endpoint descriptor.
    constexpr uint8_t dir() {
        return this->parts.dir << 3;
    }

    // Extract the type from an endpoint descriptor.
    constexpr uint8_t type() {
        return this->parts.type;
    }

    // Extract the max packet length from an endpoint descriptor.
    constexpr uint8_t maxlen() {
        return this->parts.maxlen;
    }
};

/*
 * Mappings from Arduino USB API to USBCore singleton functions.
 */
#define TRANSFER_PGM     0x80
#define TRANSFER_ZERO    0x20
#define TRANSFER_RELEASE 0x40

#define USB_SendControl     USBCore().sendControl
#define USB_RecvControl     USBCore().recvControl
#define USB_RecvControlLong USBCore().recvControlLong
#define USB_Available       USBCore().available
#define USB_SendSpace       USBCore().sendSpace
#define USB_Send            USBCore().send
#define USB_Recv            USBCore().recv
#define USB_Flush           USBCore().flush

template<size_t L>
class EPBuffer
{
    public:
        void init(uint8_t ep);

        size_t push(const void* d, size_t len);
        size_t pop(void* d, size_t len);
        void reset();
        size_t len();
        size_t available();
        size_t sendSpace();
        void flush();
        uint8_t* ptr();
        void enableOutEndpoint();
        void setTimeout(uint16_t timeout);

        void transcIn();
        void transcOut();

        /*
         * Busy loop until the endpoint has finished its current
         * transmission.
         */
        bool waitForWriteComplete();

        /*
         * Flag for whether we are waiting for data from the host.
         *
         * If this is ‘true’, there is no data available in the
         * peripheral's SRAM from the host for this endpoint.
         */
        volatile bool rxWaiting = true;

        /*
         * Flag for whether the current transmission is complete.
         *
         * If this is ‘true’, the USB peripheral is currently sending
         * the contents of this endpoint's SRAM to the host, and it is
         * not safe to start a new transmission.
         */
        volatile bool txWaiting = false;
    private:
        volatile uint8_t buf[L];
        volatile uint8_t* tail = buf;
        volatile uint8_t* p = buf;

        /*
         * Prevent more than one simultaneous call to ‘flush’.
         */
        volatile bool currentlyFlushing = false;

        volatile uint32_t startTime;
        uint16_t timeout;
        volatile bool timedOut;

        uint8_t ep;
};

template<size_t L, size_t C>
class EPBuffers_
{
    public:
        EPBuffers_();
        void init();

        EPBuffer<L>& buf(uint8_t ep);

        static EPDesc* desc(uint8_t ep);

    private:
        EPBuffer<L> epBufs[C];
};

EPBuffers_<USBD_EP0_MAX_SIZE, EP_COUNT>& EPBuffers();

class USBCore_
{
    public:
        USBCore_();

        void connect();
        void disconnect();
        bool isSuspended();
        bool configured();

        /*
         * PluggableUSB interface.
         */
        int sendControl(uint8_t flags, const void* data, int len);
        int recvControl(void* data, int len);
        int recvControlLong(void* data, int len);
        uint8_t available(uint8_t ep);
        uint8_t sendSpace(uint8_t ep);
        int send(uint8_t ep, const void* data, int len);
        int recv(uint8_t ep, void* data, int len);
        int recv(uint8_t ep);
        int flush(uint8_t ep);
        void setResetHook(void (*hook)());
        void setTimeout(uint8_t ep, uint16_t timeout);

        // Debug counters
        volatile uint16_t nreset;
        volatile uint16_t nsusp;
        volatile uint16_t nresume;
        volatile uint16_t nerror;

        uint8_t setupCtlOut(usb_req* req);
        void setupClass(uint16_t wLength);
        void ctlOut(usb_dev* udev);

        void logEP(char kind, uint8_t ep, char dir, size_t len);
        void hexDump(char prefix, const uint8_t *buf, size_t len);
        void logStatus(const char *status);
        /*
         * Static member function helpers called from ISR.
         *
         * These pull the core handle from ‘usbd’ and use it to call the
         * instance member functions.
         */
        static void transcSetupHelper(usb_dev* usbd, uint8_t ep);
        static void transcOutHelper(usb_dev* usbd, uint8_t ep);
        static void transcInHelper(usb_dev* usbd, uint8_t ep);

        void buildDeviceConfigDescriptor();

        /*
         * Shadow the global ‘usbd’, which shouldn't be used directly,
         * so that all access is mediated by this class, ensuring all
         * access to it happens after this class is constructed (which
         * initializes ‘usbd’).
         */
        usb_dev& usbDev();
    private:
        // TODO: verify that this only applies to the control endpoint’s use of wLength
        // I think this is only on the setup packet, so it should be fine.
        uint16_t maxWrite = 0;

#define USBCORE_EP0_ROUND(n) \
    (USBD_EP0_MAX_SIZE * (n + (USBD_EP0_MAX_SIZE - 1)) / USBD_EP0_MAX_SIZE)

        /*
         * Fixed size buffer for control transfers, rounded up to a multiple
         * of the EP0 packet size, because low-level firmware doesn't properly
         * limit write length to transc->xfer_len for control writes.
         */
        uint8_t ctlBuf[USBCORE_EP0_ROUND(USBCORE_CTL_BUFSZ)];
        // Next index in ctlBuf to be written to or read from
        size_t ctlIdx;
        // Transfer size for setCtlOutDest
        size_t ctlOutLen;

        uint8_t cfgDesc[USBCORE_CTL_BUFSZ];

        /*
         * Pointers to the transaction routines specified by ‘usbd_init’.
         */
        void (*oldTranscSetup)(usb_dev* usbd, uint8_t ep);
        void (*oldTranscOut)(usb_dev* usbd, uint8_t ep);
        void (*oldTranscIn)(usb_dev* usbd, uint8_t ep);

        void transcSetup(usb_dev* usbd, uint8_t ep);
        void transcOut(usb_dev* usbd, uint8_t ep);
        void transcIn(usb_dev* usbd, uint8_t ep);

        void sendZLP(usb_dev* usbd, uint8_t ep);
};

USBCore_& USBCore();
