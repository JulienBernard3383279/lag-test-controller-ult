#include "usb/logic.hpp"
#include "usb/protocol.hpp"

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "pico/util/queue.h"

#include "global.hpp"

#include <array>
#include <functional>

#include "hardware/structs/systick.h"
#include "hardware/adc.h"
#include "hardware/regs/rosc.h"

#include "string.h"

#include "joybus.hpp"

void local_ep_in_handler(uint8_t *buf, uint16_t len);
void local_ep_out_handler(uint8_t *buf, uint16_t len);
void local_main();
void core1_entry();

const uint16_t descriptor_strings_len = 3;
const char *descriptor_strings[descriptor_strings_len] = {
        "Arte",
        "Lag test controller ult",
        "1" // The "release number"
};

const int inReportSize = 64;
const int outReportSize = 64;

//uint8_t hidReportDescriptor[1] = { 0 };

#define NUMBER_OF_READINGS_PER_TEST 1024

// 2048 * 2 bytes = toute la stack (4KB par core)

void await_time32us(uint32_t target) {
    while ( (time_us_32() - target) & (1 << 31) );
}

void enterMode() {

    USBConfiguration usbConfiguration =
    {
        .inEpMaxPacketSize = inReportSize,
        .inEpActualPacketSize = inReportSize,
        .outEpMaxPacketSize = outReportSize,
        .epOutId = 2,
        .descriptorStrings = descriptor_strings,
        .descriptorStringsLen = descriptor_strings_len,
        .hid = false,
        .bcdHID = 0x0110,
        .hidReportDescriptor = nullptr,
        .hidReportDescriptorLen = 0,
        .useWinUSB = true,
        .VID = 0xA57E,
        .PID = 0x0002,
        .bcdDevice = 0x100,

        .ep_in_handler = local_ep_in_handler,
        .ep_out_handler = local_ep_out_handler
    };

    initMode(usbConfiguration);

    multicore_launch_core1(core1_entry);

    local_main();
}

volatile bool transferHappened = true;
void local_ep_in_handler(uint8_t *buf, uint16_t len) {
    transferHappened = true;
}

void local_ep_out_handler(uint8_t *buf, uint16_t len) {
}

/* Lag test controller ult mode
*
* Core 1: acts as a controller; initially select port 1 and accept, wait for a few secs then start the lag test (set a flag in the bridge)
* Lag test: joybus mode, with the function checking the bridge's l flag to know if it should set L and analog L
* 
* Core 0 : lag test: wait for X + random(0, 1frame) then set L and analog L in the bridge; start the phototransistor reading then ; then start sending over USB
*/

enum class State {
    NONE,
    UP,
    A,
    LandAnalogL
};

struct Bridge {
    volatile State state = State::NONE;
}; 
Bridge bridge;

uint32_t rnd(void){
    int k, random=0;
    volatile uint32_t *rnd_reg=(uint32_t *)(ROSC_BASE + ROSC_RANDOMBIT_OFFSET);
    
    for(k=0;k<32;k++){
    
    random = random << 1;
    random=random + (0x00000001 & (*rnd_reg));

    }
    return random;
}

void local_main() {
    std::array<uint8_t, 64> full0 {};
    std::array<uint16_t, NUMBER_OF_READINGS_PER_TEST> array;

    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);

    multicore_fifo_push_blocking((uintptr_t)&bridge);

    // Sequence of controller selection

    // Wait 1s
    // Up 0.5s
    // Wait 0.2s
    // A 0.2s
    // Wait 0.2s
    // A 0.2s
    // Wait 2s

    {
        sleep_ms(2000);
        bridge.state = State::A;
        sleep_ms(200);
        bridge.state = State::NONE;
        sleep_ms(200);
        bridge.state = State::A;
        sleep_ms(200);
        bridge.state = State::UP;
        sleep_ms(300);
        bridge.state = State::NONE;
        sleep_ms(200);
        bridge.state = State::A;
        sleep_ms(200);
        bridge.state = State::NONE;
        sleep_ms(200);
        bridge.state = State::A;        
        sleep_ms(2000);
        bridge.state = State::NONE;
    }

    uint32_t target = time_us_32();
    uint32_t awaitTransferTimestamp = 0;

    gpio_init(18);

    while (true) {
        // Wait
        
        /*if (false) {
            __stateLabel_awaitNewTest:
            while (!transferHappened);
            target = time_us_32();
        }*/

        /*target += 1'000'000;
        await_time32us(target);*/

        sleep_ms(750);
        sleep_us(rnd() % 16'667);
        bridge.state = State::LandAnalogL;
        gpio_set_dir(18, GPIO_OUT);
        gpio_put(18, 0);

        // Read
        uint32_t timestamp = time_us_32();
        std::array<uint16_t, NUMBER_OF_READINGS_PER_TEST> array;
        for (int i = 0; i<array.size(); i++) {
            if (i == 35*5) {
                bridge.state = State::NONE; // Turn off the shield after 35ms
                gpio_put(18, 1);
                //gpio_set_dir(18, GPIO_IN);
            }
            while (time_us_32() - timestamp <= 200*i); // 200 <=> every 0.2ms
            array[i] = adc_read();
        }

        // Report
        //awaitTransferTimestamp = time_us_32();
        while (!transferHappened) {
            //if (time_us_32() - awaitTransferTimestamp >= 10'000'000) goto __stateLabel_awaitNewTest;
        }
        usb_start_transfer_in_ep((uint8_t*) &full0, 64);
        gpio_put(25, 1);
        transferHappened = false;
        for (int i = 0; i < NUMBER_OF_READINGS_PER_TEST*2/64; i++) {
            //awaitTransferTimestamp = time_us_32();
            while (!transferHappened) {
                //if (time_us_32() - awaitTransferTimestamp >= 10'000'000) goto __stateLabel_awaitNewTest;
            }
            transferHappened = false;
            usb_start_transfer_in_ep(((uint8_t*) &(array[0]) +i*64), 64);
        }
    }
    
}

void core1_entry() {
    set_sys_clock_khz(us*1000, true);


    Bridge* localBridge = (Bridge*) multicore_fifo_pop_blocking();
    std::function<GCReport()> callback = [localBridge](){ // Quand on passe des choses à un lambda on ne peut plus le convertir en pointeur vers fonction //TODO
        GCReport gcReport = defaultGcReport;
        State state = localBridge->state;
        switch (state) {
        case State::NONE:
            break;
        case State::UP:
            gcReport.yStick = 208;
            break;
        case State::A:
            gcReport.a = 1;
            break;
        case State::LandAnalogL:
            gcReport.l = 1;
            gcReport.analogL = 200;
            break;
        }
        return gcReport;
    };

    CommunicationProtocols::Joybus::enterMode(22, callback);    
}