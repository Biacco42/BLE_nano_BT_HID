/* mbed Microcontroller Library
 * Copyright (c) 2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "HIDServiceBase.h"
#include "CircularBuffer.h"

Serial pc(USBTX, USBRX);

/**
 * Report descriptor for a standard 101 keys keyboard, following the HID specification example:
 * - 8 bytes input report (1 byte for modifiers and 6 for keys)
 * - 1 byte output report (LEDs)
 */
const report_map_t KEYBOARD_REPORT_MAP = {
    USAGE_PAGE(1),      0x01,       // Generic Desktop Ctrls
    USAGE(1),           0x06,       // Keyboard
    COLLECTION(1),      0x01,       // Application
        REPORT_ID(1),       0x01,
        USAGE_PAGE(1),      0x07,       //   Kbrd/Keypad
        USAGE_MINIMUM(1),   0xE0,
        USAGE_MAXIMUM(1),   0xE7,
        LOGICAL_MINIMUM(1), 0x00,
        LOGICAL_MAXIMUM(1), 0x01,
        REPORT_SIZE(1),     0x01,       //   1 byte (Modifier)
        REPORT_COUNT(1),    0x08,
        INPUT(1),           0x02,       //   Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position

        REPORT_COUNT(1),    0x01,       //   1 byte (Reserved)
        REPORT_SIZE(1),     0x08,
        INPUT(1),           0x01,       //   Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position

        REPORT_COUNT(1),    0x05,       //   5 bits (Num lock, Caps lock, Scroll lock, Compose, Kana)
        REPORT_SIZE(1),     0x01,
        USAGE_PAGE(1),      0x08,       //   LEDs
        USAGE_MINIMUM(1),   0x01,       //   Num Lock
        USAGE_MAXIMUM(1),   0x05,       //   Kana
        OUTPUT(1),          0x02,       //   Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile

        REPORT_COUNT(1),    0x01,       //   3 bits (Padding)
        REPORT_SIZE(1),     0x03,
        OUTPUT(1),          0x01,       //   Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile

        REPORT_COUNT(1),    0x06,       //   6 bytes (Keys)
        REPORT_SIZE(1),     0x08,
        LOGICAL_MINIMUM(1), 0x00,
        LOGICAL_MAXIMUM(1), 0xDD,       //   Generic keys
        USAGE_PAGE(1),      0x07,       //   Kbrd/Keypad
        USAGE_MINIMUM(1),   0x00,
        USAGE_MAXIMUM(1),   0x65,
        INPUT(1),           0x00,       //   Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position
        
    END_COLLECTION(0)
};

static const report_reference_t consumerInputReportReferenceData = { 3, INPUT_REPORT };
static const GattAttribute consumerInputReportReferenceDescriptor(BLE_UUID_DESCRIPTOR_REPORT_REFERENCE, (uint8_t *)&consumerInputReportReferenceData, 2, 2, false);
static const GattAttribute * consumerInputReportDescriptors[] = {
    &consumerInputReportReferenceDescriptor,
};

union InputReportData {
    uint8_t raw[8];
    struct {
        uint8_t modifier;
        uint8_t padding;
        uint8_t keycode[6];
    } data;
};

union OutputReportData {
    uint8_t raw[1];
};

union FeatureReportData {
    uint8_t raw[1];
};

union ConsumerInputReportData {
    uint8_t raw[2];
    uint16_t key;
};

class InputReportBuffer {
    CircularBuffer<InputReportData, 8> buffer;
    bool isPending;
    bool shouldClearKey;
    InputReportData pendingReport;
public:
    InputReportBuffer(void):
        buffer(),
        isPending(false),
        shouldClearKey(false)
    {
        buffer.reset();
        
        for(int i = 0; i < 8; i++) {
            pendingReport.raw[i] = (uint8_t)0;
        }
    }
    
    void push(InputReportData report) {
        buffer.push(report);
    }
    
    bool pop(InputReportData &report) {
        if(shouldClearKey) {
            for(uint8_t i = 0; i < 8; i++) {
                report.raw[i] = 0;
            }
            return true;
        }
        
        if(isPending) {
            report = pendingReport;
            return true;
        }
        
        if(!empty()) {
            buffer.pop(pendingReport);
            report = pendingReport;
            isPending = true;
            return true;
        }
        
        return false;
    }
    
    bool empty(void) {
        return buffer.empty() && !isPending && !shouldClearKey;
    }
    
    bool full(void) {
        return buffer.full();
    }
    
    void reset(void) {
        buffer.reset();
        isPending = false;
        shouldClearKey = false;
        
        for(int i = 0; i < 8; i++) {
            pendingReport.raw[i] = (uint8_t)0;
        }
    }
    
    void flushPending(void) {
        isPending = false;
    }
    
    void flushClearKey(void) {
        shouldClearKey = false;
    }
};

class KeyboardService : public HIDServiceBase {
    /**
     * Boot Protocol
     * Share input/output report with Report Protocol for memmory saving
     */
    GattCharacteristic bootKeyboardInputReportCharacteristic;
    GattCharacteristic bootKeyboardOutputReportCharacteristic;
    GattCharacteristic consumerInputReportCharacteristic;

    InputReportBuffer *inputReportBuffer;
    InputReportData inputReportData;
    InputReportData inputReportDataTemp;
    OutputReportData outputReportData;
    FeatureReportData featureReportData;
    ConsumerInputReportData consumerInputReportData;
    
    // Member variables
    uint8_t consecutiveFailures;
    
public:
    KeyboardService(BLE& _ble) :
        HIDServiceBase(
            _ble,
            KEYBOARD_REPORT_MAP,
            sizeof(KEYBOARD_REPORT_MAP),
            inputReport         = inputReportData.raw,
            outputReport        = outputReportData.raw,
            featureReport       = featureReportData.raw,
            inputReportLength   = sizeof(inputReportData),
            outputReportLength  = sizeof(outputReportData),
            featureReportLength = sizeof(featureReportData),
            reportTickerDelay   = 24
        ),
        bootKeyboardInputReportCharacteristic(
            GattCharacteristic::UUID_BOOT_KEYBOARD_INPUT_REPORT_CHAR,
            (uint8_t *)inputReport, inputReportLength, inputReportLength,
              GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ
            | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
            | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE
        ),
        bootKeyboardOutputReportCharacteristic(
            GattCharacteristic::UUID_BOOT_KEYBOARD_OUTPUT_REPORT_CHAR,
            (uint8_t *)outputReport, outputReportLength, outputReportLength,
              GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ
            | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE
            | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE
        ),
        consumerInputReportCharacteristic(
            GattCharacteristic::UUID_REPORT_CHAR,
            (uint8_t *)&consumerInputReportData, sizeof(consumerInputReportData), sizeof(consumerInputReportData),
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ
            | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
            | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE,
            const_cast<GattAttribute**>(consumerInputReportDescriptors), 1
        ),
        consecutiveFailures(0)
    {
        for (int i = 0; i < 8; i++) {
            inputReportData.raw[i] = (uint8_t)0;
            inputReportDataTemp.raw[i] = (uint8_t)0;
        }
        
        outputReportData.raw[0] = (uint8_t)0;
        
        inputReportBuffer = new InputReportBuffer();
        inputReportBuffer->reset();
    }

    virtual void addExtraCharacteristics(GattCharacteristic** characteristics, uint8_t& charIndex) {
        pc.printf("addExtraCharacteristics %d\r\n", charIndex);
        characteristics[charIndex++] = &bootKeyboardInputReportCharacteristic;
        characteristics[charIndex++] = &bootKeyboardOutputReportCharacteristic;
        characteristics[charIndex++] = &consumerInputReportCharacteristic;
    }
    
    virtual void onConnection(const Gap::ConnectionCallbackParams_t *params)
    {
        HIDServiceBase::onConnection(params);

        /* Drain buffer, in case we've been disconnected while transmitting */
        if (!reportTickerIsActive && !inputReportBuffer->empty()) {
            startReportTicker();
        }
    }

    virtual void onDisconnection(const Gap::DisconnectionCallbackParams_t *params)
    {
        stopReportTicker();
        HIDServiceBase::onDisconnection(params);
    }

    virtual ble_error_t send(const report_t report) {
        ble_error_t ret = BLE_ERROR_NONE;
        if (protocolMode == REPORT_PROTOCOL) { // this is default
            ret = ble.gattServer().write(
                inputReportCharacteristic.getValueHandle(),
                report,
                inputReportLength
            );
        } else {
            ret = ble.gattServer().write(
                bootKeyboardInputReportCharacteristic.getValueHandle(),
                report,
                inputReportLength
            );
        }
        
        if (ret == BLE_STACK_BUSY)
            consecutiveFailures++;
        else
            consecutiveFailures = 0;

        if (consecutiveFailures > 20) {
            /*
             * We're not transmitting anything anymore. Might as well avoid overloading the
             * system in case it can magically fix itself. Ticker will start again on next _putc
             * call, or on next connection.
             */
            stopReportTicker();
            consecutiveFailures = 0;
        }

        return ret;
    }
    
    void clearInputReportData(InputReportData &report) {
        for(uint8_t i = 0; i < 8; i++) {
            report.raw[i] = 0;
        }
    }
    
    
    
    /**
     * Pop a report from the internal FIFO, and attempt to send it over BLE
     *
     * keyUp reports should theoretically be sent after every keyDown, but we optimize the
     * throughput by only sending one when strictly necessary:
     * - when we need to repeat the same key
     * - when there is no more key to report
     *
     * In case of error, put the key event back in the buffer, and retry on next tick.
     */
    virtual void sendCallback(void) {
        ble_error_t ret;
    }
};
