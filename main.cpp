/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
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

#include "mbed.h"
#include "ble/BLE.h"
#include "BLEKeyboardService.h"
#include "ScanParametersService.h"
#include "BatteryService.h"
#include "DeviceInformationService.h"

DigitalOut led1(LED1);

static const char MANUFACTURERERS_NAME[] = "biacco42.info";
static const char MODEL_NAME[] = "ErgoDox BT";
static const char SERIAL_NUMBER[] = "X00";
static const char HARDWARE_REVISION[] = "0.1";
static const char FIRMWARE_REVISION[] = "0.1";
static const char SOFTWARE_REVISION[] = "0.1";
static PnPID_t PNP_ID = {
    0x01,  // From Bluetooth SIG
    0xFFFE,
    0x0001,
    0x01,
};

static const uint8_t DEVICE_NAME[] = "ErgoDox BT";

static const bool ENABLE_BONDING = true;
static const bool REQUIRE_MITM = true;
static const uint8_t PASSKEY[6] = {'1','2','3','4','5','6'}; // must be 6-digits number

static const uint16_t uuid16_list[]        = {GattService::UUID_HUMAN_INTERFACE_DEVICE_SERVICE,
                                              GattService::UUID_DEVICE_INFORMATION_SERVICE,
                                              GattService::UUID_BATTERY_SERVICE,
                                              GattService::UUID_SCAN_PARAMETERS_SERVICE};
static volatile bool  triggerSensorPolling = false;

uint8_t hrmCounter = 100; // init HRM to 100bps

static KeyboardService* keyboardService;
static BatteryService* batteryService;
static DeviceInformationService* deviceInformationService;
static ScanParametersService* scanParametersService;

static BLEProtocol::Address_t peerAddress;

void periodicCallback(void)
{
    led1 = !led1; /* Do blinky on LED1 while we're waiting for BLE events */

    /* Note that the periodicCallback() executes in interrupt context, so it is safer to do
     * heavy-weight sensor polling from the main thread. */
    triggerSensorPolling = true;
}

static void onConnect(const Gap::ConnectionCallbackParams_t *params) {
    peerAddress.type = params->peerAddrType;
    memcpy(peerAddress.address, params->peerAddr, Gap::ADDR_LEN);

    BLEProtocol::Address_t peerAddresses[2];
    Gap::Whitelist_t whitelist;
    whitelist.size = 0;
    whitelist.capacity = 2;
    whitelist.addresses = peerAddresses;
    BLE::Instance(BLE::DEFAULT_INSTANCE).gap().getWhitelist(whitelist);
    // printf("getWhitelist %d\r\n", whitelist.size);
    for (int i = 0; i < whitelist.size; i++) {
        if (whitelist.addresses[i].type == params->peerAddrType &&
                memcmp(whitelist.addresses[i].address, params->peerAddr, Gap::ADDR_LEN) == 0) {

            BLE::Instance(BLE::DEFAULT_INSTANCE).gap().setAdvertisingPolicyMode(Gap::ADV_POLICY_FILTER_ALL_REQS);
            // printf("peer is found in whitelist\r\n");
            return;
        }
    }

    // printf("peer is not found in whitelist\r\n");
}

static void onDisconnect(const Gap::DisconnectionCallbackParams_t *params) {
    // printf("onDisconnect\r\n"); // TODO recconect
    // BLE::Instance(BLE::DEFAULT_INSTANCE).gap().startAdvertising();
}

static void onTimeout(const Gap::TimeoutSource_t source) {
    // printf("onTimeout %d\r\n", source);
    switch (source) {
        case Gap::TIMEOUT_SRC_ADVERTISING:
            // printf("Advertising timeout");
            return;
        
        // treat following timeout as DISCONNECT (retry advertising)
        case Gap::TIMEOUT_SRC_SECURITY_REQUEST:
        case Gap::TIMEOUT_SRC_SCAN:
        case Gap::TIMEOUT_SRC_CONN:
            // printf("Disconnected by timeout");
            return;
    }
}

static void passkeyDisplayCallback(Gap::Handle_t handle, const SecurityManager::Passkey_t passkey) {
    // printf("Input passKey: ");
    for (unsigned i = 0; i < Gap::ADDR_LEN; i++) {
        // printf("%c", passkey[i]);
    }
    // printf("\r\n");
}

static void securitySetupCompletedCallback(Gap::Handle_t handle, SecurityManager::SecurityCompletionStatus_t status) {
    if (status == SecurityManager::SEC_STATUS_SUCCESS) {
        // printf("Security success %d\r\n", status);
        // printf("Set whitelist\r\n");
        Gap::Whitelist_t whitelist;
        whitelist.size = 1;
        whitelist.capacity = 1;
        whitelist.addresses = &peerAddress;

        BLE::Instance(BLE::DEFAULT_INSTANCE).gap().setWhitelist(whitelist);
        // printf("Set Advertising Policy Mode\r\n");
        // BLE::Instance(BLE::DEFAULT_INSTANCE).gap().setAdvertisingPolicyMode(Gap::ADV_POLICY_FILTER_SCAN_REQS);
        // BLE::Instance(BLE::DEFAULT_INSTANCE).gap().setAdvertisingPolicyMode(Gap::ADV_POLICY_FILTER_CONN_REQS);
        BLE::Instance(BLE::DEFAULT_INSTANCE).gap().setAdvertisingPolicyMode(Gap::ADV_POLICY_FILTER_ALL_REQS);
    } else {
        // printf("Security failed %d\r\n", status);
    }
}

static void securitySetupInitiatedCallback(Gap::Handle_t, bool allowBonding, bool requireMITM, SecurityManager::SecurityIOCapabilities_t iocaps) {
    // printf("Security setup initiated\r\n");
}

static void bleInitComplete(BLE::InitializationCompleteCallbackContext *params) {
    // https://developer.mbed.org/compiler/#nav:/keyboard/BLE_API/ble/blecommon.h;
    ble_error_t error;
    BLE &ble          = params->ble;

    /**< Minimum Connection Interval in 1.25 ms units, see BLE_GAP_CP_LIMITS.*/
    uint16_t minConnectionInterval = Gap::MSEC_TO_GAP_DURATION_UNITS(20);
    /**< Maximum Connection Interval in 1.25 ms units, see BLE_GAP_CP_LIMITS.*/
    uint16_t maxConnectionInterval = Gap::MSEC_TO_GAP_DURATION_UNITS(24);
    /**< Slave Latency in number of connection events, see BLE_GAP_CP_LIMITS.*/
    uint16_t slaveLatency = 50;
    /**< Connection Supervision Timeout in 10 ms units, see BLE_GAP_CP_LIMITS.*/ 
    uint16_t connectionSupervisionTimeout = 32 * 100;
    Gap::ConnectionParams_t connectionParams = {
        minConnectionInterval,
        maxConnectionInterval,
        slaveLatency,
        connectionSupervisionTimeout
    };

    error = params->error;
    if (error != BLE_ERROR_NONE) {
        // printf("error on ble.init() \r\n");
        goto return_error;
    }

    ble.gap().onDisconnection(onDisconnect);
    ble.gap().onConnection(onConnect);
    ble.gap().onTimeout(onTimeout);

    ble.securityManager().onSecuritySetupInitiated(securitySetupInitiatedCallback);
    ble.securityManager().onPasskeyDisplay(passkeyDisplayCallback);
    ble.securityManager().onSecuritySetupCompleted(securitySetupCompletedCallback);

    // bonding with hard-coded passkey.
    error = ble.securityManager().init(ENABLE_BONDING, REQUIRE_MITM, SecurityManager::IO_CAPS_DISPLAY_ONLY, PASSKEY);
    if (error != BLE_ERROR_NONE) {
        // printf("error on ble.securityManager().init()");
        goto return_error;
    }
    
    // Service instantiate
    pc.printf("BLE before instantiate services\r\n");
    keyboardService = new KeyboardService(ble); 
    deviceInformationService = new DeviceInformationService(ble, MANUFACTURERERS_NAME, MODEL_NAME, SERIAL_NUMBER, HARDWARE_REVISION, FIRMWARE_REVISION, SOFTWARE_REVISION, &PNP_ID);
    batteryService = new BatteryService(ble, 100, 5000);
    scanParametersService = new ScanParametersService(ble);
    pc.printf("BLE after instantiate services\r\n");

    ble.gap().setPreferredConnectionParams(&connectionParams);

    error = ble.gap().accumulateAdvertisingPayload( GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE );
    if (error != BLE_ERROR_NONE) goto return_error;

    error = ble.gap().accumulateAdvertisingPayload(
        GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS,
        (uint8_t*)uuid16_list, sizeof(uuid16_list)
    );
    if (error != BLE_ERROR_NONE) goto return_error;

    // see 5.1.2: HID over GATT Specification (pg. 25)
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000);
    ble.gap().setAdvertisingTimeout(300);

    // Set keyboard
    error = ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::KEYBOARD);
    if (error != BLE_ERROR_NONE) goto return_error;

    // Set complete local name
    error = ble.gap().accumulateScanResponse(
        GapAdvertisingData::COMPLETE_LOCAL_NAME,
        DEVICE_NAME, sizeof(DEVICE_NAME)
    );
    if (error != BLE_ERROR_NONE) goto return_error;

    // Set device name
    error = ble.gap().setDeviceName(DEVICE_NAME);
    if (error != BLE_ERROR_NONE) goto return_error;
    /* (Valid values are -40, -20, -16, -12, -8, -4, 0, 4) */
    ble.gap().setTxPower(0);

    {
        BLEProtocol::Address_t peerAddresses[2];
        Gap::Whitelist_t whitelist;
        whitelist.size = 0;
        whitelist.capacity = 2;
        whitelist.addresses = peerAddresses;
        error = ble.securityManager().getAddressesFromBondTable(whitelist);
        // printf("getAddressesFromBondTable %d\r\n", whitelist.size);
        ble.gap().setWhitelist(whitelist);
    }

    ble.gap().setAdvertisingPolicyMode(Gap::ADV_POLICY_IGNORE_WHITELIST);
    ble.gap().setScanningPolicyMode(Gap::SCAN_POLICY_FILTER_ALL_ADV);
    ble.gap().setInitiatorPolicyMode(Gap::INIT_POLICY_FILTER_ALL_ADV);

    error = ble.gap().startAdvertising();
    if (error != BLE_ERROR_NONE) goto return_error;
    return;

return_error:
    pc.printf("error with %d\r\n", error);
    return;
}

int main(void)
{
    pc.baud(9600);
    pc.printf("BLE keyboard start\r\n");
    led1 = 1;
    Ticker ticker;
    ticker.attach(periodicCallback, 1); // blink LED every second

    pc.printf("BLE initialize start\r\n");
    BLE& ble = BLE::Instance(BLE::DEFAULT_INSTANCE);
    ble.init(bleInitComplete);

    /* SpinWait for initialization to complete. This is necessary because the
     * BLE object is used in the main loop below. */
    while (ble.hasInitialized()  == false) { /* spin loop */ }
    pc.printf("BLE initialize comp\r\n");

    // infinite loop
    while (1) {
        // check for trigger from periodicCallback()
        if (triggerSensorPolling && ble.getGapState().connected) {
            triggerSensorPolling = false;

            // Do blocking calls or whatever is necessary for sensor polling.
            // In our case, we simply update the HRM measurement.

        } else {
            ble.waitForEvent(); // low power wait for event
        }
    }
}
