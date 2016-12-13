/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
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

#ifndef __BLE_BATTERY_SERVICE_H__
#define __BLE_BATTERY_SERVICE_H__

#include "ble/BLE.h"

static const UUID UUID_BATTERY_VOLTAGE = UUID("416B6DFF-D80C-477E-9841-30CB59C65C93");


/**
* @class BatteryService
* @brief BLE Battery Service. This service displays the battery level from 0% to 100%, represented as an 8bit number.
* Service:  https://developer.bluetooth.org/gatt/services/Pages/ServiceViewer.aspx?u=org.bluetooth.service.battery_service.xml
* Battery Level Char:  https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.battery_level.xml
*/
class BatteryService {
public:

	/**
	 * @param[in] _ble
	 *               BLE object for the underlying controller.
	 * @param[in] level
	 *               8bit batterly level. Usually used to represent percentage of batterly charge remaining.
	 */
	BatteryService(BLE &_ble, uint8_t level = 100, uint16_t voltage = 0) :
		ble(_ble),
		batteryLevel(level),
		batteryLevelCharacteristic(GattCharacteristic::UUID_BATTERY_LEVEL_CHAR, &batteryLevel, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY),
		batteryVoltage(voltage),
		batteryVoltageCharacteristic(UUID_BATTERY_VOLTAGE, &batteryVoltage, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY)
		 {

		// required for OS X bonding
		SecurityManager::SecurityMode_t securityMode = SecurityManager::SECURITY_MODE_ENCRYPTION_NO_MITM;
		batteryLevelCharacteristic.requireSecurity(securityMode);

		GattCharacteristic *charTable[] = {&batteryLevelCharacteristic, &batteryVoltageCharacteristic};
		GattService         batteryService(GattService::UUID_BATTERY_SERVICE, charTable, sizeof(charTable) / sizeof(GattCharacteristic *));

		ble.addService(batteryService);
	}

	/**
	 * @brief Update the battery level with a new value. Valid values lie between 0 and 100,
	 * anything outside this range will be ignored.
	 *
	 * @param newLevel
	 *              Update to battery level.
	 */
	void updateBatteryLevel(const uint8_t newLevel, const uint16_t newVoltage) {
		if (batteryLevel != newLevel) {
			batteryLevel = newLevel;
			ble.gattServer().write(batteryLevelCharacteristic.getValueHandle(), &batteryLevel, 1);
		}
		if (batteryVoltage != newVoltage) {
			batteryVoltage = newVoltage;
			ble.gattServer().write(batteryVoltageCharacteristic.getValueHandle(), reinterpret_cast<uint8_t*>(&batteryVoltage), 2);
		}
	}

protected:
	/**
	 * A reference to the underlying BLE instance that this object is attached to.
	 * The services and characteristics will be registered in this BLE instance.
	 */
	BLE &ble;

	/**
	 * The current battery level represented as an integer from 0% to 100%.
	 */
	uint8_t    batteryLevel;
	/**
	 * A ReadOnlyGattCharacteristic that allows access to the peer device to the
	 * batteryLevel value through BLE.
	 */
	ReadOnlyGattCharacteristic<uint8_t> batteryLevelCharacteristic;
	
	uint16_t batteryVoltage; // voltage in mV
	ReadOnlyGattCharacteristic<uint16_t> batteryVoltageCharacteristic;
};

#endif /* #ifndef __BLE_BATTERY_SERVICE_H__*/
