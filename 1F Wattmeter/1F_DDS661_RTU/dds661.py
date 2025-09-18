#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DDS661 Wattmeter RS485 Configuration Tool
Komunikace s DDS661 wattmetrem přes RS485/Modbus
"""

import serial
import struct
import time
import sys
from typing import Optional, Tuple

# ===== GLOBAL SETTINGS =====
DEBUG_MODE = False  # Set to True for debug output, False for normal operation

class DDS661Controller:
    def __init__(self, port: str = "COM11", baudrate: int = 9600, parity: str = "E"):
        """Initialize DDS661 controller"""
        self.port = port
        self.baudrate = baudrate
        self.parity = parity
        self.serial_conn: Optional[serial.Serial] = None
        
        # Parity mapping
        self.parity_map = {
            "E": serial.PARITY_EVEN,
            "O": serial.PARITY_ODD, 
            "N": serial.PARITY_NONE
        }
        
        # Baudrate mapping
        self.baudrate_map = {
            1200: 0,
            2400: 1,
            4800: 2,
            9600: 3,
            19200: 4,
            38400: 5,
            57600: 6,
            115200: 7
        }
        
        # Reverse mappings
        self.parity_reverse = {v: k for k, v in self.parity_map.items()}
        self.baudrate_reverse = {v: k for k, v in self.baudrate_map.items()}

    def connect(self) -> bool:
        """Connect to DDS661 via RS485"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity=self.parity_map.get(self.parity, serial.PARITY_EVEN),
                stopbits=1,
                timeout=1  # Reduced timeout for faster scanning
            )
            print(f"✓ Připojeno k {self.port} ({self.baudrate} bps, parity {self.parity})")
            return True
        except Exception as e:
            print(f"✗ Chyba připojení: {e}")
            return False

    def disconnect(self):
        """Disconnect from DDS661"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Odpojeno od DDS661")

    def calculate_crc16(self, data: bytes) -> int:
        """Calculate Modbus CRC16"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def read_register(self, address: int, register: int, count: int = 2, debug: bool = None, read_type: str = "read") -> Optional[float]:
        """Read register from DDS661"""
        # Use global DEBUG_MODE if debug not specified
        if debug is None:
            debug = DEBUG_MODE
            
        if not self.serial_conn or not self.serial_conn.is_open:
            if debug:
                print(f"DEBUG: Serial connection not open")
            return None
            
        try:
            # Choose Modbus function code
            if read_type == "read":
                # Modbus RTU read input registers (0x04) - for measurements
                function_code = 0x04
            else:
                # Modbus RTU read holding registers (0x03) - for settings
                function_code = 0x03
                
            cmd = struct.pack('>BBHH', address, function_code, register, count)
            crc = self.calculate_crc16(cmd)
            cmd += struct.pack('<H', crc)
            
            if debug:
                print(f"DEBUG: Sending command to address {address}, register 0x{register:04X}")
                print(f"DEBUG: Command bytes: {' '.join(f'{b:02X}' for b in cmd)}")
            
            self.serial_conn.write(cmd)
            time.sleep(0.1)  # Increased delay for debugging
            
            response = self.serial_conn.read(9)  # Expected response length
            
            if debug:
                print(f"DEBUG: Response length: {len(response)}")
                if len(response) > 0:
                    print(f"DEBUG: Response bytes: {' '.join(f'{b:02X}' for b in response)}")
            
            if len(response) < 5:
                if debug:
                    print(f"DEBUG: Response too short: {len(response)} bytes")
                return None
                
            # Verify CRC
            received_crc = struct.unpack('<H', response[-2:])[0]
            calculated_crc = self.calculate_crc16(response[:-2])
            
            if debug:
                print(f"DEBUG: CRC check - received: 0x{received_crc:04X}, calculated: 0x{calculated_crc:04X}")
            
            if received_crc != calculated_crc:
                if debug:
                    print(f"DEBUG: CRC mismatch")
                return None
                
            # Extract data (FP32 format)
            if len(response) >= 9:
                data_bytes = response[3:7]
                value = struct.unpack('>f', data_bytes)[0]
                if debug:
                    print(f"DEBUG: Extracted value: {value}")
                return value
                
        except Exception as e:
            if debug:
                print(f"DEBUG: Exception: {e}")
            pass
            
        return None

    def write_register(self, address: int, register: int, value: float) -> bool:
        """Write register to DDS661"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return False
            
        try:
            # Convert float to 2x16-bit words (big-endian)
            value_bytes = struct.pack('>f', value)
            word1 = struct.unpack('>H', value_bytes[:2])[0]
            word2 = struct.unpack('>H', value_bytes[2:])[0]
            
            # Modbus RTU write multiple registers (0x10)
            # Format: address, function_code, register, count, byte_count, word1, word2
            cmd = struct.pack('>BBHHBHH', address, 0x10, register, 2, 4, word1, word2)
            crc = self.calculate_crc16(cmd)
            cmd += struct.pack('<H', crc)
            
            if DEBUG_MODE:
                print(f"DEBUG: Writing to address {address}, register 0x{register:04X}, value {value}")
                print(f"DEBUG: Command bytes: {' '.join(f'{b:02X}' for b in cmd)}")
            
            self.serial_conn.write(cmd)
            time.sleep(0.1)
            
            response = self.serial_conn.read(8)  # Expected response length
            
            if DEBUG_MODE:
                print(f"DEBUG: Response length: {len(response)}")
                if len(response) > 0:
                    print(f"DEBUG: Response bytes: {' '.join(f'{b:02X}' for b in response)}")
            
            if len(response) >= 8:
                # Verify response
                if response[0] == address and response[1] == 0x10:
                    if DEBUG_MODE:
                        print("DEBUG: Write successful")
                    return True
                    
        except Exception as e:
            print(f"Chyba zápisu registru {register}: {e}")
            
        return False

    def scan_devices(self, start_addr: int = 1, end_addr: int = 10, parity: str = None, baudrate: int = None) -> dict:
        """Scan for DDS661 devices with specific settings"""
        if parity:
            print(f"🔍 Skenování adres {start_addr}-{end_addr} (parity: {parity}, rychlost: {baudrate} bps)...")
        else:
            print(f"🔍 Skenování adres {start_addr}-{end_addr}...")
        found_devices = {}
        
        for addr in range(start_addr, end_addr + 1):
            print(f"  {addr}...", end=" ")
            
            # Try to read voltage register (0x0000) - faster timeout
            voltage = self.read_register(addr, 0x0000, read_type="read")
            
            if voltage is not None and 200 <= voltage <= 260:
                print(f"✓ {voltage:.1f}V")
                
                # Read additional info only if device found
                current = self.read_register(addr, 0x0008, read_type="read")
                power = self.read_register(addr, 0x0012, read_type="read")
                frequency = self.read_register(addr, 0x0036, read_type="read")
                
                found_devices[addr] = {
                    'voltage': voltage,
                    'current': current,
                    'power': power,
                    'frequency': frequency,
                    'parity': parity,
                    'baudrate': baudrate
                }
                
                # Stop scanning after first device found
                print(f"✅ Zařízení nalezeno na adrese {addr}! Skenování ukončeno.")
                break
            else:
                print("✗")
                
        return found_devices

    def scan_fast(self, baudrate: int) -> dict:
        """Fast scan - first 10 addresses with all parity options"""
        print(f"🚀 RYCHLÉ SKENOVÁNÍ - adresy 1-10, rychlost {baudrate} bps")
        print("Testování všech parity (EVEN/NONE/ODD)...")
        
        all_devices = {}
        parities = ["E", "N", "O"]
        
        for parity in parities:
            print(f"\n📡 Testování parity {parity}...")
            
            # Temporarily change connection settings
            old_parity = self.parity
            old_baudrate = self.baudrate
            
            if self.parity != parity or self.baudrate != baudrate:
                self.disconnect()
                self.parity = parity
                self.baudrate = baudrate
                if not self.connect():
                    print(f"✗ Nelze se připojit s parity {parity}")
                    continue
            
            devices = self.scan_devices(1, 10, parity, baudrate)
            
            if devices:
                all_devices.update(devices)
                print(f"✅ Nalezeno {len(devices)} zařízení s parity {parity}")
                # Stop scanning after first device found
                break
            
            # Restore original settings
            if old_parity != parity or old_baudrate != baudrate:
                self.disconnect()
                self.parity = old_parity
                self.baudrate = old_baudrate
                self.connect()
        
        return all_devices

    def scan_full(self, baudrate: int) -> dict:
        """Full scan - addresses 1-254 with all parity options"""
        print(f"🔍 PLNÉ SKENOVÁNÍ - adresy 1-254, rychlost {baudrate} bps")
        print("Testování všech parity (EVEN/NONE/ODD)...")
        print("⚠️  Toto může trvat několik minut!")
        
        all_devices = {}
        parities = ["E", "N", "O"]
        
        for parity in parities:
            print(f"\n📡 Testování parity {parity}...")
            
            # Temporarily change connection settings
            old_parity = self.parity
            old_baudrate = self.baudrate
            
            if self.parity != parity or self.baudrate != baudrate:
                self.disconnect()
                self.parity = parity
                self.baudrate = baudrate
                if not self.connect():
                    print(f"✗ Nelze se připojit s parity {parity}")
                    continue
            
            devices = self.scan_devices(1, 254, parity, baudrate)
            
            if devices:
                all_devices.update(devices)
                print(f"✅ Nalezeno {len(devices)} zařízení s parity {parity}")
                # Stop scanning after first device found
                break
            
            # Restore original settings
            if old_parity != parity or old_baudrate != baudrate:
                self.disconnect()
                self.parity = old_parity
                self.baudrate = old_baudrate
                self.connect()
        
        return all_devices

    def scan_manual(self, address: int, parity: str, baudrate: int) -> dict:
        """Manual scan - specific address with specific settings"""
        print(f"🎯 MANUÁLNÍ SKENOVÁNÍ - adresa {address}, parity {parity}, rychlost {baudrate} bps")
        
        # Temporarily change connection settings
        old_parity = self.parity
        old_baudrate = self.baudrate
        
        if self.parity != parity or self.baudrate != baudrate:
            self.disconnect()
            self.parity = parity
            self.baudrate = baudrate
            if not self.connect():
                print(f"✗ Nelze se připojit s parity {parity}")
                return {}
        
        devices = self.scan_devices(address, address, parity, baudrate)
        
        # Restore original settings
        if old_parity != parity or old_baudrate != baudrate:
            self.disconnect()
            self.parity = old_parity
            self.baudrate = old_baudrate
            self.connect()
        
        return devices

    def get_current_voltage(self, address: int = 1) -> Optional[float]:
        """Get current voltage reading for status display"""
        return self.read_register(address, 0x0000, read_type="read")
    
    def get_device_status(self, address: int) -> dict:
        """Get complete device status for display"""
        voltage = self.read_register(address, 0x0000, read_type="read")
        current = self.read_register(address, 0x0008, read_type="read")
        power = self.read_register(address, 0x0012, read_type="read")
        frequency = self.read_register(address, 0x0036, read_type="read")
        
        return {
            'voltage': voltage,
            'current': current,
            'power': power,
            'frequency': frequency
        }

    def auto_detect_default(self) -> dict:
        """Auto-detect device with default settings (address 1, EVEN parity, 9600 bps)"""
        print("🔍 Automatické hledání zařízení s výchozím nastavením...")
        print("   Adresa: 1, Parity: EVEN, Rychlost: 9600 bps")
        
        if DEBUG_MODE:
            print("DEBUG: Testing voltage register 0x0000 with READ function (0x04)...")
        voltage = self.read_register(1, 0x0000, read_type="read")
        
        if voltage is not None and 200 <= voltage <= 260:
            if DEBUG_MODE:
                print(f"DEBUG: Found voltage {voltage:.1f}V in register 0x0000")
        else:
            if DEBUG_MODE:
                print(f"DEBUG: Voltage not found or invalid: {voltage}")
                print("DEBUG: Testing other measurement registers...")
                print("DEBUG: Current register 0x0008:")
                current = self.read_register(1, 0x0008, read_type="read")
                print("DEBUG: Power register 0x0012:")
                power = self.read_register(1, 0x0012, read_type="read")
        
        if voltage is not None and 200 <= voltage <= 260:
            print(f"✅ Zařízení nalezeno! Napětí: {voltage:.1f}V")
            
            # Read additional info
            current = self.read_register(1, 0x0008, read_type="read")
            power = self.read_register(1, 0x0012, read_type="read")
            frequency = self.read_register(1, 0x0036, read_type="read")
            
            return {
                1: {
                    'voltage': voltage,
                    'current': current,
                    'power': power,
                    'frequency': frequency,
                    'parity': 'E',
                    'baudrate': 9600
                }
            }
        else:
            print(f"❌ Žádné zařízení nenalezeno s výchozím nastavením (voltage: {voltage})")
            return {}

    def get_device_info(self, address: int) -> dict:
        """Get complete device information"""
        print(f"📊 Čtení informací z adresy {address}...")
        
        info = {}
        
        # Basic measurements
        info['voltage'] = self.read_register(address, 0x0000)
        info['current'] = self.read_register(address, 0x0008)
        info['power'] = self.read_register(address, 0x0012)
        info['frequency'] = self.read_register(address, 0x0036)
        
        # Settings (use holding register read)
        info['device_address'] = self.read_register(address, 0x0008, read_type="holding")
        info['baudrate'] = self.read_register(address, 0x0000, read_type="holding")
        info['parity'] = self.read_register(address, 0x0002, read_type="holding")
        
        return info

    def change_address(self, current_addr: int, new_addr: int) -> bool:
        """Change device address"""
        print(f"🔄 Změna adresy z {current_addr} na {new_addr}...")
        
        if not (1 <= new_addr <= 247):
            print("✗ Neplatná adresa (1-247)")
            return False
            
        success = self.write_register(current_addr, 0x0008, float(new_addr))
        
        if success:
            print(f"✓ Adresa změněna na {new_addr}")
            print("⚠️  Restartujte zařízení pro aktivaci změny!")
            print("🔄 Přepínám program na novou adresu...")
            return True
        else:
            print("✗ Chyba při změně adresy")
            return False

    def change_baudrate(self, address: int, new_baudrate: int) -> bool:
        """Change baudrate"""
        print(f"🔄 Změna rychlosti na {new_baudrate} bps...")
        
        if new_baudrate not in self.baudrate_map:
            print(f"✗ Nepodporovaná rychlost. Dostupné: {list(self.baudrate_map.keys())}")
            return False
            
        baudrate_code = self.baudrate_map[new_baudrate]
        success = self.write_register(address, 0x0000, float(baudrate_code))
        
        if success:
            print(f"✓ Rychlost změněna na {new_baudrate} bps")
            print("⚠️  Restartujte zařízení pro aktivaci změny!")
            print("🔄 Přepínám program na novou rychlost...")
            return True
        else:
            print("✗ Chyba při změně rychlosti")
            return False

    def change_parity(self, address: int, new_parity: str) -> bool:
        """Change parity"""
        print(f"🔄 Změna parity na {new_parity}...")
        
        parity_codes = {"E": 0.0, "O": 1.0, "N": 2.0}
        
        if new_parity not in parity_codes:
            print("✗ Neplatná parity (E/O/N)")
            return False
            
        parity_code = parity_codes[new_parity]
        success = self.write_register(address, 0x0002, parity_code)
        
        if success:
            print(f"✓ Parity změněna na {new_parity}")
            print("⚠️  Restartujte zařízení pro aktivaci změny!")
            print("🔄 Přepínám program na novou parity...")
            return True
        else:
            print("✗ Chyba při změně parity")
            return False

def print_menu():
    """Print main menu"""
    print("\n" + "="*50)
    print("🔧 DDS661 Wattmeter Configuration Tool")
    print("="*50)
    print("1. SCAN FAST - Rychlé skenování (adresy 1-10)")
    print("2. SCAN FULL - Plné skenování (adresy 1-254)")
    print("3. SCAN MANUAL - Manuální skenování (konkrétní adresa)")
    print("4. INFO - Informace o zařízení")
    print("5. CHANGE ADDRESS - Změna adresy")
    print("6. CHANGE BAUDRATE - Změna rychlosti")
    print("7. CHANGE PARITY - Změna parity")
    print("8. EXIT - Ukončení")
    print("="*50)

def print_baudrate_menu():
    """Print baudrate selection menu"""
    print("\n" + "="*30)
    print("📡 VÝBĚR RYCHLOSTI")
    print("="*30)
    print("1. 1200 bps")
    print("2. 2400 bps")
    print("3. 4800 bps")
    print("4. 9600 bps (výchozí)")
    print("5. 19200 bps")
    print("6. 38400 bps")
    print("7. 57600 bps")
    print("8. 115200 bps")
    print("="*30)

def print_parity_menu():
    """Print parity selection menu"""
    print("\n" + "="*20)
    print("📡 VÝBĚR PARITY")
    print("="*20)
    print("1. EVEN (E) - výchozí")
    print("2. NONE (N)")
    print("3. ODD (O)")
    print("="*20)

def select_baudrate() -> int:
    """Select baudrate from menu"""
    baudrates = [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200]
    
    while True:
        print_baudrate_menu()
        choice = input("Vyberte rychlost (1-8, výchozí 4): ").strip()
        
        if not choice:
            return 9600  # Default
        
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(baudrates):
                return baudrates[idx]
            else:
                print("❌ Neplatná volba")
        except ValueError:
            print("❌ Neplatná volba")

def select_parity() -> str:
    """Select parity from menu"""
    parities = ["E", "N", "O"]
    
    while True:
        print_parity_menu()
        choice = input("Vyberte parity (1-3, výchozí 1): ").strip()
        
        if not choice:
            return "E"  # Default
        
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(parities):
                return parities[idx]
            else:
                print("❌ Neplatná volba")
        except ValueError:
            print("❌ Neplatná volba")

def main():
    """Main program"""
    print("🚀 Spouštění DDS661 Configuration Tool...")
    if DEBUG_MODE:
        print("🐛 DEBUG MODE: ZAPNUTÝ")
    else:
        print("🐛 DEBUG MODE: VYPNUTÝ")
    
    # Initialize controller with default settings
    controller = DDS661Controller(port="COM11", baudrate=9600, parity="E")
    
    if not controller.connect():
        print("❌ Nelze se připojit k DDS661. Zkontrolujte:")
        print("   - COM port (aktuálně: COM11)")
        print("   - Připojení RS485")
        print("   - Napájení DDS661")
        return
    
    # Auto-detect device with default settings
    detected_devices = controller.auto_detect_default()
    current_device_address = 1 if detected_devices else None
    current_device_settings = None
    last_status_update = 0
    
    try:
        while True:
            # Update status every 5 seconds
            current_time = time.time()
            if current_device_address and (current_time - last_status_update >= 5):
                status = controller.get_device_status(current_device_address)
                if status['voltage'] is not None and 200 <= status['voltage'] <= 260:
                    print(f"\n⚡ DDS661 Status (adresa {current_device_address}):")
                    print(f"   Napětí: {status['voltage']:.1f}V")
                    if status['current'] is not None:
                        print(f"   Proud: {status['current']:.2f}A")
                    if status['power'] is not None:
                        print(f"   Výkon: {status['power']:.0f}W")
                    if status['frequency'] is not None:
                        print(f"   Frekvence: {status['frequency']:.1f}Hz")
                    if current_device_settings:
                        parity_name = {"E": "EVEN", "N": "NONE", "O": "ODD"}[current_device_settings['parity']]
                        print(f"   Nastavení: {current_device_settings['baudrate']} bps, parity {parity_name}")
                else:
                    print(f"\n⚡ DDS661 Status (adresa {current_device_address}): N/A")
                last_status_update = current_time
            
            print_menu()
            choice = input("Vyberte možnost (1-8): ").strip()
            
            if choice == "1":
                # SCAN FAST
                baudrate = select_baudrate()
                devices = controller.scan_fast(baudrate)
                
                if devices:
                    print(f"\n✅ Nalezeno {len(devices)} zařízení:")
                    for addr, info in devices.items():
                        parity_name = {"E": "EVEN", "N": "NONE", "O": "ODD"}[info['parity']]
                        print(f"   Adresa {addr}: {info['voltage']:.1f}V, {info['current']:.2f}A, {info['power']:.0f}W")
                        print(f"      Nastavení: {info['baudrate']} bps, parity {parity_name}")
                    # Update current device address and settings
                    current_device_address = list(devices.keys())[0]
                    current_device_settings = devices[current_device_address]
                    last_status_update = 0  # Force immediate status update
                else:
                    print("\n❌ Žádná zařízení nenalezena")
                    
            elif choice == "2":
                # SCAN FULL
                baudrate = select_baudrate()
                devices = controller.scan_full(baudrate)
                
                if devices:
                    print(f"\n✅ Nalezeno {len(devices)} zařízení:")
                    for addr, info in devices.items():
                        parity_name = {"E": "EVEN", "N": "NONE", "O": "ODD"}[info['parity']]
                        print(f"   Adresa {addr}: {info['voltage']:.1f}V, {info['current']:.2f}A, {info['power']:.0f}W")
                        print(f"      Nastavení: {info['baudrate']} bps, parity {parity_name}")
                    # Update current device address and settings
                    current_device_address = list(devices.keys())[0]
                    current_device_settings = devices[current_device_address]
                    last_status_update = 0  # Force immediate status update
                else:
                    print("\n❌ Žádná zařízení nenalezena")
                    
            elif choice == "3":
                # SCAN MANUAL
                addr_input = input("Adresa zařízení (1-254): ").strip()
                if addr_input:
                    try:
                        address = int(addr_input)
                        if 1 <= address <= 254:
                            parity = select_parity()
                            baudrate = select_baudrate()
                            devices = controller.scan_manual(address, parity, baudrate)
                            
                            if devices:
                                print(f"\n✅ Zařízení nalezeno na adrese {address}:")
                                info = devices[address]
                                parity_name = {"E": "EVEN", "N": "NONE", "O": "ODD"}[info['parity']]
                                print(f"   Napětí: {info['voltage']:.1f}V")
                                print(f"   Proud: {info['current']:.2f}A")
                                print(f"   Výkon: {info['power']:.0f}W")
                                print(f"   Frekvence: {info['frequency']:.1f}Hz")
                                print(f"   Nastavení: {info['baudrate']} bps, parity {parity_name}")
                                # Update current device address and settings
                                current_device_address = address
                                current_device_settings = info
                                last_status_update = 0  # Force immediate status update
                            else:
                                print(f"\n❌ Žádné zařízení nenalezeno na adrese {address}")
                        else:
                            print("❌ Neplatná adresa (1-254)")
                    except ValueError:
                        print("❌ Neplatná adresa")
                    
            elif choice == "4":
                # INFO
                addr = input("Adresa zařízení: ").strip()
                if addr:
                    address = int(addr)
                    info = controller.get_device_info(address)
                    
                    print(f"\n📊 Informace o zařízení na adrese {address}:")
                    print(f"   Napětí: {info['voltage']:.1f}V" if info['voltage'] else "   Napětí: N/A")
                    print(f"   Proud: {info['current']:.2f}A" if info['current'] else "   Proud: N/A")
                    print(f"   Výkon: {info['power']:.0f}W" if info['power'] else "   Výkon: N/A")
                    print(f"   Frekvence: {info['frequency']:.1f}Hz" if info['frequency'] else "   Frekvence: N/A")
                    print(f"   Adresa: {info['device_address']:.0f}" if info['device_address'] else "   Adresa: N/A")
                    print(f"   Rychlost: {controller.baudrate_reverse.get(int(info['baudrate']), 'N/A')} bps" if info['baudrate'] else "   Rychlost: N/A")
                    print(f"   Parity: {['E', 'O', 'N'][int(info['parity'])] if info['parity'] is not None else 'N/A'}")
                    
            elif choice == "5":
                # CHANGE ADDRESS
                current = input("Aktuální adresa: ").strip()
                new = input("Nová adresa (1-247): ").strip()
                
                if current and new:
                    success = controller.change_address(int(current), int(new))
                    if success:
                        # Update program settings to new address
                        current_device_address = int(new)
                        if current_device_settings:
                            current_device_settings['address'] = int(new)
                        last_status_update = 0  # Force immediate status update
                        print("✅ Program přepnut na novou adresu")
                    
            elif choice == "6":
                # CHANGE BAUDRATE
                addr = input("Adresa zařízení: ").strip()
                baudrate = select_baudrate()
                
                if addr:
                    success = controller.change_baudrate(int(addr), baudrate)
                    if success:
                        # Update program settings to new baudrate
                        controller.disconnect()
                        controller.baudrate = baudrate
                        if controller.connect():
                            if current_device_settings:
                                current_device_settings['baudrate'] = baudrate
                            last_status_update = 0  # Force immediate status update
                            print("✅ Program přepnut na novou rychlost")
                        else:
                            print("❌ Chyba při přepnutí na novou rychlost")
                    
            elif choice == "7":
                # CHANGE PARITY
                addr = input("Adresa zařízení: ").strip()
                parity = select_parity()
                
                if addr:
                    success = controller.change_parity(int(addr), parity)
                    if success:
                        # Update program settings to new parity
                        controller.disconnect()
                        controller.parity = parity
                        if controller.connect():
                            if current_device_settings:
                                current_device_settings['parity'] = parity
                            last_status_update = 0  # Force immediate status update
                            print("✅ Program přepnut na novou parity")
                        else:
                            print("❌ Chyba při přepnutí na novou parity")
                    
            elif choice == "8":
                # EXIT
                print("👋 Ukončování...")
                break
                
            else:
                print("❌ Neplatná volba")
                
    except KeyboardInterrupt:
        print("\n👋 Ukončování...")
    except Exception as e:
        print(f"❌ Chyba: {e}")
    finally:
        controller.disconnect()

if __name__ == "__main__":
    main()
