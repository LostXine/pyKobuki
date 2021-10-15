def lsb(p):
    res = 0
    for byte in p[::-1]:
        res = res << 8
        res += byte
    return res
    
def signed2lsb(v, n=2):
    v = int(v)
    res = list(v.to_bytes(length=n, byteorder='little', signed=True))
    # print(f'V: {v} to', res)
    return res

def lsb2signed(p):
    return int.from_bytes(bytes(p), 'little', signed=True)
    
def parse_voltage(b):
    return lsb(b) * 3.3 / 4095
            
            
charger_state = {
    0: "DISCHARGING",
    2: "DOCKING_CHARGED",
    6: "DOCKING_CHARGING",
 	18: "ADAPTER_CHARGED",
 	22: "ADAPTER_CHARGING",
}

ir_bit_state = {
    0x01: "NEAR_LEFT",
    0x02: "NEAR_CENTER",
    0x04: "NEAR_RIGHT",
    0x08: "FAR_CENTER",
    0x10: "FAR_LEFT",
    0x20: "FAR_RIGHT",
}