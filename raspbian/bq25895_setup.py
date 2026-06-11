# this file will be run on startup to setup the bq25895
reg_values = {
    0: 0x3f, # HiZ Off, Ilim OFF, IINLIM: 3.25A
    # skip 1 - no changes needed
    2: 0x30, # no adc conversion, ADC one-shot, 1.5MHz boost frequency, enable ICO, disable power source detection
             # this means will charge rapidly from "dumb" chargers, but ICO will prevent too much voltage droop
    # skip 3 - no changes needed
    4: 0x40, # set fast charge current limit to 4A (theoretical max is 12A so this is fine...)
    5: 0xA8, # precharge current 640mA, termination current 512mA
    # skip 6 - leave values as needed
    7: 0x8d, # charge termination enabled, STAT enabled, watchdog disabled, charge timer set to 12h
    # skip 8 = leave as is
    # skip 9
    # skip A  - leave boost voltage at 5.126
    # skip B  - read only
    # skip C - read only
    # skip D - leave as is
    # 0E-13 - read only
    # skip 1f - register reset only, otherwise read only
}