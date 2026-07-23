'''decode the AM32 debug1 structure carried in DroneCAN FlexDebug'''

import struct

FLEXDEBUG_AM32_DEBUG1 = 100

V1_FMT = '<BIHHHHiB'
V1_FIELDS = ('ver', 'ci', 'ncmd', 'ninp', 'rx_errors', 'rxframe_error',
             'rx_ecode', 'adv')
V2_FMT = V1_FMT + 'HHHHHB'
V2_FIELDS = V1_FIELDS + ('duty', 'duty_max', 'adj_input',
                         'adc_current', 'adc_volts', 'flags')


def decode_debug1(data):
    '''return a dict of fields, or None if not a known debug1 layout'''
    if len(data) == struct.calcsize(V1_FMT) and data[0] == 1:
        fmt, fields = V1_FMT, V1_FIELDS
    elif len(data) == struct.calcsize(V2_FMT) and data[0] == 2:
        fmt, fields = V2_FMT, V2_FIELDS
    else:
        return None
    row = dict(zip(fields, struct.unpack(fmt, bytes(data))))
    del row['ver']
    if 'flags' in row:
        row['armed'] = row['flags'] & 1
        row['running'] = (row['flags'] >> 1) & 1
        row['sine'] = (row['flags'] >> 2) & 1
        del row['flags']
    return row
