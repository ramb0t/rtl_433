/** @file
    EPH Controls RFRP-OT room thermostat and RF1A-OT boiler receiver.

    Copyright (C) 2026 Ramb0t

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "decoder.h"

/**
EPH Controls RFRP-OT room thermostat and RF1A-OT boiler receiver.

The thermostat and its boiler receiver talk on ~867.99 MHz, 2-FSK, 10 kbit/s NRZ, deviation +-20 kHz.
The thermostat sends a heartbeat every 2 minutes and on every change; the receiver acks each one 20-30 ms later.
The thermostat sometimes repeats a frame byte for byte 1.0-1.2 s later, when it missed the ack.

Tune ~40 kHz off the carrier (e.g. `-f 867.95M`) and use 250 kS/s: at 1 MS/s the bits slip.
The frames are weak at moderate gain; `-Y minlevel=-35` may be needed.

A frame is preamble `aa aa aa aa`, sync `2d d4`, then:

Data layout:

    LL XX FF RR DD SS SS SS SS TT TT TT TT [records]

- L: 8 bit length, bytes that follow
- X: 8 bit checksum, XOR of bytes 2 to L
- F: 8 bit flags, 0x00, or 0x20 on a receiver frame with reads still outstanding
- R: 8 bit record length, L - 12
- D: 8 bit direction, 0x00 thermostat to receiver, 0x01 receiver to thermostat
- S: 32 bit source address
- T: 32 bit destination address

Each record is `id cls op len value`, value little-endian, `len` bytes:

- op 0x0e report or write, 0x0f ack (len 0), 0x0c read (len 0), 0x0d read answer
- 00/10 room temperature, 01/10 effective setpoint, 10/10 manual setpoint: signed 16 bit, scale 10
- 0f/10 mode: 0x00 auto, 0x01 manual, 0x04 off
- 0a/10 heat demand: 0x01 off, 0x02 on
- 01/20 flow temperature request (OpenTherm id 1), signed 16 bit, scale 10
- 13/20 status: 0x00 off, 0x02 idle, 0x13 heating
- 01/00 firmware version, 8 bytes ASCII

The default (min/max) FSK pulse detector drops the last pulse and gap of a package, i.e. the final run of 1 bits
and the 0 bits after it; a value ending in zero bytes can lose over 30 bits. A frame short by up to 40 bits whose
last received bit is 0 has its tail rebuilt as 1s then 0s, and is accepted only if exactly one rebuild passes the
checksum and the record structure.
*/

#define EPH_MAX_LEN  62 // largest L seen, 63 bytes with the length byte
#define EPH_MAX_TAIL 40 // missing tail bits that may be rebuilt

/// Returns 0 for a valid frame, else DECODE_FAIL_SANITY or DECODE_FAIL_MIC.
static int eph_rfrpot_check(uint8_t const *b)
{
    unsigned len = b[0];
    if (len < 12 || len > EPH_MAX_LEN)
        return DECODE_FAIL_SANITY;
    if (b[3] != len - 12 || b[4] > 1 || (b[2] != 0x00 && !(b[2] == 0x20 && b[4] == 1)))
        return DECODE_FAIL_SANITY;
    // records must tile the frame exactly; reads and acks carry no value
    unsigned i = 13;
    while (i + 4 <= len + 1) {
        uint8_t op = b[i + 2];
        if (op < 0x0c || op > 0x0f || ((op == 0x0c || op == 0x0f) && b[i + 3] != 0))
            return DECODE_FAIL_SANITY;
        i += 4 + b[i + 3];
    }
    if (i != len + 1)
        return DECODE_FAIL_SANITY;
    if (xor_bytes(&b[2], len - 1) != b[1])
        return DECODE_FAIL_MIC;
    return 0;
}

static void eph_set_tail(uint8_t *b, unsigned have, unsigned total, unsigned ones)
{
    for (unsigned k = have; k < total; ++k) {
        uint8_t mask = 0x80 >> (k & 7);
        if (k < have + ones)
            b[k >> 3] |= mask;
        else
            b[k >> 3] &= ~mask;
    }
}

static int eph_rfrpot_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t const sync[] = {0xaa, 0x2d, 0xd4};
    int events           = 0;
    int ret              = DECODE_ABORT_EARLY;

    for (unsigned row = 0; row < bitbuffer->num_rows; ++row) {
        unsigned bits = bitbuffer->bits_per_row[row];
        unsigned pos  = bitbuffer_search(bitbuffer, row, 0, sync, 24);
        if (pos + 24 + 8 > bits) {
            continue;
        }
        pos += 24;

        uint8_t b[EPH_MAX_LEN + 1] = {0};
        unsigned have              = MIN(bits - pos, sizeof(b) * 8);
        bitbuffer_extract_bytes(bitbuffer, row, pos, b, have);
        if (b[0] < 12 || b[0] > EPH_MAX_LEN) {
            ret = DECODE_ABORT_LENGTH;
            continue;
        }
        unsigned total = (b[0] + 1) * 8;

        if (have < total) {
            // rebuild the dropped final pulse and gap
            unsigned missing = total - have;
            if (missing > EPH_MAX_TAIL) {
                ret = DECODE_ABORT_LENGTH;
                continue;
            }
            // the dropped pulse follows a gap, so the last bit received is 0
            if (b[(have - 1) >> 3] & (0x80 >> ((have - 1) & 7))) {
                ret = DECODE_FAIL_SANITY;
                continue;
            }
            int found     = 0;
            unsigned ones = 0;
            for (unsigned k = 1; k <= missing; ++k) {
                eph_set_tail(b, have, total, k);
                if (eph_rfrpot_check(b) == 0) {
                    found += 1;
                    ones = k;
                }
            }
            if (found != 1) {
                decoder_logf(decoder, 1, __func__, "%u tail bits missing, %d rebuilds pass", missing, found);
                ret = DECODE_FAIL_MIC;
                continue;
            }
            eph_set_tail(b, have, total, ones);
            decoder_logf(decoder, 2, __func__, "rebuilt %u tail bits", missing);
        }
        else {
            int err = eph_rfrpot_check(b);
            if (err) {
                decoder_log_bitrow(decoder, 1, __func__, b, total, err == DECODE_FAIL_MIC ? "checksum fail" : "structure fail");
                ret = err;
                continue;
            }
        }

        unsigned len = b[0];
        char src[9], dst[9];
        snprintf(src, sizeof(src), "%02x%02x%02x%02x", b[5], b[6], b[7], b[8]);
        snprintf(dst, sizeof(dst), "%02x%02x%02x%02x", b[9], b[10], b[11], b[12]);

        int has_temp = 0, has_setp = 0, has_man = 0, has_mode = 0, has_demand = 0, has_flow = 0, has_status = 0;
        int temp = 0, setp = 0, man = 0, mode = 0, demand = 0, flow = 0, status = 0;
        char firmware[9]  = {0};
        unsigned n_ops[4] = {0}; // 0c 0d 0e 0f
        unsigned records  = 0;

        for (unsigned i = 13; i + 4 <= len + 1; i += 4 + b[i + 3]) {
            uint8_t id = b[i], cls = b[i + 1], op = b[i + 2], vlen = b[i + 3];
            uint8_t const *v = &b[i + 4];
            records += 1;
            n_ops[op - 0x0c] += 1;
            if (op != 0x0e && op != 0x0d)
                continue;
            int16_t t = vlen == 2 ? (int16_t)(v[0] | v[1] << 8) : 0;
            if (cls == 0x10 && id == 0x00 && vlen == 2) {
                has_temp = 1;
                temp     = t;
            }
            else if (cls == 0x10 && id == 0x01 && vlen == 2) {
                has_setp = 1;
                setp     = t;
            }
            else if (cls == 0x10 && id == 0x10 && vlen == 2) {
                has_man = 1;
                man     = t;
            }
            else if (cls == 0x10 && id == 0x0f && vlen == 1) {
                has_mode = 1;
                mode     = v[0];
            }
            else if (cls == 0x10 && id == 0x0a && vlen == 1) {
                has_demand = 1;
                demand     = v[0] == 0x02;
            }
            else if (cls == 0x20 && id == 0x01 && vlen == 2) {
                has_flow = 1;
                flow     = t;
            }
            else if (cls == 0x20 && id == 0x13 && vlen == 1) {
                has_status = 1;
                status     = v[0];
            }
            else if (cls == 0x00 && id == 0x01 && vlen == 8) {
                for (unsigned k = 0; k < 8; ++k)
                    firmware[k] = v[k] >= 0x20 && v[k] < 0x7f ? v[k] : '.';
                for (int k = 7; k >= 0 && firmware[k] == ' '; --k)
                    firmware[k] = '\0';
            }
        }

        char const *msg_type = "mixed";
        if (records == 0)
            msg_type = "empty";
        else if (n_ops[3] == records)
            msg_type = "ack";
        else if (n_ops[2] == records)
            msg_type = "report";
        else if (n_ops[0] == records)
            msg_type = "read";
        else if (n_ops[1] == records)
            msg_type = "answer";

        char mode_str[8];
        snprintf(mode_str, sizeof(mode_str), "%s", mode == 0x00 ? "auto" : mode == 0x01 ? "manual"
                                                                   : mode == 0x04       ? "off"
                                                                                        : "");
        if (!mode_str[0])
            snprintf(mode_str, sizeof(mode_str), "0x%02x", mode);

        /* clang-format off */
        data_t *data = data_make(
                "model",            "",                 DATA_STRING, "EPH-RFRPOT",
                "id",               "",                 DATA_STRING, src,
                "dest_id",          "Destination",      DATA_STRING, dst,
                "source",           "Source",           DATA_STRING, b[4] ? "Receiver" : "Thermostat",
                "msg_type",         "Message",          DATA_STRING, msg_type,
                "flags",            "Flags",            DATA_COND,   b[2] != 0, DATA_INT, b[2],
                "temperature_C",    "Temperature",      DATA_COND,   has_temp, DATA_FORMAT, "%.1f C", DATA_DOUBLE, temp * 0.1,
                "setpoint_C",       "Setpoint",         DATA_COND,   has_setp, DATA_FORMAT, "%.1f C", DATA_DOUBLE, setp * 0.1,
                "setpoint_manual_C", "Manual setpoint", DATA_COND,   has_man, DATA_FORMAT, "%.1f C", DATA_DOUBLE, man * 0.1,
                "mode",             "Mode",             DATA_COND,   has_mode, DATA_STRING, mode_str,
                "heating",          "Heat demand",      DATA_COND,   has_demand, DATA_INT, demand,
                "flow_setpoint_C",  "Flow request",     DATA_COND,   has_flow, DATA_FORMAT, "%.1f C", DATA_DOUBLE, flow * 0.1,
                "status",           "Status",           DATA_COND,   has_status, DATA_INT, status,
                "firmware",         "Firmware",         DATA_COND,   firmware[0] != 0, DATA_STRING, firmware,
                "records",          "Records",          DATA_INT,    records,
                "mic",              "Integrity",        DATA_STRING, "CHECKSUM",
                NULL);
        /* clang-format on */
        decoder_output_data(decoder, data);
        events += 1;
    }

    return events > 0 ? events : ret;
}

static char const *const output_fields[] = {
        "model",
        "id",
        "dest_id",
        "source",
        "msg_type",
        "flags",
        "temperature_C",
        "setpoint_C",
        "setpoint_manual_C",
        "mode",
        "heating",
        "flow_setpoint_C",
        "status",
        "firmware",
        "records",
        "mic",
        NULL,
};

r_device const eph_rfrpot = {
        .name        = "EPH Controls RFRP-OT thermostat and RF1A-OT boiler receiver",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 100,
        .long_width  = 100,
        .reset_limit = 10000, // frames hold long runs of 0 bits; the carrier drop ends the package
        .decode_fn   = &eph_rfrpot_decode,
        .fields      = output_fields,
};
