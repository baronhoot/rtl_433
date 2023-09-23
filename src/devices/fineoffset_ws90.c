/** @file
    Fine Offset Electronics WS90 weather station.

    Copyright (C) 2022 Christian W. Zuckschwerdt <zany@triq.net>
    Protocol description by @davidefa

    Copy of fineoffset_ws80.c with changes made to support Fine Offset WS90
    sensor array.  Changes made by John Pochmara <john@zoiedog.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "decoder.h"

/**
Fine Offset Electronics WS90 weather station.

The WS90 is a WS80 with the addition of a piezoelectric rain gauge.
Data bytes 1-13 are the same between the two models.  The new rain data
is in bytes 16-20, with bytes 19 and 20 reporting total rain.  Bytes
17 and 18 are affected by rain, but it is unknown what they report.  Byte
21 reports the voltage of the super cap. And the checksum and CRC
have been moved to bytes 30 and 31.  What is reported in the other
bytes is unknown at this time.

Also sold by EcoWitt.

Preamble is aaaa aaaa aaaa, sync word is 2dd4.

Packet layout:

     0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31
    YY II II II LL LL BB FF TT HH WW DD GG VV UU UU R0 R1 R2 R3 R4 SS UU UU UU UU UU UU UU ZZ AA XX
    90 00 34 2b 00 77 a4 82 62 39 00 3e 00 00 3f ff 20 00 ba 00 00 26 02 00 ff 9f f8 00 00 82 92 4f

- Y = fixed sensor type 0x90
- I = device ID, might be less than 24 bit?
- L = light value, unit of 10 lux
- B = battery voltage, unit of 20 mV, we assume a range of 3.0V to 1.4V
- F = flags and MSBs, 0x03: temp MSB, 0x10: wind MSB, 0x20: bearing MSB, 0x40: gust MSB
      0x80 or 0x08: maybe battery good? seems to be always 0x88
- T = temperature, lowest 8 bits of temperature, offset 40, scale 10
- H = humidity
- W = wind speed, lowest 8 bits of wind speed, m/s, scale 10
- D = wind bearing, lowest 8 bits of wind bearing, range 0-359 deg, 0x1ff if invalid
- G = wind gust, lowest 8 bits of wind gust, m/s, scale 10
- V = uv index, scale 10
- U = unknown (bytes 14 and 15 appear to be fixed at 3f ff)
- R = rain total (R3 << 8 | R4) * 0.1 mm
- S = super cap voltage, unit of 0.1V, lower 6 bits, mask 0x3f
- Z = Firmware version. 0x82 = 130 = 1.3.0
- A = checksum
- X = CRC

*/

static int fineoffset_ws90_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t const preamble[] = {0xaa, 0xaa, 0x2d, 0xd4}; // 32 bit, part of preamble and sync word
    uint8_t buf[32];

    // Validate package, WS90 nominal size is 330 bit periods
    if (bitbuffer->bits_per_row[0] < 168 || bitbuffer->bits_per_row[0] > 330) {
        decoder_logf_bitbuffer(decoder, 2, __func__, bitbuffer, "abort length" );
        return DECODE_ABORT_LENGTH;
    }

    // Find a data package and extract data buffer
    unsigned bit_offset = bitbuffer_search(bitbuffer, 0, 0, preamble, 32) + 32;
    if (bit_offset + sizeof(buf) * 8 > bitbuffer->bits_per_row[0]) { // Did not find a big enough package
        decoder_logf_bitbuffer(decoder, 2, __func__, bitbuffer, "short package at %u (%u)", bit_offset, bitbuffer->bits_per_row[0]);
        return DECODE_ABORT_LENGTH;
    }

    // Extract package data
    bitbuffer_extract_bytes(bitbuffer, 0, bit_offset, buf, sizeof(buf) * 8);

    if (buf[0] != 0x90) // Check for family code 0x90
        return DECODE_ABORT_EARLY;

    decoder_logf(decoder, 1, __func__, "WS90 detected, buffer is %u bits length", bitbuffer->bits_per_row[0]);

    // Verify checksum and CRC
    uint8_t crc = crc8(buf, 31, 0x31, 0x00);
    uint8_t chk = add_bytes(buf, 31);
    if (crc != 0 || chk != buf[31]) {
        decoder_logf(decoder, 1, __func__, "Checksum error: %02x %02x (%02x)", crc, chk, buf[31]);
        return DECODE_FAIL_MIC;
    }

    int id          = (buf[1] << 16) | (buf[2] << 8) | (buf[3]);
    int light_raw   = (buf[4] << 8) | (buf[5]);
    float light_lux = light_raw * 10;        // Lux
    int battery_mv  = (buf[6] * 20);         // mV
    int battery_lvl = battery_mv < 1680 ? 0 : (battery_mv - 1680) / 16; // 1.68V-3.0V is 0-100
    int flags       = buf[7]; // to find the wind msb
    int temp_raw    = ((buf[7] & 0x03) << 8) | (buf[8]);
    float temp_c    = (temp_raw - 400) * 0.1f;
    int humidity    = (buf[9]);
    int wind_avg    = ((buf[7] & 0x10) << 4) | (buf[10]);
    int wind_dir    = ((buf[7] & 0x20) << 3) | (buf[11]);
    int wind_max    = ((buf[7] & 0x40) << 2) | (buf[12]);
    int uv_index    = (buf[13]);
    int rain_raw    = (buf[19] << 8) | buf[20];
    int supercap_V  = (buf[21] & 0x3f);
    int firmware    = buf[29];
    char extra[31];

    if (battery_lvl > 100)
        battery_lvl = 100;

    char id_str[7];
    sprintf(id_str, "%06x", id);

    sprintf(extra, "%02x%02x%02x%02x%02x------%02x%02x%02x%02x%02x%02x%02x", buf[14], buf[15], buf[16], buf[17], buf[18], /* buf[19,20] is the rain sensor, buf[21] is supercap_V */ buf[22], buf[23], buf[24], buf[25], buf[26], buf[27], buf[28]);

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",                 DATA_STRING, "Fineoffset-WS90",
            "id",               "ID",               DATA_STRING, id_str,
            "battery_ok",       "Battery",          DATA_DOUBLE, battery_lvl * 0.01f,
            "battery_mV",       "Battery Voltage",  DATA_FORMAT, "%d mV", DATA_INT,    battery_mv,
            "temperature_C",    "Temperature",      DATA_COND, temp_raw != 0x3ff,   DATA_FORMAT, "%.1f C",   DATA_DOUBLE, temp_c,
            "humidity",         "Humidity",         DATA_COND, humidity != 0xff,    DATA_FORMAT, "%u %%",    DATA_INT, humidity,
            "wind_dir_deg",     "Wind direction",   DATA_COND, wind_dir != 0x1ff,   DATA_INT, wind_dir,
            "wind_avg_m_s",     "Wind speed",       DATA_COND, wind_avg != 0x1ff,   DATA_FORMAT, "%.1f m/s", DATA_DOUBLE, wind_avg * 0.1f,
            "wind_max_m_s",     "Gust speed",       DATA_COND, wind_max != 0x1ff,   DATA_FORMAT, "%.1f m/s", DATA_DOUBLE, wind_max * 0.1f,
            "uv",               "UVI",              DATA_COND, uv_index != 0xff,    DATA_FORMAT, "%.1f",     DATA_DOUBLE, uv_index * 0.1f,
            "lux",              "Light",            DATA_COND, light_raw != 0xffff, DATA_FORMAT, "%.1f lux", DATA_DOUBLE, (double)light_lux,
            "flags",            "Flags",            DATA_FORMAT, "%02x", DATA_INT, flags,
            "rain_mm",          "Total rainfall",   DATA_FORMAT, "%.1f mm", DATA_DOUBLE, rain_raw * 0.1f,
            "supercap_V",       "Supercap Voltage", DATA_COND, supercap_V != 0xff, DATA_FORMAT, "%.1f V", DATA_DOUBLE, supercap_V * 0.1f,
            "firmware",         "Firmware Version", DATA_INT, firmware,
            "data",             "Extra Data",       DATA_STRING, extra,
            "mic",              "Integrity",        DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);

    return 1;
}

static char *output_fields[] = {
        "model",
        "id",
        "battery_ok",
        "battery_mV",
        "temperature_C",
        "humidity",
        "wind_dir_deg",
        "wind_avg_m_s",
        "wind_max_m_s",
        "uv",
        "lux",
        "flags",
        "rain_mm",
        "supercap_V",
        "firmware",
        "data",
        "mic",
        NULL,
};

r_device fineoffset_ws90 = {
        .name        = "Fine Offset Electronics WS90 weather station",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 58,
        .long_width  = 58,
        .reset_limit = 3000,
        .decode_fn   = &fineoffset_ws90_decode,
        .fields      = output_fields,
};
