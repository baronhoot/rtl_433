/** @file
    Fine Offset Electronics WS90 weather station.

    Copyright (C) 2022 Christian W. Zuckschwerdt <zany@triq.net>
    Protocol description by @davidefa

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "decoder.h"

/**
Fine Offset Electronics WS90 weather station.

Also sold by EcoWitt, used with the weather station GW1000.

Preamble is aaaa aaaa aaaa, sync word is 2dd4.

Packet layout:

     0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17
    YY II II II LL LL BB FF TT HH WW DD GG VV UU UU AA XX
    80 0a 00 3b 00 00 88 8a 59 38 18 6d 1c 00 ff ff d8 df

- Y = fixed sensor type 0x90
- I = device ID, might be less than 24 bit?
- L = light value, unit of 10 Lux (or 0.078925 W/m2)
- B = battery voltage, unit of 20 mV, we assume a range of 3.0V to 1.4V
- F = flags and MSBs, 0x03: temp MSB, 0x10: wind MSB, 0x20: bearing MSB, 0x40: gust MSB
      0x80 or 0x08: maybe battery good? seems to be always 0x88
- T = temperature, lowest 8 bits of temperature, offset 40, scale 10
- H = humidity
- W = wind speed, lowest 8 bits of wind speed, m/s, scale 10
- D = wind bearing, lowest 8 bits of wind bearing, range 0-359 deg, 0x1ff if invalid
- G = wind gust, lowest 8 bits of wind gust, m/s, scale 10
- V = uv index, scale 10
- U = unknown, might be rain option
- A = checksum
- X = CRC

      0     1    2       4       6
   TYPE:8h ?8h ID:16h LUX:16d BATT:8d 

   7:0x80     0x40    0x20     0x10   0x0b  0x03
   B_OK:1b WG_HI:1b WD_HI:1b WS_HI:1b ?2b T_HI:2b

      8     9    10       11        12       13
   T_LO:8d H:8d WS_LO:8d WD_LO:8d WG_LO:8d UVI:8d

      14   15   16  17  18  19  20  21  22
    56h ?:8h ?:8h ?8h ?8h ?8h ?8h ?8h ?8h ?8h 
   CHK:8h CRC:8h ?8h4h

*/

static int fineoffset_ws90_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t const preamble[] = {0xaa, 0x2d, 0xd4}; // 24 bit, part of preamble and sync word
    uint8_t buf[32];

    // Validate package, WS90 nominal size is 327 bit periods
    if (bitbuffer->bits_per_row[0] < 280 || bitbuffer->bits_per_row[0] > 350) {
        return DECODE_ABORT_LENGTH;
    }

    decoder_logf(decoder, 1, __func__, "Found possible data for WS90 with len=%2d", bitbuffer->bits_per_row[0]);

    // Find a data package and extract data buffer
    unsigned bit_offset = bitbuffer_search(bitbuffer, 0, 0, preamble, 24) + 24;
    if (bit_offset + sizeof(buf) * 8 > bitbuffer->bits_per_row[0]) { // Did not find a big enough package
        decoder_logf_bitbuffer(decoder, 2, __func__, bitbuffer, "short package at %u", bit_offset);
        return DECODE_ABORT_LENGTH;
    }

    // Extract package data
    bitbuffer_extract_bytes(bitbuffer, 0, bit_offset, buf, sizeof(buf) * 8);

    if (buf[0] != 0x90) { // Check for family code 0x90
        decoder_logf(decoder, 0, __func__, "Aborting early with code: %02x", buf[0]);
        return DECODE_ABORT_EARLY;
    }
    
    // Verify checksum and CRC
    uint8_t crc = crc8(buf, 31, 0x31, 0x00);
    uint8_t chk = add_bytes(buf, 31);
    if (crc != 0 || chk != buf[31]) {
        decoder_logf_bitbuffer(decoder, 0, __func__, bitbuffer, "CRC error: %02x %02x", crc, chk);
        return DECODE_FAIL_MIC;
    }

    int id          = (buf[1] << 16) | (buf[2] << 8) | (buf[3]);

    char id_str[7];
    sprintf(id_str, "%06x", id);

    int light_raw   = (buf[4] << 8) | (buf[5]);
    float light_lux = light_raw * 10;        // Lux
    int battery_mv  = (buf[6] * 20);         // mV
    int battery_lvl = battery_mv < 1680 ? 0 : (battery_mv - 1680) / 16; // 1.4V-3.0V is 0-100
    int flags       = buf[7]; // to find the wind msb
    int temp_raw    = ((buf[7] & 0x03) << 8) | (buf[8]);
    float temp_c    = (temp_raw - 400) * 0.1f;
    int humidity    = (buf[9]);
    int wind_avg    = ((buf[7] & 0x10) << 4) | (buf[10]);
    int wind_dir    = ((buf[7] & 0x20) << 3) | (buf[11]);
    int wind_max    = ((buf[7] & 0x40) << 2) | (buf[12]);
    int uv_index    = (buf[13]);

    int rain_raw    = ((buf[19] & 0x0f) << 8) | buf[20];
    float rain      = rain_raw * 0.1f;

    char g1[12];
    sprintf(g1, "%02x %02x %02x %02x", buf[15], buf[16], buf[17], buf[18]);

    char g2[12];
    sprintf(g2, "%02x %02x %02x %02x", buf[21], buf[22], buf[23], buf[24]);

    char g3[15];
    sprintf(g3, "%02x %02x %02x %02x %02x", buf[25], buf[26], buf[27], buf[28], buf[29]);

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
            "rain_mm",          "Total rainfall",   DATA_FORMAT, "%.1f mm", DATA_DOUBLE, rain,
            "flags",            "Flags",            DATA_FORMAT, "%02x", DATA_INT, flags,
            "g1",               "Group 1",          DATA_STRING, g1,
            "g2",               "Group 2",          DATA_STRING, g2,
            "g3",               "Group 3",          DATA_STRING, g3,
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
        "rain_mm",
        "flags",
        "g1",
        "g2",
        "g3",
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
