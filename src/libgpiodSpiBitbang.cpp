// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2020-2022 Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>
 * Copyright (C) 2023 Niklas Ekström <mail@niklasekstrom.nu>
 */

#include "libgpiodSpiBitbang.hpp"

#include <gpiod.h>
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <stdexcept>

#include "display.hpp"

#define DEBUG 1

#ifdef DEBUG
#define display(...) \
	do { \
		if (_verbose) fprintf(stdout, __VA_ARGS__); \
	} while(0)
#else
#define display(...) do {} while(0)
#endif

LibgpiodSpiBitbang::LibgpiodSpiBitbang(
		const spi_pins_conf_t *pin_conf,
		const std::string &dev,
		uint8_t verbose)
{
	_verbose = verbose;

	_cs_pin = pin_conf->cs_pin;
	_sck_pin = pin_conf->sck_pin;
	_mosi_pin = pin_conf->mosi_pin;
	_miso_pin = pin_conf->miso_pin;

	std::string chip_dev = dev;
	if (chip_dev.empty())
		chip_dev = "/dev/gpiochip0";

	display("libgpiod spi bitbang driver, dev=%s, cs_pin=%d, sck_pin=%d, mosi_pin=%d, miso_pin=%d\n",
		chip_dev.c_str(), _cs_pin, _sck_pin, _mosi_pin, _miso_pin);

	if (chip_dev.length() < 14 || chip_dev.substr(0, 13) != "/dev/gpiochip") {
		display("Invalid gpio chip %s, should be /dev/gpiochipX\n", chip_dev.c_str());
		throw std::runtime_error("Invalid gpio chip\n");
	}

	/* Validate pins */
#ifdef GPIOD_APIV2
	const unsigned int pins[] = {_cs_pin, _sck_pin, _mosi_pin, _miso_pin};
#else
	const int pins[] = {_cs_pin, _sck_pin, _mosi_pin, _miso_pin};
#endif
	for (uint32_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
		if (pins[i] < 0 || pins[i] >= 1000) {
			display("Pin %d is outside of valid range\n", pins[i]);
			throw std::runtime_error("A pin is outside of valid range\n");
		}

		for (uint32_t j = i + 1; j < sizeof(pins) / sizeof(pins[0]); j++) {
			if (pins[i] == pins[j]) {
				display("Two or more pins are assigned to the same pin number %d\n", pins[i]);
				throw std::runtime_error("Two or more pins are assigned to the same pin number\n");
			}
		}
	}

	_chip = gpiod_chip_open(chip_dev.c_str());
	if (!_chip) {
		display("Unable to open gpio chip %s\n", chip_dev.c_str());
		throw std::runtime_error("Unable to open gpio chip\n");
	}

#ifdef GPIOD_APIV2
	_cs_req_cfg = gpiod_request_config_new();
	_sck_req_cfg = gpiod_request_config_new();
	_mosi_req_cfg = gpiod_request_config_new();
	_miso_req_cfg = gpiod_request_config_new();

	gpiod_request_config_set_consumer(_cs_req_cfg, "_cs");
	gpiod_request_config_set_consumer(_sck_req_cfg, "_sck");
	gpiod_request_config_set_consumer(_mosi_req_cfg, "_mosi");
	gpiod_request_config_set_consumer(_miso_req_cfg, "_miso");

	_cs_settings = gpiod_line_settings_new();
	_sck_settings = gpiod_line_settings_new();
	_mosi_settings = gpiod_line_settings_new();
	_miso_settings = gpiod_line_settings_new();

	gpiod_line_settings_set_direction(
		_cs_settings, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_direction(
		_sck_settings, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_direction(
		_mosi_settings, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_direction(
		_miso_settings, GPIOD_LINE_DIRECTION_INPUT);

	gpiod_line_settings_set_bias(
		_cs_settings, GPIOD_LINE_BIAS_DISABLED);
	gpiod_line_settings_set_bias(
		_sck_settings, GPIOD_LINE_BIAS_DISABLED);
	gpiod_line_settings_set_bias(
		_mosi_settings, GPIOD_LINE_BIAS_DISABLED);
	gpiod_line_settings_set_bias(
		_miso_settings, GPIOD_LINE_BIAS_DISABLED);

	_cs_line_cfg = gpiod_line_config_new();
	_sck_line_cfg = gpiod_line_config_new();
	_mosi_line_cfg = gpiod_line_config_new();
	_miso_line_cfg = gpiod_line_config_new();

	gpiod_line_config_add_line_settings(
		_cs_line_cfg, &_cs_pin, 1, _cs_settings);
	gpiod_line_config_add_line_settings(
		_sck_line_cfg, &_sck_pin, 1, _sck_settings);
	gpiod_line_config_add_line_settings(
		_mosi_line_cfg, &_mosi_pin, 1, _mosi_settings);
	gpiod_line_config_add_line_settings(
		_miso_line_cfg, &_miso_pin, 1, _miso_settings);

	_cs_request = gpiod_chip_request_lines(
		_chip, _cs_req_cfg, _cs_line_cfg);
	_sck_request = gpiod_chip_request_lines(
		_chip, _sck_req_cfg, _sck_line_cfg);
	_mosi_request = gpiod_chip_request_lines(
		_chip, _mosi_req_cfg, _mosi_line_cfg);
	_miso_request = gpiod_chip_request_lines(
		_chip, _miso_req_cfg, _miso_line_cfg);
#else
	_cs_line = get_line(_cs_pin, 1, GPIOD_LINE_REQUEST_DIRECTION_OUTPUT);
	_sck_line = get_line(_sck_pin, 0, GPIOD_LINE_REQUEST_DIRECTION_OUTPUT);
	_mosi_line = get_line(_mosi_pin, 0, GPIOD_LINE_REQUEST_DIRECTION_OUTPUT);
	_miso_line = get_line(_miso_pin, 0, GPIOD_LINE_REQUEST_DIRECTION_INPUT);
#endif

	_curr_cs = 0;
	_curr_sck = 1;
	_curr_mosi = 1;

	update_pins(1, 0, 0);

	_cs_mode = SPI_CS_AUTO;
}

LibgpiodSpiBitbang::~LibgpiodSpiBitbang()
{
#ifdef GPIOD_APIV2
	if (_miso_request)
		gpiod_line_request_release(_miso_request);
	if (_miso_line_cfg)
		gpiod_line_config_free(_miso_line_cfg);
	if (_miso_settings)
		gpiod_line_settings_free(_miso_settings);

	if (_mosi_request)
		gpiod_line_request_release(_mosi_request);
	if (_mosi_line_cfg)
		gpiod_line_config_free(_mosi_line_cfg);
	if (_mosi_settings)
		gpiod_line_settings_free(_mosi_settings);

	if (_sck_request)
		gpiod_line_request_release(_sck_request);
	if (_sck_line_cfg)
		gpiod_line_config_free(_sck_line_cfg);
	if (_sck_settings)
		gpiod_line_settings_free(_sck_settings);

	if (_cs_request)
		gpiod_line_request_release(_cs_request);
	if (_cs_line_cfg)
		gpiod_line_config_free(_cs_line_cfg);
	if (_cs_settings)
		gpiod_line_settings_free(_cs_settings);
#else
	if (_miso_line)
		gpiod_line_release(_miso_line);

	if (_mosi_line)
		gpiod_line_release(_mosi_line);

	if (_sck_line)
		gpiod_line_release(_sck_line);

	if (_cs_line)
		gpiod_line_release(_cs_line);
#endif

	if (_chip)
		gpiod_chip_close(_chip);
}

#ifndef GPIOD_APIV2
gpiod_line *LibgpiodSpiBitbang::get_line(unsigned int offset, int val, int dir)
{
	gpiod_line *line = gpiod_chip_get_line(_chip, offset);
	if (!line) {
		display("Unable to get gpio line %u\n", offset);
		throw std::runtime_error("Unable to get gpio line\n");
	}

	gpiod_line_request_config config = {
		.consumer = "openFPGALoader",
		.request_type = dir,
		.flags = 0,
	};

	int ret = gpiod_line_request(line, &config, val);
	if (ret < 0) {
		display("Error requesting gpio line %u\n", offset);
		throw std::runtime_error("Error requesting gpio line\n");
	}

	return line;
}
#endif

int LibgpiodSpiBitbang::update_pins(int cs, int sck, int mosi)
{
	if (mosi != _curr_mosi) {
#ifdef GPIOD_APIV2
		if (gpiod_line_request_set_value(_mosi_request, _mosi_pin,
			(mosi == 0) ? GPIOD_LINE_VALUE_INACTIVE :
				GPIOD_LINE_VALUE_ACTIVE) < 0)
#else
		if (gpiod_line_set_value(_mosi_line, mosi) < 0)
#endif
			display("Unable to set gpio pin mosi\n");
	}

	if (sck != _curr_sck) {
#ifdef GPIOD_APIV2
		if (gpiod_line_request_set_value(_sck_request, _sck_pin,
			(sck == 0) ? GPIOD_LINE_VALUE_INACTIVE :
				GPIOD_LINE_VALUE_ACTIVE) < 0)
#else
		if (gpiod_line_set_value(_sck_line, sck) < 0)
#endif
			display("Unable to set gpio pin sck\n");
	}

	if (cs != _curr_cs) {
#ifdef GPIOD_APIV2
		if (gpiod_line_request_set_value(_cs_request, _cs_pin,
			(cs == 0) ? GPIOD_LINE_VALUE_INACTIVE :
				GPIOD_LINE_VALUE_ACTIVE) < 0)
#else
		if (gpiod_line_set_value(_cs_line, cs) < 0)
#endif
			display("Unable to set gpio pin cs\n");
	}

	_curr_cs = cs;
	_curr_sck = sck;
	_curr_mosi = mosi;

	return 0;
}

int LibgpiodSpiBitbang::read_miso()
{
#ifdef GPIOD_APIV2
	gpiod_line_value req = gpiod_line_request_get_value(_miso_request, _miso_pin);
    if (req == GPIOD_LINE_VALUE_ERROR)
    {
		display("Error reading miso line\n");
		throw std::runtime_error("Error reading miso line\n");
	}
	return (req == GPIOD_LINE_VALUE_ACTIVE) ? 1 : 0;
#else
	return gpiod_line_get_value(_miso_line);
#endif
}

void LibgpiodSpiBitbang::setCs()
{
    update_pins(1, _curr_sck, _curr_mosi);
}

void LibgpiodSpiBitbang::clearCs()
{
    update_pins(0, _curr_sck, _curr_mosi);
}

int LibgpiodSpiBitbang::spi_wr_then_rd(const uint8_t *tx_data, uint32_t tx_len, uint8_t *rx_data, uint32_t rx_len)
{
	setCSmode(SPI_CS_MANUAL);
	clearCs();
	uint32_t ret = spi_wr_and_rd(tx_len, tx_data, NULL);
	if (ret != 0) {
		printf("%s : write error %d %d\n", __func__, ret, tx_len);
	} else {
		ret = spi_wr_and_rd(rx_len, NULL, rx_data);
		if (ret != 0) {
			printf("%s : read error\n", __func__);
		}
	}
	setCs();
	setCSmode(SPI_CS_AUTO);
	return ret;
}

/* Returns 0 upon success, a negative number upon errors. */
int LibgpiodSpiBitbang::spi_wr_and_rd(uint32_t writecnt, const uint8_t * writearr, uint8_t * readarr)
{
	if (_cs_mode == SPI_CS_AUTO) {
		clearCs();
	}

    const uint8_t *wp = writearr;
    uint8_t *rp = readarr;

    // FIXME: Please note that this routine is hardcoded to handle mode 0 only.

    for (int i = 0; i < writecnt; i++) {
        uint8_t wv = 0;
        if (wp)
            wv = *wp++;
        uint8_t rv = 0;
        for (int j = 0; j < 8; j++) {
            update_pins(_curr_cs, 0, wv & 0x80 ? 1 : 0);
            wv <<= 1;
            update_pins(_curr_cs, 1, _curr_mosi);
            rv = (rv << 1) | (read_miso() ? 1 : 0);
            update_pins(_curr_cs, 0, _curr_mosi);
        }
        if (rp)
            *rp++ = rv;
    }

	if (_cs_mode == SPI_CS_AUTO) {
		setCs();
	}

	return 0;
}

int LibgpiodSpiBitbang::spi_put(uint8_t cmd, uint8_t *tx, uint8_t *rx, uint32_t len)
{
	uint32_t xfer_len = len + 1;
	uint8_t jtx[xfer_len];
	uint8_t jrx[xfer_len];

	jtx[0] = cmd;
	if (tx != NULL)
		memcpy(jtx+1, tx, len);

	/* send first already stored cmd,
	 * in the same time store each byte
	 * to next
	 */
	spi_wr_and_rd(xfer_len, jtx, (rx != NULL)?jrx:NULL);

	if (rx != NULL)
		memcpy(rx, jrx+1, len);

	return 0;
}

int LibgpiodSpiBitbang::spi_put(uint8_t *tx, uint8_t *rx, uint32_t len)
{
	return spi_wr_and_rd(len, tx, rx);
}

int LibgpiodSpiBitbang::spi_wait(uint8_t cmd, uint8_t mask, uint8_t cond, uint32_t timeout, bool verbose)
{
	uint8_t rx;
	uint32_t count = 0;

	setCSmode(SPI_CS_MANUAL);
	clearCs();
	spi_wr_and_rd(1, &cmd, NULL);
	do {
		spi_wr_and_rd(1, NULL, &rx);
		count ++;
		if (count == timeout) {
			printf("timeout: %2x %d\n", rx, count);
			break;
		}

		if (verbose) {
			printf("%02x %02x %02x %02x\n", rx, mask, cond, count);
		}
	} while((rx & mask) != cond);
	setCs();
	setCSmode(SPI_CS_AUTO);

	if (count == timeout) {
		printf("%x\n", rx);
		std::cout << "wait: Error" << std::endl;
		return -ETIME;
	} else
		return 0;
}
