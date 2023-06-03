// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2020-2022 Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>
 * Copyright (C) 2023 Niklas Ekström <mail@niklasekstrom.nu>
 */

#ifndef LIBGPIODSPIBITBANG_H
#define LIBGPIODSPIBITBANG_H

#include <string>

#include "gpiod.h"

#include "board.hpp"
#include "spiInterface.hpp"

/*!
 * \file libgpiodSpiBitbang.hpp
 * \class libgpiodSpiBitbang
 * \brief concrete class between spi implementation and gpio bitbang
 * \author Niklas Ekström
 */

struct gpiod_chip;
struct gpiod_line;

class LibgpiodSpiBitbang : public SPIInterface {
 public:
	LibgpiodSpiBitbang(const spi_pins_conf_t *pin_conf, const std::string &dev, uint8_t verbose);
	virtual ~LibgpiodSpiBitbang();

	virtual int spi_put(uint8_t cmd, uint8_t *tx, uint8_t *rx, uint32_t len) override;
	virtual int spi_put(uint8_t *tx, uint8_t *rx, uint32_t len) override;
	virtual int spi_wait(uint8_t cmd, uint8_t mask, uint8_t cond, uint32_t timeout, bool verbose = false) override;

 private:
	enum SPI_CS_mode {
		SPI_CS_AUTO   = 0,
		SPI_CS_MANUAL = 1
	};

	void setCSmode(int cs_mode) { _cs_mode = cs_mode; }
	void setCs();
	void clearCs();

#ifndef GPIOD_APIV2
	gpiod_line *get_line(unsigned int offset, int val, int dir);
#endif

	int update_pins(int cs, int sck, int mosi);
	int read_miso();

	int spi_wr_then_rd(const uint8_t *tx_data, uint32_t tx_len, uint8_t *rx_data, uint32_t rx_len);
	int spi_wr_and_rd(uint32_t writecnt, const uint8_t *writearr, uint8_t *readarr);

	bool _verbose;

#ifdef GPIOD_APIV2
	unsigned int _cs_pin;
	unsigned int _sck_pin;
	unsigned int _mosi_pin;
	unsigned int _miso_pin;
#else
	int _cs_pin;
	int _sck_pin;
	int _mosi_pin;
	int _miso_pin;
#endif

	gpiod_chip *_chip;

#ifdef GPIOD_APIV2
	gpiod_request_config *_cs_req_cfg;
	gpiod_request_config *_sck_req_cfg;
	gpiod_request_config *_mosi_req_cfg;
	gpiod_request_config *_miso_req_cfg;

	gpiod_line_config *_cs_line_cfg;
	gpiod_line_config *_sck_line_cfg;
	gpiod_line_config *_mosi_line_cfg;
	gpiod_line_config *_miso_line_cfg;

	gpiod_line_settings *_cs_settings;
	gpiod_line_settings *_sck_settings;
	gpiod_line_settings *_mosi_settings;
	gpiod_line_settings *_miso_settings;

	gpiod_line_request *_cs_request;
	gpiod_line_request *_sck_request;
	gpiod_line_request *_mosi_request;
	gpiod_line_request *_miso_request;
#else
	gpiod_line *_cs_line;
	gpiod_line *_sck_line;
	gpiod_line *_mosi_line;
	gpiod_line *_miso_line;
#endif

	int _cs_mode;

	int _curr_cs;
	int _curr_sck;
	int _curr_mosi;
};

#endif
