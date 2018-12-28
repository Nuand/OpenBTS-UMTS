/*
 * Device support for bladeRF, based on:
 * Written by Robert Ghilduta <robert.ghilduta@gmail.com>
 *
 * Device support for Ettus Research UHD driver 
 * Written by Tom Tsou <tom@tsou.cc>
 *
 * Copyright 2010-2011 Free Software Foundation, Inc.
 * Copyright 2014 Ettus Research LLC
 * Copyright 2017 Ettus Research LLC
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * See the COPYING file in the main directory for details.
 */

#include "Threads.h"
#include "Logger.h"
#include "bladeRFDevice.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* Timestamped ring buffer size */
#define SAMPLE_BUF_SZ		(1 << 20)

bladeRFDevice::bladeRFDevice(double rate)
	: tx_gain_min(0.0), tx_gain_max(0.0),
	  rx_gain_min(0.0), rx_gain_max(0.0),
	  tx_rate(rate), rx_rate(rate), tx_freq(0.0), rx_freq(0.0),
	  started(false), aligned(false), rx_pkt_cnt(0), drop_cnt(0),
	  prev_ts(0), ts_offset(0), rx_buffer(NULL), firstGoodTXDone(false)
{
}

bladeRFDevice::~bladeRFDevice()
{
	stop();
}

void bladeRFDevice::init_gains()
{
    setTxGain(10);
    setRxGain(25);
	return;
}

int bladeRFDevice::set_master_clk(double clk_rate)
{
	return 0;
}

int bladeRFDevice::set_rates(double tx_rate, double rx_rate)
{
	double offset_limit = 1.0;
	double tx_offset, rx_offset;
    unsigned rx_actual, tx_actual;
    int status;

    status = bladerf_set_sample_rate(dev, BLADERF_MODULE_TX, tx_rate, &tx_actual);
    if (status < 0) {
        LOG(ALERT) << "Failed setting TX sampling rate";
        return -1;
    }

    LOG(ALERT) << "TX sampling rate: " << tx_actual << " sps";
    this->tx_rate = tx_actual;

    status = bladerf_set_sample_rate(dev, BLADERF_MODULE_RX, rx_rate, &rx_actual);
    if (status < 0) {
        LOG(ALERT) << "Failed setting RX sampling rate";
        return -1;
    }

    LOG(ALERT) << "RX sampling rate: " << rx_actual << " sps";
    this->rx_rate = rx_actual;

	tx_offset = fabs(this->tx_rate - tx_rate);
	rx_offset = fabs(this->rx_rate - rx_rate);
	if ((tx_offset > offset_limit) || (rx_offset > offset_limit)) {
		LOG(ALERT) << "Actual sample rate differs from desired rate";
		LOG(ALERT) << "Tx/Rx (" << this->tx_rate << "/"
			  << this->rx_rate << ")" << std::endl;
		return -1;
	}

	return 0;
}

double bladeRFDevice::setTxGain(double db)
{
    double tx_gain;
    int hw_gain;

    bladerf_set_gain(dev, BLADERF_CHANNEL_TX(0), db);
    bladerf_get_gain(dev, BLADERF_CHANNEL_TX(0), &hw_gain);

    tx_gain = (double)hw_gain;

    LOG(INFO) << "Set TX gain to " << tx_gain << "dB";

    return tx_gain;
}

double bladeRFDevice::setRxGain(double db)
{
    int hw_gain;
    double rx_gain;

    bladerf_set_gain(dev, BLADERF_CHANNEL_RX(0), db);
    bladerf_get_gain(dev, BLADERF_CHANNEL_RX(0), &hw_gain);

    rx_gain = (double)hw_gain;
    LOG(INFO) << "Set RX gain to " << rx_gain << "dB";

    return rx_gain;
}

bool bladeRFDevice::open(const std::string &args, bool extref)
{
    int status;

    status = bladerf_open(&dev, NULL);
    if (status != 0) {
		LOG(ALERT) << "Could not open any bladeRF device";
        return false;
    }

    status = bladerf_is_fpga_configured(dev);
    if (status != 1) {
		LOG(ALERT) << "bladeRF FPGA is not configured";
        return false;
    }

#define NUM_BUFFERS 1024
#define NUM_XFERS 16
#define BUF_SIZE 1024
#define TIMEOUT_MS 0
    status = bladerf_sync_config(dev,
            BLADERF_RX_X1,
            BLADERF_FORMAT_SC16_Q11_META,
            NUM_BUFFERS,
            BUF_SIZE,
            NUM_XFERS,
            TIMEOUT_MS);

    if (status < 0) {
        LOG(ALERT) << "bladeRF RX config failed";
        return false;
    }

    status = bladerf_sync_config(dev,
            BLADERF_TX_X1,
            BLADERF_FORMAT_SC16_Q11_META,
            NUM_BUFFERS,
            BUF_SIZE,
            NUM_XFERS,
            TIMEOUT_MS);

    if (status < 0) {
        LOG(ALERT) << "bladeRF TX config failed";
        return false;
    }

	set_rates(tx_rate, rx_rate);

	/* Initialize and shadow gain values */
	init_gains();

	LOG(INFO) << "\n";

	return true;
}

bool bladeRFDevice::flush_recv(size_t num_pkts)
{
	LOG(INFO) << "Initial timestamp " << ts_initial << std::endl;

	return true;
}

bool bladeRFDevice::restart()
{
    return true;
}

bool bladeRFDevice::start()
{
    int status;
	LOG(INFO) << "Starting bladeRF...";

	if (started) {
		LOG(ERR) << "Device already running";
		return false;
	}

	setPriority();

    status = bladerf_enable_module(dev, BLADERF_MODULE_RX, 1);
    if (status < 0) {
        return false;
    }

    status = bladerf_enable_module(dev, BLADERF_MODULE_TX, 1);
    if (status < 0) {
        return false;
    }

	/* Start receive streaming */
	if (!restart()) {
		return false;
    }

	started = true;
	return true;
}

/*
 * Stop the bladeRF device
 *
 * Issue a stop streaming command and cancel the asynchronous message loop.
 * Deallocate the asynchronous thread object since we can't reuse it anyways.
 * A new thread object is allocated on device start.
 */
bool bladeRFDevice::stop()
{
    int status;

	if (!started) {
		LOG(ERR) << "Device not running";
		return false;
	}

    status = bladerf_enable_module(dev, BLADERF_MODULE_RX, 0);
    if (status < 0)
        return false;

    status = bladerf_enable_module(dev, BLADERF_MODULE_TX, 0);
    if (status < 0) {
        return false;
    }

	started = false;
	return true;
}

void bladeRFDevice::setPriority()
{
	return;
}

int bladeRFDevice::readSamples(short *buf, int len, bool *overrun,
			    long long timestamp, bool *underrun, unsigned *RSSI)
{
    int status;
    struct bladerf_metadata meta;

    memset(&meta, 0, sizeof(meta));
    meta.timestamp = timestamp;

    status = bladerf_sync_rx(dev, buf, len, &meta, TIMEOUT_MS);
    if (status < -14) {
        memset(buf, 0, sizeof(short) * len);
    }

    return len;
}

int bladeRFDevice::writeSamples(short *buf, int len,
			     bool *underrun, long long ts)
{
    int status;
    struct bladerf_metadata meta;

    memset(&meta, 0, sizeof(meta));

    meta.flags = 0;
    meta.timestamp = ts;

    if (firstGoodTXDone)
        meta.flags = BLADERF_META_FLAG_TX_UPDATE_TIMESTAMP;
    else
        meta.flags = BLADERF_META_FLAG_TX_BURST_START;

    status = bladerf_sync_tx(dev, buf, len, &meta, TIMEOUT_MS);

    if (status == 0) {
        firstGoodTXDone = true;
    }

    if (status < 0) {
        uint64_t curr_ts;
        LOG(ALERT) << "TX Error " << status;
        status = bladerf_get_timestamp(dev, BLADERF_TX, &curr_ts);
        LOG(ALERT) << len << " Requested " << ts << " now: " << curr_ts;
    }

    return len;
}

bool bladeRFDevice::setTxFreq(double wFreq)
{
    uint64_t actual;
    int status;

    status = bladerf_set_frequency(dev, BLADERF_MODULE_TX, (unsigned int)wFreq);
    if (status < 0) {
        LOG(ALERT) << "Failed setting TX frequency to: " << wFreq;
        return false;
    }

    status = bladerf_get_frequency(dev, BLADERF_MODULE_TX, &actual);
    if (status < 0) {
        LOG(ALERT) << "Failed reading TX frequency";
        return false;
    }

    LOG(ALERT) << "Set TX frequency to: " << actual;
    tx_freq = actual;

	return true;
}

bool bladeRFDevice::setRxFreq(double wFreq)
{
    uint64_t actual;
    int status;
    status = bladerf_set_frequency(dev, BLADERF_MODULE_RX, (unsigned int)wFreq);
    if (status < 0) {
        LOG(ALERT) << "Failed setting RX frequency to: " << wFreq;
        return false;
    }

    status = bladerf_get_frequency(dev, BLADERF_MODULE_RX, &actual);
    if (status < 0) {
        LOG(ALERT) << "Failed reading RX frequency";
        return false;
    }

    LOG(ALERT) << "Set RX frequency to: " << actual;
    rx_freq = actual;

	return true;
}
