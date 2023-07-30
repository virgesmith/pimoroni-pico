
#pragma once

#include "include/bsec_datatypes.h"
#include "include/bsec_interface.h"

// hack to access private members
#define private public
#include "bme68x.hpp"
#undef private

#include <string>

#define BME68X_ERROR            INT8_C(-1)
#define BME68X_WARNING          INT8_C(1)


class BSEC
{
public:
	int64_t nextCall;			// Stores the time when the algorithm has to be called next in ms
	float iaq, rawTemperature, pressure, rawHumidity, gasResistance, stabStatus, runInStatus, temperature, humidity,
	      staticIaq, co2Equivalent, breathVocEquivalent, compGasValue, gasPercentage;
	uint8_t iaqAccuracy, staticIaqAccuracy, co2Accuracy, breathVocAccuracy, compGasAccuracy, gasPercentageAccuracy;
	int64_t outputTimestamp;	// Timestamp in ms of the output
	bme68x_conf conf;
	bme68x_heatr_conf heatrConf;

	BSEC(pimoroni::I2C* i2c, uint8_t address, float temp_offset = 0.0);

  std::string version();

  // BSEC_OK = 0, < 0 is an error state
  // see bsec_datatypes.h
  bsec_library_return_t bsec_status() const {
    return m_bsec_status;
  }

  // BME68X_OK = 0, < 0 is an error state
  // see bme68x_defs.h
  int8_t bme68x_status() const {
    return m_bme68x_status;
  }

  pimoroni::BME68X& sensor() {
    return m_bme68x;
  }

  bme68x_dev* raw_sensor() {
    return &m_bme68x.device;
  }

  bme68x_data& raw_data() {
    return m_data;
  }

	/**
	 * @brief Function that sets the desired sensors and the sample rates
	 * @param sensorList	: The list of output sensors
	 * @param nSensors		: Number of outputs requested
	 * @param sampleRate	: The sample rate of requested sensors
	 */
	void updateSubscription(bsec_virtual_sensor_t sensorList[], uint8_t nSensors, float sampleRate = BSEC_SAMPLE_RATE_ULP);

	/**
	 * @brief Callback from the user to trigger reading of data from the BME68x, process and store outputs
	 * @return true if there are new outputs. false otherwise
	 */
	bool run();

	/**
	 * @brief Function to get the state of the algorithm to save to non-volatile memory
	 * @param state			: Pointer to a memory location that contains the state
	 */
	void getState(uint8_t *state);

	/**
	 * @brief Function to set the state of the algorithm from non-volatile memory
	 * @param state			: Pointer to a memory location that contains the state
	 */
	void setState(uint8_t *state);

	/**
	 * @brief Function to set the configuration of the algorithm from memory
	 * @param state			: Pointer to a memory location that contains the configuration
	 */
	void setConfig(const uint8_t *config);

private:
  bsec_library_return_t m_bsec_status;
  int8_t m_bme68x_status;
  pimoroni::BME68X m_bme68x;
	bme68x_data m_data;
	float m_temp_offset;

	/**
	 * @brief Read data from the BME68x and process it
	 * @param currTimeNs: Current time in ns
	 * @param bme68xSettings: BME68x sensor's settings
	 * @return true if there are new outputs. false otherwise
	 */
	bool readProcessData(int64_t currTimeNs, bsec_bme_settings_t bme68xSettings);

	/**
	 * @brief Set the BME68x sensor's configuration
	 * @param bme68xSettings: Settings to configure the BME68x sensor
	 * @return BME68x return code. BME68X_OK for success, failure otherwise
	 */
	int8_t setBme68xConfig(bsec_bme_settings_t bme68xSettings);

	/**
	 * @brief Function to zero the outputs
	 */
	void zeroOutputs();

  /**
   * @brief Function to zero the inputs
   */
  void zeroInputs();
};

