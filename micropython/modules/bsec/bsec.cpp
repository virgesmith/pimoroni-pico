#include "bsec.h"


BSEC::BSEC(pimoroni::I2C* i2c, uint8_t address, float temp_offset) : m_bme68x(i2c, address), m_temp_offset(temp_offset)
{
	zeroInputs();
  zeroOutputs();

  m_bme68x_status = m_bme68x.init() ? BME68X_OK : -1;
  m_bsec_status = bsec_init();
	//m_bme_status = bme68x_init(&m_bme68x.device);
}

std::string BSEC::version()
{
  bsec_version_t bsec_version;
  /*bsec_library_return_t  ret = */bsec_get_version(&bsec_version);
  return std::to_string(bsec_version.major) + "." + std::to_string(bsec_version.minor) + "."
    + std::to_string(bsec_version.major_bugfix) + "." + std::to_string(bsec_version.minor_bugfix);
}


/**
 * @brief Function that sets the desired sensors and the sample rates
 */
void BSEC::updateSubscription(bsec_virtual_sensor_t sensorList[], uint8_t nSensors, float sampleRate)
{
	bsec_sensor_configuration_t virtualSensors[BSEC_NUMBER_OUTPUTS],
	        sensorSettings[BSEC_MAX_PHYSICAL_SENSOR];
	uint8_t nSensorSettings = BSEC_MAX_PHYSICAL_SENSOR;

	for (uint8_t i = 0; i < nSensors; i++) {
		virtualSensors[i].sensor_id = sensorList[i];
		virtualSensors[i].sample_rate = sampleRate;
	}

	m_bsec_status = bsec_update_subscription(virtualSensors, nSensors, sensorSettings, &nSensorSettings);
}

/**
 * @brief Callback from the user to trigger reading of data from the BME68x, process and store outputs
 */
bool BSEC::run()
{
	bool newData = false;
	/* Check if the time has arrived to call do_steps() */
	int64_t callTimeMs = to_ms_since_boot(get_absolute_time());

	if (callTimeMs >= nextCall) {

		bsec_bme_settings_t bme68xSettings;

		int64_t callTimeNs = callTimeMs * INT64_C(1000000);

		m_bsec_status = bsec_sensor_control(callTimeNs, &bme68xSettings);
		if (m_bsec_status < BSEC_OK)
			return false;

		nextCall = bme68xSettings.next_call / INT64_C(1000000); // Convert from ns to ms

		m_bme68x_status = setBme68xConfig(bme68xSettings);
		if (m_bme68x_status != BME68X_OK) {
			return false;
		}

		newData = readProcessData(callTimeNs, bme68xSettings);
	}

	return newData;
}

/**
 * @brief Function to get the state of the algorithm to save to non-volatile memory
 */
void BSEC::getState(uint8_t *state)
{
	uint8_t workBuffer[BSEC_MAX_STATE_BLOB_SIZE];
	uint32_t n_serialized_state = BSEC_MAX_STATE_BLOB_SIZE;
	m_bsec_status = bsec_get_state(0, state, BSEC_MAX_STATE_BLOB_SIZE, workBuffer, BSEC_MAX_STATE_BLOB_SIZE, &n_serialized_state);
}

/**
 * @brief Function to set the state of the algorithm from non-volatile memory
 */
void BSEC::setState(uint8_t *state)
{
	uint8_t workBuffer[BSEC_MAX_STATE_BLOB_SIZE];

	m_bsec_status = bsec_set_state(state, BSEC_MAX_STATE_BLOB_SIZE, workBuffer, BSEC_MAX_STATE_BLOB_SIZE);
}

/**
 * @brief Function to set the configuration of the algorithm from memory
 */
void BSEC::setConfig(const uint8_t *state)
{
	uint8_t workBuffer[BSEC_MAX_PROPERTY_BLOB_SIZE];

	m_bsec_status = bsec_set_configuration(state, BSEC_MAX_PROPERTY_BLOB_SIZE, workBuffer, sizeof(workBuffer));
}


/**
 * @brief Read data from the BME68x and process it
 */
bool BSEC::readProcessData(int64_t currTimeNs, bsec_bme_settings_t bme68xSettings)
{
	bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR]; // Temperature, Pressure, Humidity & Gas Resistance
	uint8_t nInputs = 0, nOutputs = 0, nFields = 0;

	if (bme68xSettings.process_data)
	{
		m_bme68x_status = bme68x_get_data(BME68X_FORCED_MODE, &m_data, &nFields, &m_bme68x.device);
		if (m_bme68x_status != BME68X_OK)
		{
			return false;
		}

		if (nFields)
		{
			if (bme68xSettings.process_data & BSEC_PROCESS_TEMPERATURE)
			{
				inputs[nInputs].sensor_id = BSEC_INPUT_TEMPERATURE;
				inputs[nInputs].signal = m_data.temperature;

				inputs[nInputs].time_stamp = currTimeNs;
				++nInputs;
				/* Temperature offset from the real temperature due to external heat sources */
				inputs[nInputs].sensor_id = BSEC_INPUT_HEATSOURCE;
				inputs[nInputs].signal = m_temp_offset;
				inputs[nInputs].time_stamp = currTimeNs;
				++nInputs;
			}
			if (bme68xSettings.process_data & BSEC_PROCESS_HUMIDITY)
			{
				inputs[nInputs].sensor_id = BSEC_INPUT_HUMIDITY;
				inputs[nInputs].signal = m_data.humidity;
				inputs[nInputs].time_stamp = currTimeNs;
				++nInputs;
			}
			if (bme68xSettings.process_data & BSEC_PROCESS_PRESSURE)
			{
				inputs[nInputs].sensor_id = BSEC_INPUT_PRESSURE;
				inputs[nInputs].signal = m_data.pressure;
				inputs[nInputs].time_stamp = currTimeNs;
				++nInputs;
			}
			if (bme68xSettings.process_data & BSEC_PROCESS_GAS)
			{
				/* Check whether gas_valid flag is set */
				if (m_data.status & BME68X_GASM_VALID_MSK)
				{
					inputs[nInputs].sensor_id = BSEC_INPUT_GASRESISTOR;
					inputs[nInputs].signal = m_data.gas_resistance;
					inputs[nInputs].time_stamp = currTimeNs;
					++nInputs;
				}
			}
		}
	}

	if (nInputs)
	{
		nOutputs = BSEC_NUMBER_OUTPUTS;
		bsec_output_t _outputs[BSEC_NUMBER_OUTPUTS];

		m_bsec_status = bsec_do_steps(inputs, nInputs, _outputs, &nOutputs);
		if (m_bsec_status != BSEC_OK)
			return false;

		zeroOutputs();

		if (nOutputs > 0)
		{
			outputTimestamp = _outputs[0].time_stamp / 1000000; // Convert from ns to ms

			for (uint8_t i = 0; i < nOutputs; i++)
			{
				switch (_outputs[i].sensor_id)
				{
					case BSEC_OUTPUT_IAQ:
						iaq = _outputs[i].signal;
						iaqAccuracy = _outputs[i].accuracy;
						break;
					case BSEC_OUTPUT_STATIC_IAQ:
						staticIaq = _outputs[i].signal;
						staticIaqAccuracy = _outputs[i].accuracy;
						break;
					case BSEC_OUTPUT_CO2_EQUIVALENT:
						co2Equivalent = _outputs[i].signal;
						co2Accuracy = _outputs[i].accuracy;
						break;
					case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
						breathVocEquivalent = _outputs[i].signal;
						breathVocAccuracy = _outputs[i].accuracy;
						break;
					case BSEC_OUTPUT_RAW_TEMPERATURE:
						rawTemperature = _outputs[i].signal;
						break;
					case BSEC_OUTPUT_RAW_PRESSURE:
						pressure = _outputs[i].signal;
						break;
					case BSEC_OUTPUT_RAW_HUMIDITY:
						rawHumidity = _outputs[i].signal;
						break;
					case BSEC_OUTPUT_RAW_GAS:
						gasResistance = _outputs[i].signal;
						break;
					case BSEC_OUTPUT_STABILIZATION_STATUS:
						stabStatus = _outputs[i].signal;
						break;
					case BSEC_OUTPUT_RUN_IN_STATUS:
						runInStatus = _outputs[i].signal;
						break;
					case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
						temperature = _outputs[i].signal;
						break;
					case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
						humidity = _outputs[i].signal;
						break;
					case BSEC_OUTPUT_GAS_PERCENTAGE:
						gasPercentage = _outputs[i].signal;
						gasPercentageAccuracy = _outputs[i].accuracy;
						break;
					default:
						continue;
				}
			}
			return true;
		}
	}

	return false;
}

/**
 * @brief Set the BME68x sensor's configuration
 */
int8_t BSEC::setBme68xConfig(bsec_bme_settings_t bme68xSettings)
{
	int8_t bme68xSts = BME68X_OK;
	uint16_t meas_period;
	uint8_t current_op_mode;

	/* Check if a forced-mode measurement should be triggered now */
    if (bme68xSettings.trigger_measurement)
	{
		conf.filter = BME68X_FILTER_OFF;
		conf.odr = BME68X_ODR_NONE;
		conf.os_hum = bme68xSettings.humidity_oversampling;
		conf.os_temp = bme68xSettings.temperature_oversampling;
		conf.os_pres = bme68xSettings.pressure_oversampling;

		bme68xSts = bme68x_set_conf(&conf, &m_bme68x.device);

		if (bme68xSts == BME68X_ERROR)
		{
			return bme68xSts;
		}

		heatrConf.enable = bme68xSettings.run_gas;
		heatrConf.heatr_temp = bme68xSettings.heater_temperature;
		heatrConf.heatr_dur = bme68xSettings.heater_duration;

		bme68xSts = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatrConf, &m_bme68x.device);

		if (bme68xSts == BME68X_ERROR)
		{
			return bme68xSts;
		}

		bme68xSts = bme68x_set_op_mode(BME68X_FORCED_MODE, &m_bme68x.device);

		if (bme68xSts == BME68X_ERROR)
		{
			return bme68xSts;
		}
		meas_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &m_bme68x.device) + heatrConf.heatr_dur*1000;
		/* Delay till the measurement is ready. Timestamp resolution in ms */
    sleep_us((uint32_t)meas_period);
	}

	/* Call the API to get current operation mode of the sensor */
    bme68xSts = bme68x_get_op_mode(&current_op_mode, &m_bme68x.device);
    /* When the measurement is completed and data is ready for reading, the sensor must be in bme68x_SLEEP_MODE.
     * Read operation mode to check whether measurement is completely done and wait until the sensor is no more
     * in bme68x_FORCED_MODE. */
    while (current_op_mode == BME68X_FORCED_MODE)
    {
        /* sleep for 5 ms */
        sleep_us(5 * 1000);
        bme68xSts = bme68x_get_op_mode(&current_op_mode, &m_bme68x.device);
    }
	return bme68xSts;
}

/**
 * @brief Function to zero the outputs
 */
void BSEC::zeroOutputs(void)
{
	temperature = 0.0f;
	pressure = 0.0f;
	humidity = 0.0f;
	gasResistance = 0.0f;
	rawTemperature = 0.0f;
	rawHumidity = 0.0f;
	stabStatus = 0.0f;
	runInStatus = 0.0f;
	iaq = 0.0f;
	iaqAccuracy = 0;
	staticIaq = 0.0f;
	staticIaqAccuracy = 0;
	co2Equivalent = 0.0f;
	co2Accuracy = 0;
	breathVocEquivalent = 0.0f;
	breathVocAccuracy = 0;
	compGasValue = 0.0f;
	compGasAccuracy = 0;
	gasPercentage = 0.0f;
	gasPercentageAccuracy = 0;
}

/**
 * @brief Function to zero the outputs
 */
void BSEC::zeroInputs(void)
{
    nextCall = 0;
    m_bme68x_status = BME68X_OK;
    outputTimestamp = 0;
   	m_bsec_status = BSEC_OK;

}

