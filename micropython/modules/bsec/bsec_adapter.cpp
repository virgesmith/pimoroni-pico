#include "bsec.h"
// #include "drivers/bme68x/bme68x.hpp"
#include "micropython/modules/util.hpp"

extern "C"
{
#include "bsec_adapter.h"
#include "pimoroni_i2c.h"

// #define BSEC_CFG_18V_3S_4D
#define BSEC_CFG_33V_3S_4D
#include "bsec_config.inl"

using namespace pimoroni;

typedef struct _bsec_BSEC_obj_t
{
  mp_obj_base_t base;
  BSEC *breakout;
  _PimoroniI2C_obj_t *i2c;
} bsec_BSEC_obj_t;

mp_obj_t BSEC_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args)
{
  bsec_BSEC_obj_t *self = nullptr;

  enum
  {
    ARG_i2c,
    ARG_address,
    ARG_int
  };
  static const mp_arg_t allowed_args[] = {
      {MP_QSTR_i2c, MP_ARG_OBJ, {.u_obj = nullptr}},
      {MP_QSTR_address, MP_ARG_INT, {.u_int = BME68X::DEFAULT_I2C_ADDRESS}},
      //{ MP_QSTR_interrupt, MP_ARG_INT, {.u_int = PIN_UNUSED} },
  };

  // Parse args.
  mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
  mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

  self = m_new_obj(bsec_BSEC_obj_t);
  self->base.type = &bsec_BSEC_type;

  self->i2c = PimoroniI2C_from_machine_i2c_or_native(args[ARG_i2c].u_obj);

  self->breakout = m_new_class(BSEC, (pimoroni::I2C *)(self->i2c->i2c), args[ARG_address].u_int);

  // TODO mp_print_str(print?, ...)
  self->breakout->setConfig(bsec_config);

  sleep_ms(1000);

  bsec_virtual_sensor_t sensorList[] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_GAS_PERCENTAGE
  };

  self->breakout->updateSubscription(sensorList, sizeof(sensorList)/sizeof(sensorList[0]), BSEC_SAMPLE_RATE_LP);

  // if (self->breakout->bsec_status() != BSEC_OK)
  // {
  //   mp_raise_msg(&mp_type_RuntimeError, "BSEC: library initialisation error");
  // }
  // printf("BSEC library version %s\n", bsec.version().c_str());
  // printf("bsec_init: %d bme68x_init: %d\n", bsec.bsec_status(), bsec.bme68x_status());

  return MP_OBJ_FROM_PTR(self);
}

  mp_obj_t BSEC_status(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
  {
    enum
    {
      ARG_self,
    };
    static const mp_arg_t allowed_args[] = {
      {MP_QSTR_, MP_ARG_REQUIRED | MP_ARG_OBJ},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    bsec_BSEC_obj_t* self = MP_OBJ_TO_PTR2(args[ARG_self].u_obj, bsec_BSEC_obj_t);

    mp_obj_t tuple[2];
    tuple[0] = mp_obj_new_int(self->breakout->bsec_status());
    tuple[1] = mp_obj_new_int(self->breakout->bme68x_status());
    return mp_obj_new_tuple(2, tuple);
  }

  mp_obj_t BSEC_read(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
  {
    enum
    {
      ARG_self,
    };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_, MP_ARG_REQUIRED | MP_ARG_OBJ},
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    bsec_BSEC_obj_t* self = MP_OBJ_TO_PTR2(args[ARG_self].u_obj, bsec_BSEC_obj_t);

    for(;;) {
      if (self->breakout->run()) {
        mp_obj_t tuple[14];
        tuple[0] = mp_obj_new_float(self->breakout->rawTemperature);
        tuple[1] = mp_obj_new_float(self->breakout->pressure);
        tuple[2] = mp_obj_new_float(self->breakout->rawHumidity);
        tuple[3] = mp_obj_new_float(self->breakout->gasResistance);
        tuple[4] = mp_obj_new_int(self->breakout->stabStatus);
        tuple[5] = mp_obj_new_int(self->breakout->runInStatus);
        tuple[6] = mp_obj_new_float(self->breakout->iaq);
        tuple[7] = mp_obj_new_int(self->breakout->iaqAccuracy);
        tuple[8] = mp_obj_new_float(self->breakout->staticIaq);
        tuple[9] = mp_obj_new_float(self->breakout->co2Equivalent);
        tuple[10] = mp_obj_new_float(self->breakout->breathVocEquivalent);
        tuple[11] = mp_obj_new_float(self->breakout->temperature);
        tuple[12] = mp_obj_new_float(self->breakout->humidity);
        tuple[13] = mp_obj_new_float(self->breakout->gasPercentage);
        return mp_obj_new_tuple(16, tuple);
      }
      sleep_ms(3000);
    }
    // else
    // {
    //   mp_raise_msg(&mp_type_RuntimeError, "BSEC: failed read_forced");
    //   return mp_const_none;
    // }

    // bme68x_data result;
    // if(self->breakout->sensor().read_forced(&result, args[ARG_temp].u_int, args[ARG_duration].u_int)){
    //     mp_obj_t tuple[7];
    //     tuple[0] = mp_obj_new_float(result.temperature);
    //     tuple[1] = mp_obj_new_float(result.pressure);
    //     tuple[2] = mp_obj_new_float(result.humidity);
    //     tuple[3] = mp_obj_new_float(result.gas_resistance);
    //     tuple[4] = mp_obj_new_int(result.status);
    //     tuple[5] = mp_obj_new_int(result.gas_index);
    //     tuple[6] = mp_obj_new_int(result.meas_index);
    //     return mp_obj_new_tuple(7, tuple);
    // }
    // else {
    //     mp_raise_msg(&mp_type_RuntimeError, "BSEC: failed read_forced");
    //     return mp_const_none;
    // }
  }

  mp_obj_t BSEC_configure(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
  {
    enum
    {
      ARG_self,
      ARG_filter,
      ARG_standby_time,
      ARG_os_pressure,
      ARG_os_temp,
      ARG_os_humidity
    };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_, MP_ARG_REQUIRED | MP_ARG_OBJ},
        {MP_QSTR_filter, MP_ARG_INT, {.u_int = BME68X_FILTER_SIZE_3}},
        {MP_QSTR_standby_time, MP_ARG_INT, {.u_int = BME68X_ODR_0_59_MS}},
        {MP_QSTR_os_pressure, MP_ARG_INT, {.u_int = BME68X_OS_16X}},
        {MP_QSTR_os_temp, MP_ARG_INT, {.u_int = BME68X_OS_2X}},
        {MP_QSTR_os_humidity, MP_ARG_INT, {.u_int = BME68X_OS_1X}}};

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    bsec_BSEC_obj_t *self = MP_OBJ_TO_PTR2(args[ARG_self].u_obj, bsec_BSEC_obj_t);
    self->breakout->sensor().configure(
        args[ARG_filter].u_int,
        args[ARG_standby_time].u_int,
        args[ARG_os_humidity].u_int,
        args[ARG_os_pressure].u_int,
        args[ARG_os_temp].u_int);

    return mp_const_none;
  }
}