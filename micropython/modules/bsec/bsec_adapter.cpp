#include "bsec.h"
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
  BSEC *object;
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

  self->object = m_new_class(BSEC, (pimoroni::I2C *)(self->i2c->i2c), args[ARG_address].u_int);

  self->object->setConfig(bsec_config);

  // TODO check

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

  self->object->updateSubscription(sensorList, sizeof(sensorList)/sizeof(sensorList[0]), BSEC_SAMPLE_RATE_LP);

  if (self->object->bsec_status() != BSEC_OK)
  {
    mp_raise_msg(&mp_type_RuntimeError, "BSEC: library initialisation error");
  }

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
  tuple[0] = mp_obj_new_int(self->object->bsec_status());
  tuple[1] = mp_obj_new_int(self->object->bme68x_status());
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
    if (self->object->run()) {
      mp_obj_t tuple[14];
      tuple[0] = mp_obj_new_float(self->object->rawTemperature);
      tuple[1] = mp_obj_new_float(self->object->pressure);
      tuple[2] = mp_obj_new_float(self->object->rawHumidity);
      tuple[3] = mp_obj_new_float(self->object->gasResistance);
      tuple[4] = mp_obj_new_int(self->object->stabStatus);
      tuple[5] = mp_obj_new_int(self->object->runInStatus);
      tuple[6] = mp_obj_new_float(self->object->iaq);
      tuple[7] = mp_obj_new_int(self->object->iaqAccuracy);
      tuple[8] = mp_obj_new_float(self->object->staticIaq);
      tuple[9] = mp_obj_new_float(self->object->co2Equivalent);
      tuple[10] = mp_obj_new_float(self->object->breathVocEquivalent);
      tuple[11] = mp_obj_new_float(self->object->temperature);
      tuple[12] = mp_obj_new_float(self->object->humidity);
      tuple[13] = mp_obj_new_float(self->object->gasPercentage);
      return mp_obj_new_tuple(16, tuple);
    }
    sleep_ms(3000);
  }
}


mp_obj_t BSEC_set_state(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
  enum
  {
    ARG_self,
    ARG_state_vector,
  };
  static const mp_arg_t allowed_args[] = {
    {MP_QSTR_, MP_ARG_REQUIRED | MP_ARG_OBJ},
    {MP_QSTR_state_vector, MP_ARG_REQUIRED | MP_ARG_OBJ}
  };

  mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
  mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

  bsec_BSEC_obj_t* self = MP_OBJ_TO_PTR2(args[ARG_self].u_obj, bsec_BSEC_obj_t);

  mp_buffer_info_t bufinfo;
  mp_get_buffer_raise(args[ARG_state_vector].u_obj, &bufinfo, MP_BUFFER_READ);
  if(bufinfo.len != (size_t)(BSEC_MAX_STATE_BLOB_SIZE)) {
    mp_raise_ValueError("BSEC state vector buffer size should be 221 bytes");
  }
  uint8_t* state = (uint8_t*)bufinfo.buf;

  self->object->setState(state);

  return mp_const_none;
}


mp_obj_t BSEC_get_state(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
  enum
  {
    ARG_self,
  };
  static const mp_arg_t allowed_args[] = {{MP_QSTR_, MP_ARG_REQUIRED | MP_ARG_OBJ}};

  mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
  mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

  bsec_BSEC_obj_t* self = MP_OBJ_TO_PTR2(args[ARG_self].u_obj, bsec_BSEC_obj_t);

  uint8_t state[BSEC_MAX_STATE_BLOB_SIZE] = {0};

  self->object->getState(state);

  return mp_obj_new_bytes(state, BSEC_MAX_STATE_BLOB_SIZE);
}

} // extern "C"