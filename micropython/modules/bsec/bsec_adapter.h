#pragma once

#include "py/runtime.h"
#include "drivers/bme68x/src/bme68x_defs.h"

/* Constants */

/* Extern of Class Definition */
extern const mp_obj_type_t bsec_BSEC_type;

/* Extern of Class Methods */
extern mp_obj_t BSEC_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args);
extern mp_obj_t BSEC_status(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args);
extern mp_obj_t BSEC_read(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args);
extern mp_obj_t BSEC_get_state(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args);
extern mp_obj_t BSEC_set_state(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args);

