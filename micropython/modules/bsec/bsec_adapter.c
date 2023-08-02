#include "bsec_adapter.h"

// BSEC class
MP_DEFINE_CONST_FUN_OBJ_KW(BSEC_status_obj, 1, BSEC_status);
MP_DEFINE_CONST_FUN_OBJ_KW(BSEC_get_state_obj, 1, BSEC_get_state);
MP_DEFINE_CONST_FUN_OBJ_KW(BSEC_set_state_obj, 1, BSEC_set_state);
MP_DEFINE_CONST_FUN_OBJ_KW(BSEC_read_obj, 1, BSEC_read);

STATIC const mp_rom_map_elem_t BSEC_locals_dict_table[] = {
  { MP_ROM_QSTR(MP_QSTR_status), MP_ROM_PTR(&BSEC_status_obj) },
  { MP_ROM_QSTR(MP_QSTR_get_state), MP_ROM_PTR(&BSEC_get_state_obj) },
  { MP_ROM_QSTR(MP_QSTR_set_state), MP_ROM_PTR(&BSEC_set_state_obj) },
  { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&BSEC_read_obj) },
};
STATIC MP_DEFINE_CONST_DICT(BSEC_locals_dict, BSEC_locals_dict_table);


#ifdef MP_DEFINE_CONST_OBJ_TYPE
MP_DEFINE_CONST_OBJ_TYPE(
    bsec_BSEC_type,
    MP_QSTR_BSEC,
    MP_TYPE_FLAG_NONE,
    make_new, BSEC_make_new,
    locals_dict, (mp_obj_dict_t*)&BSEC_locals_dict
);
#else
const mp_obj_type_t bsec_BSEC_type = {
    { &mp_type_type },
    .name = MP_QSTR_BSEC,
    .make_new = BSEC_make_new,
    .locals_dict = (mp_obj_dict_t*)&BSEC_locals_dict,
};
#endif

// bsec module

STATIC MP_DEFINE_CONST_FUN_OBJ_0(bsec_version_obj, bsec_version);

STATIC const mp_map_elem_t bsec_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_bsec) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_version), (mp_obj_t)(&bsec_version_obj)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_BSEC), (mp_obj_t)&bsec_BSEC_type },

};
STATIC MP_DEFINE_CONST_DICT(mp_module_bsec_globals, bsec_globals_table);

// module
const mp_obj_module_t bsec_user_cmodule = {
    .base = { &mp_type_module },
    //.attr = bsec_attr ???
    .globals = (mp_obj_dict_t*)&mp_module_bsec_globals,
};

#if MICROPY_VERSION <= 70144
MP_REGISTER_MODULE(MP_QSTR_bsec, bsec_user_cmodule, MODULE_BSEC_ENABLED);
#else
MP_REGISTER_MODULE(MP_QSTR_bsec, bsec_user_cmodule);
#endif
