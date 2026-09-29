// udimpacker for 3ds Max: what the plugin's parts share.
#pragma once

#include <max.h>
#include <iparamb2.h>

#include "maxcore_api.h"

extern HINSTANCE hInstance;

const MCHAR* udim_string(int id);  // a string from udimpacker.rc

// The loaded core DLL, loading it on first use; nullptr when it cannot be loaded.
const UdimCoreApi* udim_core();
MSTR udim_core_error();

ClassDesc2* unsubdivide_desc();
ClassDesc2* decimate_desc();
void reduce_refresh_all();  // after the core DLL was reloaded: recompute every reducing modifier
