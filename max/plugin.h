// Exodus for 3ds Max: what the plugin's parts share.
#pragma once

#include <max.h>
#include <iparamb2.h>

#include "maxcore_api.h"

extern HINSTANCE hInstance;

const MCHAR* exodus_string(int id);  // a string from exodus.rc

// The loaded core DLL, loading it on first use; nullptr when it cannot be loaded.
const ExodusCoreApi* exodus_core();
const ExodusCoreApi* exodus_core_loaded();  // the core DLL if it is loaded, without loading it
MSTR exodus_core_error();

ClassDesc2* unsubdivide_desc();
ClassDesc2* decimate_desc();
void reduce_refresh_all();  // after the core DLL was reloaded: recompute every reducing modifier

ClassDesc2* pack_desc();
void pack_refresh_all();                  // after the core DLL was reloaded: lay out again
MSTR pack_bake(ReferenceTarget* target);  // Bake of a UDIM Pack modifier; returns its report
MSTR pack_preview(ReferenceTarget* target);  // Preview of a UDIM Pack modifier (opens its window); returns its report
MSTR pack_detect(ReferenceTarget* target);   // Auto detect of a UDIM Pack modifier (maps from materials); returns its report
