#pragma once
#ifndef IMPORT
#define IMPORT __declspec(dllexport)
#endif
#include "../SDK/sdk.h"

void loadAPI(const char * dll_name);
void unloadAPI();
void reloadAPI();
