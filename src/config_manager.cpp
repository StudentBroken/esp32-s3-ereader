#include "config_manager.h"

Preferences ConfigManager::prefs;
Preferences ConfigManager::progress_prefs;
ReaderConfig ConfigManager::config;
bool ConfigManager::dirty = false;
uint32_t ConfigManager::dirty_time = 0;
