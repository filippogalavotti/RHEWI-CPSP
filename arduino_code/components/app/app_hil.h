#pragma once
#include "app_context.h"

/**
 * @brief Initialize the application in HIL mode by setting up the necessary data structures and communication interfaces for receiving commands and sending data.
 */
void app_hil_init(void);

/**
 * @brief Handle the HIL mode operations for the application.
 * @param ctx Pointer to the application context structure.
 */
void app_hil_loop(app_context_t *ctx);