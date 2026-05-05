#pragma once

/**
 * @brief Starts the application tasks.
 * This function creates the control task and the blink task, which are responsible for handling the main application logic and providing visual feedback, respectively. 
 * The control task is created with a stack size of 16384 bytes and a priority of 6, while the blink task is created with a stack size of 1024 bytes and a priority of 1.
 */
void start_app(void);