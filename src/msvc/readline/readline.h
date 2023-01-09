//
// readline.h
// Copyright (C) 2022 Marius Greuel
// SPDX-License-Identifier: GPL-2.0-or-later
//

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef void (rl_vcpfunc_t)(char* line);

extern int rl_readline_version;

int rl_input_available(void);
void rl_callback_read_char(void);
void rl_callback_handler_install(char* prompt, rl_vcpfunc_t* handler);
void rl_callback_handler_remove(void);

#ifdef __cplusplus
}
#endif
