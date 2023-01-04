//
// readline.cpp
// Copyright (C) 2022 Marius Greuel
// SPDX-License-Identifier: GPL-2.0-or-later
//

#include <string.h>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include "readline/readline.h"
#include "readline/history.h"

int rl_readline_version = 0x0502;

static rl_vcpfunc_t* rl_handler;
static std::unique_ptr<std::thread> rl_thread;
static std::mutex rl_mutex;
static std::string rl_line;
static bool rl_has_line = false;

static void get_line_thread()
{
    std::string line;
    std::getline(std::cin, line);

    const std::lock_guard<std::mutex> lock(rl_mutex);
    rl_line = line;
    rl_has_line = true;
}

static void call_handler(const char* string)
{
    if (rl_thread)
    {
        rl_thread->join();
        rl_thread = nullptr;
    }

    if (rl_handler != nullptr)
    {
        if (string == nullptr)
        {
            rl_handler(nullptr);
        }
        else
        {
            rl_handler(_strdup(string));
        }
    }
}

int rl_input_available(void)
{
    return 1;
}

void rl_callback_read_char(void)
{
    if (std::cin.eof())
    {
        call_handler(nullptr);
    }
    else if (!rl_thread)
    {
        rl_thread = std::make_unique<std::thread>(get_line_thread);
    }
    else
    {
        const std::lock_guard<std::mutex> lock(rl_mutex);
        if (rl_has_line)
        {
            rl_has_line = false;
            call_handler(rl_line.c_str());
        }
    }
}

void rl_callback_handler_install(char* prompt, rl_vcpfunc_t* handler)
{
    rl_handler = handler;

    std::cout << prompt;
}

void rl_callback_handler_remove(void)
{
    rl_handler = nullptr;
}

void add_history(const char*)
{
}
