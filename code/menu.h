/*
 * menu.h
 */

#ifndef CODE_MENU_H_
#define CODE_MENU_H_

#include "zf_common_headfile.h"

void menu_init(void);
void menu_process(void);

/* Compatibility with old debug snippets. Shows the profile page now. */
void menu_show_pid(void);

#endif /* CODE_MENU_H_ */