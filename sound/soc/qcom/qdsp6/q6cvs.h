/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _Q6_CVS_H
#define _Q6_CVS_H

#include "q6voice.h"

struct q6voice_session;

struct q6voice_session *q6cvs_session_create(enum q6voice_path_type path);
int q6cvs_unmute(struct q6voice_session *cvs);
int q6cvs_set_rx_dtmf_detection(struct q6voice_session *cvs, bool enable);

#endif /*_Q6_CVS_H */
