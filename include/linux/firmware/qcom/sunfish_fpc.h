/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _QCOM_SUNFISH_FPC_H
#define _QCOM_SUNFISH_FPC_H
#include <linux/types.h>
/* Lab-only legacy QSEE transport. Callers retain buffers on any error. */
int qcom_scm_fpc_load(phys_addr_t image, u32 mdt_len, u32 image_len,
                      u32 *app_id);
int qcom_scm_fpc_send(u32 app_id, phys_addr_t request,
                      phys_addr_t response, phys_addr_t whitelist);
int qcom_scm_fpc_key_request(u32 app_id, phys_addr_t request, phys_addr_t response, u32 req_size, u32 rsp_size);
int qcom_scm_fpc_log(phys_addr_t buffer, u32 size);
int qcom_scm_fpc_unload(u32 app_id);
void qcom_scm_fpc_listener_set(int (*callback)(void *, u32), void *context);
int qcom_scm_fpc_listener_register(u32 id, phys_addr_t buffer, u32 size);
#endif
