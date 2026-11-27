// SPDX-License-Identifier: GPL-2.0-only
#include <linux/dma-mapping.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/soc/qcom/qmi.h>

#include "sunfish_heap_mem_ext_v01.h"
#include "sunfish_heap_mem_ext_v01.c"

#define SUNFISH_MEMSHARE_SVC		0x34
#define SUNFISH_MEMSHARE_VERS		1
#define SUNFISH_MEMSHARE_INST		1
#define SUNFISH_MEMSHARE_CLIENT_DIAG	1
#define SUNFISH_MEMSHARE_SIZE		0x500000
#define SUNFISH_MEMSHARE_MAX_MSG	4096

struct sunfish_memshare {
	struct qmi_handle qmi;
	struct platform_device *pdev;
	struct mutex lock;
	void *cookie;
	dma_addr_t dma;
	u64 src;
	bool allotted;
	u32 sequence_id;
};

static struct sunfish_memshare *g_ms;

static bool sunfish_memshare_ok(u32 client_id, u32 proc_id)
{
	return client_id == SUNFISH_MEMSHARE_CLIENT_DIAG &&
	       proc_id == DHMS_MEM_PROC_MPSS_V01;
}

static int sunfish_memshare_loan(struct sunfish_memshare *ms)
{
	struct qcom_scm_vmperm dest;
	unsigned long attrs = DMA_ATTR_NO_KERNEL_MAPPING;
	int ret;

	if (ms->allotted)
		return 0;

	if (!qcom_scm_is_available())
		return -EPROBE_DEFER;

	ms->cookie = dma_alloc_attrs(&ms->pdev->dev, SUNFISH_MEMSHARE_SIZE,
				     &ms->dma, GFP_KERNEL, attrs);
	if (!ms->cookie)
		return -ENOMEM;

	ms->src = BIT(QCOM_SCM_VMID_HLOS);
	dest.vmid = QCOM_SCM_VMID_MSS_MSA;
	dest.perm = QCOM_SCM_PERM_RW;
	ret = qcom_scm_assign_mem((phys_addr_t)ms->dma, SUNFISH_MEMSHARE_SIZE,
				  &ms->src, &dest, 1);
	if (ret) {
		dma_free_attrs(&ms->pdev->dev, SUNFISH_MEMSHARE_SIZE,
			       ms->cookie, ms->dma, attrs);
		ms->cookie = NULL;
		ms->dma = 0;
		return ret;
	}

	ms->allotted = true;
	return 0;
}

static void handle_query_size_req(struct qmi_handle *qmi,
				  struct sockaddr_qrtr *sq,
				  struct qmi_txn *txn, const void *decoded)
{
	const struct mem_query_size_req_msg_v01 *req = decoded;
	struct mem_query_size_rsp_msg_v01 *resp;
	u32 proc;
	int rc;

	resp = kzalloc(sizeof(*resp), GFP_KERNEL);
	if (!resp)
		return;

	proc = req->proc_id_valid ? req->proc_id : DHMS_MEM_PROC_MPSS_V01;
	if (sunfish_memshare_ok(req->client_id, proc)) {
		resp->size_valid = 1;
		resp->size = SUNFISH_MEMSHARE_SIZE;
		resp->resp.result = QMI_RESULT_SUCCESS_V01;
		resp->resp.error = QMI_ERR_NONE_V01;
	} else {
		resp->resp.result = QMI_RESULT_FAILURE_V01;
		resp->resp.error = QMI_ERR_INVALID_ID_V01;
	}

	pr_info("sunfish-memshare: query client=%d proc=%d size=0x%x result=%u\n",
		req->client_id, proc, resp->size, resp->resp.result);

	rc = qmi_send_response(qmi, sq, txn, MEM_QUERY_SIZE_RESP_MSG_V01,
			       MEM_QUERY_MAX_MSG_LEN_V01,
			       mem_query_size_resp_msg_data_v01_ei, resp);
	if (rc < 0)
		pr_err("sunfish-memshare: query resp %d\n", rc);
	kfree(resp);
}

static void handle_alloc_generic_req(struct qmi_handle *qmi,
				     struct sockaddr_qrtr *sq,
				     struct qmi_txn *txn, const void *decoded)
{
	const struct mem_alloc_generic_req_msg_v01 *req = decoded;
	struct mem_alloc_generic_resp_msg_v01 *resp;
	struct sunfish_memshare *ms = g_ms;
	int rc, ret;

	resp = kzalloc(sizeof(*resp), GFP_KERNEL);
	if (!resp)
		return;

	resp->resp.result = QMI_RESULT_FAILURE_V01;
	resp->resp.error = QMI_ERR_NO_MEMORY_V01;

	mutex_lock(&ms->lock);
	if (!sunfish_memshare_ok(req->client_id, req->proc_id)) {
		resp->resp.error = QMI_ERR_INVALID_ID_V01;
		goto send;
	}

	ret = sunfish_memshare_loan(ms);
	ms->sequence_id = req->sequence_id;
	resp->sequence_id_valid = 1;
	resp->sequence_id = req->sequence_id;
	if (ret) {
		pr_err("sunfish-memshare: alloc failed %d client=%d proc=%d req=%u\n",
		       ret, req->client_id, req->proc_id, req->num_bytes);
		goto send;
	}

	resp->dhms_mem_alloc_addr_info_valid = 1;
	resp->dhms_mem_alloc_addr_info_len = 1;
	resp->dhms_mem_alloc_addr_info[0].phy_addr = (u64)ms->dma;
	resp->dhms_mem_alloc_addr_info[0].num_bytes = SUNFISH_MEMSHARE_SIZE;
	resp->resp.result = QMI_RESULT_SUCCESS_V01;
	resp->resp.error = QMI_ERR_NONE_V01;
	pr_info("sunfish-memshare: alloc client=%d proc=%d req=%u pa=%pap\n",
		req->client_id, req->proc_id, req->num_bytes, &ms->dma);

send:
	mutex_unlock(&ms->lock);
	rc = qmi_send_response(qmi, sq, txn, MEM_ALLOC_GENERIC_RESP_MSG_V01,
			       sizeof(*resp),
			       mem_alloc_generic_resp_msg_data_v01_ei, resp);
	if (rc < 0)
		pr_err("sunfish-memshare: alloc resp %d\n", rc);
	kfree(resp);
}

static void handle_free_generic_req(struct qmi_handle *qmi,
				    struct sockaddr_qrtr *sq,
				    struct qmi_txn *txn, const void *decoded)
{
	const struct mem_free_generic_req_msg_v01 *req = decoded;
	struct mem_free_generic_resp_msg_v01 *resp;
	int rc;

	resp = kzalloc(sizeof(*resp), GFP_KERNEL);
	if (!resp)
		return;

	resp->resp.result = QMI_RESULT_FAILURE_V01;
	resp->resp.error = QMI_ERR_NOT_SUPPORTED_V01;
	pr_info("sunfish-memshare: free rejected client=%d proc=%d\n",
		req->client_id_valid ? req->client_id : -1,
		req->proc_id_valid ? req->proc_id : -1);

	rc = qmi_send_response(qmi, sq, txn, MEM_FREE_GENERIC_RESP_MSG_V01,
			       MEM_FREE_REQ_MAX_MSG_LEN_V01,
			       mem_free_generic_resp_msg_data_v01_ei, resp);
	if (rc < 0)
		pr_err("sunfish-memshare: free resp %d\n", rc);
	kfree(resp);
}

static const struct qmi_msg_handler sunfish_memshare_handlers[] = {
	{
		.type = QMI_REQUEST,
		.msg_id = MEM_ALLOC_GENERIC_REQ_MSG_V01,
		.ei = mem_alloc_generic_req_msg_data_v01_ei,
		.decoded_size = sizeof(struct mem_alloc_generic_req_msg_v01),
		.fn = handle_alloc_generic_req,
	},
	{
		.type = QMI_REQUEST,
		.msg_id = MEM_FREE_GENERIC_REQ_MSG_V01,
		.ei = mem_free_generic_req_msg_data_v01_ei,
		.decoded_size = sizeof(struct mem_free_generic_req_msg_v01),
		.fn = handle_free_generic_req,
	},
	{
		.type = QMI_REQUEST,
		.msg_id = MEM_QUERY_SIZE_REQ_MSG_V01,
		.ei = mem_query_size_req_msg_data_v01_ei,
		.decoded_size = sizeof(struct mem_query_size_req_msg_v01),
		.fn = handle_query_size_req,
	},
	{ },
};

static int __init sunfish_memshare_init(void)
{
	struct sunfish_memshare *ms;
	int ret;

	ms = kzalloc(sizeof(*ms), GFP_KERNEL);
	if (!ms)
		return -ENOMEM;
	mutex_init(&ms->lock);

	ms->pdev = platform_device_alloc("sunfish_memshare", PLATFORM_DEVID_NONE);
	if (!ms->pdev) {
		kfree(ms);
		return -ENOMEM;
	}
	ret = dma_coerce_mask_and_coherent(&ms->pdev->dev, DMA_BIT_MASK(64));
	if (ret)
		goto err_pdev;
	ret = platform_device_add(ms->pdev);
	if (ret)
		goto err_pdev;

	g_ms = ms;
	ret = qmi_handle_init(&ms->qmi, SUNFISH_MEMSHARE_MAX_MSG, NULL,
			      sunfish_memshare_handlers);
	if (ret)
		goto err_add;
	ret = qmi_add_server(&ms->qmi, SUNFISH_MEMSHARE_SVC,
			     SUNFISH_MEMSHARE_VERS, SUNFISH_MEMSHARE_INST);
	if (ret)
		goto err_qmi;

	pr_info("sunfish-memshare: QMI 0x34 v1 inst1 DIAG 5MiB\n");
	return 0;

err_qmi:
	qmi_handle_release(&ms->qmi);
err_add:
	g_ms = NULL;
	platform_device_unregister(ms->pdev);
	kfree(ms);
	return ret;
err_pdev:
	platform_device_put(ms->pdev);
	kfree(ms);
	return ret;
}

static void __exit sunfish_memshare_exit(void)
{
	struct sunfish_memshare *ms = g_ms;
	struct qcom_scm_vmperm dest;

	if (!ms)
		return;

	qmi_handle_release(&ms->qmi);
	if (ms->allotted) {
		dest.vmid = QCOM_SCM_VMID_HLOS;
		dest.perm = QCOM_SCM_PERM_RW;
		qcom_scm_assign_mem((phys_addr_t)ms->dma, SUNFISH_MEMSHARE_SIZE,
				    &ms->src, &dest, 1);
		dma_free_attrs(&ms->pdev->dev, SUNFISH_MEMSHARE_SIZE,
			       ms->cookie, ms->dma, DMA_ATTR_NO_KERNEL_MAPPING);
	}
	platform_device_unregister(ms->pdev);
	g_ms = NULL;
	kfree(ms);
}

module_init(sunfish_memshare_init);
module_exit(sunfish_memshare_exit);

MODULE_DESCRIPTION("sunfish memshare QMI server (Lead 3l Stage B)");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("pixel-4a bring-up");
