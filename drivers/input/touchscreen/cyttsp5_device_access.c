/*
 * cyttsp5_device_access.c
 * Cypress TrueTouch(TM) Standard Product V5 Device Access Module.
 * Configuration and Test command/status user interface.
 * For use with Cypress touchscreen controllers.
 * Supported parts include:
 * CYTMA5XX
 * CYTMA448
 * CYTMA445A
 * CYTT21XXX
 * CYTT31XXX
 *
 * Copyright (C) 2012-2015 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include "cyttsp5_regs.h"

#define CYTTSP5_DEVICE_ACCESS_NAME "cyttsp5_device_access"
#define CYTTSP5_INPUT_ELEM_SZ (sizeof("0xHH") + 1)

#define STATUS_SUCCESS	0
#define STATUS_FAIL	-1
#define PIP_CMD_MAX_LENGTH ((1 << 16) - 1)

#ifdef TTHE_TUNER_SUPPORT
struct heatmap_param {
	bool scan_start;
	enum scan_data_type_list data_type; /* raw, base, diff */
	int num_element;
};
#endif

#define CY_MAX_CONFIG_BYTES    256
#define CYTTSP5_TTHE_TUNER_GET_PANEL_DATA_FILE_NAME "get_panel_data"
#define TTHE_TUNER_MAX_BUF	(CY_MAX_PRBUF_SIZE * 3)

struct cyttsp5_device_access_data {
	struct device *dev;
	struct cyttsp5_sysinfo *si;
	struct mutex sysfs_lock;
	u8 status;
	u16 response_length;
	bool sysfs_nodes_created;
	struct kobject mfg_test;
	u8 panel_scan_data_id;
	u8 get_idac_data_id;
	u8 calibrate_sensing_mode;
	u8 calibrate_initialize_baselines;
	u8 baseline_sensing_mode;
#ifdef TTHE_TUNER_SUPPORT
	struct heatmap_param heatmap;
	struct dentry *tthe_get_panel_data_debugfs;
	struct mutex debugfs_lock;
	u8 tthe_get_panel_data_buf[TTHE_TUNER_MAX_BUF];
	u8 tthe_get_panel_data_is_open;
#endif
	struct dentry *base_dentry;
	struct dentry *mfg_test_dentry;
	u8 ic_buf[CY_MAX_PRBUF_SIZE];
	u8 response_buf[CY_MAX_PRBUF_SIZE];
};

static struct cyttsp5_core_commands *cmd;

static struct cyttsp5_module device_access_module;

static inline struct cyttsp5_device_access_data *cyttsp5_get_device_access_data(
		struct device *dev)
{
	return cyttsp5_get_module_data(dev, &device_access_module);
}

static ssize_t cyttsp5_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	u8 val;

	mutex_lock(&dad->sysfs_lock);
	val = dad->status;
	mutex_unlock(&dad->sysfs_lock);

	return scnprintf(buf, CY_MAX_PRBUF_SIZE, "%d\n", val);
}

static DEVICE_ATTR(status, S_IRUSR, cyttsp5_status_show, NULL);

static ssize_t cyttsp5_response_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	int i;
	ssize_t num_read;
	int index;

	mutex_lock(&dad->sysfs_lock);
	index = scnprintf(buf, CY_MAX_PRBUF_SIZE,
			"Status %d\n", dad->status);
	if (!dad->status)
		goto error;

	num_read = dad->response_length;

	for (i = 0; i < num_read; i++)
		index += scnprintf(buf + index, CY_MAX_PRBUF_SIZE - index,
				"0x%02X\n", dad->response_buf[i]);

	index += scnprintf(buf + index, CY_MAX_PRBUF_SIZE - index,
			"(%zd bytes)\n", num_read);

error:
	mutex_unlock(&dad->sysfs_lock);
	return index;
}

static DEVICE_ATTR(response, S_IRUSR, cyttsp5_response_show, NULL);

/*
 * Gets user input from sysfs and parse it
 * return size of parsed output buffer
 */
static int cyttsp5_ic_parse_input(struct device *dev, const char *buf,
		size_t buf_size, u8 *ic_buf, size_t ic_buf_size)
{
	const char *pbuf = buf;
	unsigned long value;
	char scan_buf[CYTTSP5_INPUT_ELEM_SZ];
	u32 i = 0;
	u32 j;
	int last = 0;
	int ret;

	dev_dbg(dev, "%s: pbuf=%p buf=%p size=%zu %s=%zu buf=%s\n", __func__,
			pbuf, buf, buf_size, "scan buf size",
			CYTTSP5_INPUT_ELEM_SZ, buf);

	while (pbuf <= (buf + buf_size)) {
		if (i >= CY_MAX_CONFIG_BYTES) {
			dev_err(dev, "%s: %s size=%d max=%d\n", __func__,
					"Max cmd size exceeded", i,
					CY_MAX_CONFIG_BYTES);
			return -EINVAL;
		}
		if (i >= ic_buf_size) {
			dev_err(dev, "%s: %s size=%d buf_size=%zu\n", __func__,
					"Buffer size exceeded", i, ic_buf_size);
			return -EINVAL;
		}
		while (((*pbuf == ' ') || (*pbuf == ','))
				&& (pbuf < (buf + buf_size))) {
			last = *pbuf;
			pbuf++;
		}

		if (pbuf >= (buf + buf_size))
			break;

		memset(scan_buf, 0, CYTTSP5_INPUT_ELEM_SZ);
		if ((last == ',') && (*pbuf == ',')) {
			dev_err(dev, "%s: %s \",,\" not allowed.\n", __func__,
					"Invalid data format.");
			return -EINVAL;
		}
		for (j = 0; j < (CYTTSP5_INPUT_ELEM_SZ - 1)
				&& (pbuf < (buf + buf_size))
				&& (*pbuf != ' ')
				&& (*pbuf != ','); j++) {
			last = *pbuf;
			scan_buf[j] = *pbuf++;
		}

		ret = kstrtoul(scan_buf, 16, &value);
		if (ret < 0) {
			dev_err(dev, "%s: %s '%s' %s%s i=%d r=%d\n", __func__,
					"Invalid data format. ", scan_buf,
					"Use \"0xHH,...,0xHH\"", " instead.",
					i, ret);
			return ret;
		}

		ic_buf[i] = value;
		i++;
	}

	return i;
}

static ssize_t cyttsp5_command_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	ssize_t length;
	int rc;

	mutex_lock(&dad->sysfs_lock);
	dad->status = 0;
	dad->response_length = 0;
	length = cyttsp5_ic_parse_input(dev, buf, size, dad->ic_buf,
			CY_MAX_PRBUF_SIZE);
	if (length <= 0) {
		dev_err(dev, "%s: %s Group Data store\n", __func__,
				"Malformed input for");
		goto exit;
	}

	/* write ic_buf to log */
	cyttsp5_pr_buf(dev, dad->ic_buf, length, "ic_buf");

	pm_runtime_get_sync(dev);
	rc = cmd->nonhid_cmd->user_cmd(dev, 1, CY_MAX_PRBUF_SIZE,
			dad->response_buf, length, dad->ic_buf,
			&dad->response_length);
	pm_runtime_put(dev);
	if (rc) {
		dad->response_length = 0;
		dev_err(dev, "%s: Failed to store command\n", __func__);
	} else {
		dad->status = 1;
	}

exit:
	mutex_unlock(&dad->sysfs_lock);
	dev_vdbg(dev, "%s: return size=%zu\n", __func__, size);
	return size;
}

static DEVICE_ATTR(command, S_IWUSR, NULL, cyttsp5_command_store);

/*
 * Suspend scan command
 */
static int cyttsp5_suspend_scan_cmd_(struct device *dev)
{
	int rc;

	rc = cmd->nonhid_cmd->suspend_scanning(dev, 0);
	if (rc < 0)
		dev_err(dev, "%s: Suspend scan failed r=%d\n",
			__func__, rc);
	return rc;
}

/*
 * Resume scan command
 */
static int cyttsp5_resume_scan_cmd_(struct device *dev)
{
	int rc;

	rc = cmd->nonhid_cmd->resume_scanning(dev, 0);
	if (rc < 0)
		dev_err(dev, "%s: Resume scan failed r=%d\n",
			__func__, rc);
	return rc;
}

/*
 * Execute scan command
 */
static int cyttsp5_exec_scan_cmd_(struct device *dev)
{
	int rc;

	rc = cmd->nonhid_cmd->exec_panel_scan(dev, 0);
	if (rc < 0)
		dev_err(dev, "%s: Heatmap start scan failed r=%d\n",
			__func__, rc);
	return rc;
}

/*
 * Retrieve panel data command
 */
static int cyttsp5_ret_scan_data_cmd_(struct device *dev, u16 read_offset,
		u16 read_count, u8 data_id, u8 *response, u8 *config,
		u16 *actual_read_len, u8 *return_buf)
{
	int rc;

	rc = cmd->nonhid_cmd->retrieve_panel_scan(dev, 0, read_offset,
			read_count, data_id, response, config, actual_read_len,
			return_buf);
	if (rc < 0)
		dev_err(dev, "%s: Retrieve scan data failed r=%d\n",
				__func__, rc);
	return rc;
}

/*
 * Get data structure command
 */
static int cyttsp5_get_data_structure_cmd_(struct device *dev, u16 read_offset,
		u16 read_length, u8 data_id, u8 *status, u8 *data_format,
		u16 *actual_read_len, u8 *data)
{
	int rc;

	rc = cmd->nonhid_cmd->get_data_structure(dev, 0, read_offset,
			read_length, data_id, status, data_format,
			actual_read_len, data);
	if (rc < 0)
		dev_err(dev, "%s: Get data structure failed r=%d\n",
				__func__, rc);
	return rc;
}

/*
 * Run self test command
 */
static int cyttsp5_run_selftest_cmd_(struct device *dev, u8 test_id,
		u8 write_idacs_to_flash, u8 *status, u8 *summary_result,
		u8 *results_available)
{
	int rc;

	rc = cmd->nonhid_cmd->run_selftest(dev, 0, test_id,
			write_idacs_to_flash, status, summary_result,
			results_available);
	if (rc < 0)
		dev_err(dev, "%s: Run self test failed r=%d\n",
				__func__, rc);
	return rc;
}

/*
 * Get self test result command
 */
static int cyttsp5_get_selftest_result_cmd_(struct device *dev,
		u16 read_offset, u16 read_length, u8 test_id, u8 *status,
		u16 *actual_read_len, u8 *data)
{
	int rc;

	rc = cmd->nonhid_cmd->get_selftest_result(dev, 0, read_offset,
			read_length, test_id, status, actual_read_len, data);
	if (rc < 0)
		dev_err(dev, "%s: Get self test result failed r=%d\n",
				__func__, rc);
	return rc;
}

/*
 * Calibrate IDACs command
 */
static int _cyttsp5_calibrate_idacs_cmd(struct device *dev,
		u8 sensing_mode, u8 *status)
{
	int rc;

	rc = cmd->nonhid_cmd->calibrate_idacs(dev, 0, sensing_mode, status);
	return rc;
}

/*
 * Initialize Baselines command
 */
static int _cyttsp5_initialize_baselines_cmd(struct device *dev,
		u8 sensing_mode, u8 *status)
{
	int rc;

	rc = cmd->nonhid_cmd->initialize_baselines(dev, 0, sensing_mode,
			status);
	return rc;
}

static int prepare_print_buffer(int status, u8 *in_buf, int length,
		u8 *out_buf, size_t out_buf_size)
{
	int index = 0;
	int i;

	index += scnprintf(out_buf, out_buf_size, "status %d\n", status);

	for (i = 0; i < length; i++) {
		index += scnprintf(&out_buf[index], out_buf_size - index,
				"%02X\n", in_buf[i]);
	}

	return index;
}

static ssize_t cyttsp5_run_and_get_selftest_result(struct device *dev,
		char *buf, size_t buf_len, u8 test_id, u16 read_length,
		bool get_result_on_pass)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	int status = STATUS_FAIL;
	u8 cmd_status = 0;
	u8 summary_result = 0;
	u16 act_length = 0;
	int length = 0;
	int size;
	int rc;

	mutex_lock(&dad->sysfs_lock);

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		dev_err(dev, "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto put_pm_runtime;
	}

	rc = cyttsp5_suspend_scan_cmd_(dev);
	if (rc < 0) {
		dev_err(dev, "%s: Error on suspend scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	rc = cyttsp5_run_selftest_cmd_(dev, test_id, 0,
			&cmd_status, &summary_result, NULL);
	if (rc < 0) {
		dev_err(dev, "%s: Error on run self test for test_id:%d r=%d\n",
				__func__, test_id, rc);
		goto resume_scan;
	}

	/* Form response buffer */
	dad->ic_buf[0] = cmd_status;
	dad->ic_buf[1] = summary_result;

	length = 2;

	/* Get data if command status is success */
	if (cmd_status != CY_CMD_STATUS_SUCCESS)
		goto status_success;

	/* Get data unless test result is pass */
	if (summary_result == CY_ST_RESULT_PASS && !get_result_on_pass)
		goto status_success;

	rc = cyttsp5_get_selftest_result_cmd_(dev, 0, read_length,
			test_id, &cmd_status, &act_length, &dad->ic_buf[6]);
	if (rc < 0) {
		dev_err(dev, "%s: Error on get self test result r=%d\n",
				__func__, rc);
		goto resume_scan;
	}

	dad->ic_buf[2] = cmd_status;
	dad->ic_buf[3] = test_id;
	dad->ic_buf[4] = LOW_BYTE(act_length);
	dad->ic_buf[5] = HI_BYTE(act_length);

	length = 6 + act_length;

status_success:
	status = STATUS_SUCCESS;

resume_scan:
	cyttsp5_resume_scan_cmd_(dev);

release_exclusive:
	cmd->release_exclusive(dev);

put_pm_runtime:
	pm_runtime_put(dev);

	if (status == STATUS_FAIL)
		length = 0;

	size = prepare_print_buffer(status, dad->ic_buf, length, buf, buf_len);

	mutex_unlock(&dad->sysfs_lock);

	return size;
}

struct cyttsp5_device_access_debugfs_data {
	struct cyttsp5_device_access_data *dad;
	ssize_t pr_buf_len;
	u8 pr_buf[3 * CY_MAX_PRBUF_SIZE];
};

static int cyttsp5_device_access_debugfs_open(struct inode *inode,
		struct file *filp)
{
	struct cyttsp5_device_access_data *dad = inode->i_private;
	struct cyttsp5_device_access_debugfs_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dad = dad;

	filp->private_data = data;

	return nonseekable_open(inode, filp);
}

static int cyttsp5_device_access_debugfs_release(struct inode *inode,
		struct file *filp)
{
	kfree(filp->private_data);

	return 0;
}

#define CY_DEBUGFS_FOPS(_name, _read, _write) \
static const struct file_operations _name##_debugfs_fops = { \
	.open = cyttsp5_device_access_debugfs_open, \
	.release = cyttsp5_device_access_debugfs_release, \
	.read = _read, \
	.write = _write, \
}

static ssize_t panel_scan_debugfs_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_debugfs_data *data = filp->private_data;
	struct cyttsp5_device_access_data *dad = data->dad;
	struct device *dev = dad->dev;
	int status = STATUS_FAIL;
	u8 config;
	u16 actual_read_len;
	int length = 0;
	u8 element_size = 0;
	u8 *buf_offset;
	int elem_offset = 0;
	int rc;

	if (*ppos)
		goto exit;


	mutex_lock(&dad->sysfs_lock);

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		dev_err(dev, "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto put_pm_runtime;
	}

	rc = cyttsp5_suspend_scan_cmd_(dev);
	if (rc < 0) {
		dev_err(dev, "%s: Error on suspend scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	rc = cyttsp5_exec_scan_cmd_(dev);
	if (rc < 0) {
		dev_err(dev, "%s: Error on execute panel scan r=%d\n",
				__func__, rc);
		goto resume_scan;
	}

	/* Set length to max to read all */
	rc = cyttsp5_ret_scan_data_cmd_(dev, 0, 0xFFFF,
			dad->panel_scan_data_id, dad->ic_buf, &config,
			&actual_read_len, NULL);
	if (rc < 0) {
		dev_err(dev, "%s: Error on retrieve panel scan r=%d\n",
				__func__, rc);
		goto resume_scan;
	}

	length = get_unaligned_le16(&dad->ic_buf[0]);
	buf_offset = dad->ic_buf + length;
	element_size = config & 0x07;
	elem_offset = actual_read_len;
	while (actual_read_len > 0) {
		rc = cyttsp5_ret_scan_data_cmd_(dev, elem_offset, 0xFFFF,
				dad->panel_scan_data_id, NULL, &config,
				&actual_read_len, buf_offset);
		if (rc < 0)
			goto resume_scan;

		length += actual_read_len * element_size;
		buf_offset = dad->ic_buf + length;
		elem_offset += actual_read_len;
	}
	/* Reconstruct cmd header */
	put_unaligned_le16(length, &dad->ic_buf[0]);
	put_unaligned_le16(elem_offset, &dad->ic_buf[7]);

	/* Do not print command header */
	length -= 5;

	status = STATUS_SUCCESS;

resume_scan:
	cyttsp5_resume_scan_cmd_(dev);

release_exclusive:
	cmd->release_exclusive(dev);

put_pm_runtime:
	pm_runtime_put(dev);

	if (status == STATUS_FAIL)
		length = 0;

	data->pr_buf_len = prepare_print_buffer(status, &dad->ic_buf[5],
			length, data->pr_buf, sizeof(data->pr_buf));

	mutex_unlock(&dad->sysfs_lock);

exit:
	return simple_read_from_buffer(buf, count, ppos, data->pr_buf,
			data->pr_buf_len);
}

static ssize_t panel_scan_debugfs_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_debugfs_data *data = filp->private_data;
	struct cyttsp5_device_access_data *dad = data->dad;
	ssize_t length;
	int rc = 0;

	rc = simple_write_to_buffer(data->pr_buf, sizeof(data->pr_buf), ppos,
			buf, count);
	if (rc < 0)
		return rc;

	count = rc;

	mutex_lock(&dad->sysfs_lock);

	length = cyttsp5_ic_parse_input(dad->dev, data->pr_buf, count,
			dad->ic_buf, CY_MAX_PRBUF_SIZE);

	if (length != 1) {
		dev_err(dad->dev, "%s: Malformed input\n", __func__);
		rc = -EINVAL;
		goto exit_unlock;
	}

	dad->panel_scan_data_id = dad->ic_buf[0];

exit_unlock:
	mutex_unlock(&dad->sysfs_lock);

	if (rc)
		return rc;

	return count;
}

CY_DEBUGFS_FOPS(panel_scan, panel_scan_debugfs_read, panel_scan_debugfs_write);

static ssize_t get_idac_debugfs_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_debugfs_data *data = filp->private_data;
	struct cyttsp5_device_access_data *dad = data->dad;
	struct device *dev = dad->dev;
	int status = STATUS_FAIL;
	u8 cmd_status = 0;
	u8 data_format = 0;
	u16 act_length = 0;
	int length = 0;
	int rc;

	if (*ppos)
		goto exit;

	mutex_lock(&dad->sysfs_lock);

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		dev_err(dev, "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto put_pm_runtime;
	}

	rc = cyttsp5_suspend_scan_cmd_(dev);
	if (rc < 0) {
		dev_err(dev, "%s: Error on suspend scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	rc = cyttsp5_get_data_structure_cmd_(dev, 0, PIP_CMD_MAX_LENGTH,
			dad->get_idac_data_id, &cmd_status, &data_format,
			&act_length, &dad->ic_buf[5]);
	if (rc < 0) {
		dev_err(dev, "%s: Error on get data structure r=%d\n",
				__func__, rc);
		goto resume_scan;
	}

	dad->ic_buf[0] = cmd_status;
	dad->ic_buf[1] = dad->get_idac_data_id;
	dad->ic_buf[2] = LOW_BYTE(act_length);
	dad->ic_buf[3] = HI_BYTE(act_length);
	dad->ic_buf[4] = data_format;

	length = 5 + act_length;

	status = STATUS_SUCCESS;

resume_scan:
	cyttsp5_resume_scan_cmd_(dev);

release_exclusive:
	cmd->release_exclusive(dev);

put_pm_runtime:
	pm_runtime_put(dev);

	if (status == STATUS_FAIL)
		length = 0;

	data->pr_buf_len = prepare_print_buffer(status, dad->ic_buf, length,
			data->pr_buf, sizeof(data->pr_buf));

	mutex_unlock(&dad->sysfs_lock);

exit:
	return simple_read_from_buffer(buf, count, ppos, data->pr_buf,
			data->pr_buf_len);
}

static ssize_t get_idac_debugfs_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_debugfs_data *data = filp->private_data;
	struct cyttsp5_device_access_data *dad = data->dad;
	ssize_t length;
	int rc = 0;

	rc = simple_write_to_buffer(data->pr_buf, sizeof(data->pr_buf), ppos,
			buf, count);
	if (rc < 0)
		return rc;

	count = rc;

	mutex_lock(&dad->sysfs_lock);

	length = cyttsp5_ic_parse_input(dad->dev, data->pr_buf, count,
			dad->ic_buf, CY_MAX_PRBUF_SIZE);
	if (length != 1) {
		dev_err(dad->dev, "%s: Malformed input\n", __func__);
		rc = -EINVAL;
		goto exit_unlock;
	}

	dad->get_idac_data_id = dad->ic_buf[0];

exit_unlock:
	mutex_unlock(&dad->sysfs_lock);

	if (rc)
		return rc;

	return count;
}

CY_DEBUGFS_FOPS(get_idac, get_idac_debugfs_read, get_idac_debugfs_write);

static ssize_t calibrate_debugfs_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_debugfs_data *data = filp->private_data;
	struct cyttsp5_device_access_data *dad = data->dad;
	struct device *dev = dad->dev;
	int status = STATUS_FAIL;
	int length = 0;
	int rc;

	if (*ppos)
		goto exit;

	mutex_lock(&dad->sysfs_lock);

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		dev_err(dev, "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto put_pm_runtime;
	}

	rc = cyttsp5_suspend_scan_cmd_(dev);
	if (rc < 0) {
		dev_err(dev, "%s: Error on suspend scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	rc = _cyttsp5_calibrate_idacs_cmd(dev, dad->calibrate_sensing_mode,
			&dad->ic_buf[0]);
	if (rc < 0) {
		dev_err(dev, "%s: Error on calibrate idacs r=%d\n",
				__func__, rc);
		goto resume_scan;
	}

	length = 1;

	/* Check if baseline initialization is requested */
	if (dad->calibrate_initialize_baselines) {
		/* Perform baseline initialization for all modes */
		rc = _cyttsp5_initialize_baselines_cmd(dev, CY_IB_SM_MUTCAP |
				CY_IB_SM_SELFCAP | CY_IB_SM_BUTTON,
				&dad->ic_buf[length]);
		if (rc < 0) {
			dev_err(dev, "%s: Error on initialize baselines r=%d\n",
					__func__, rc);
			goto resume_scan;
		}

		length++;
	}

	status = STATUS_SUCCESS;

resume_scan:
	cyttsp5_resume_scan_cmd_(dev);

release_exclusive:
	cmd->release_exclusive(dev);

put_pm_runtime:
	pm_runtime_put(dev);

	if (status == STATUS_FAIL)
		length = 0;

	data->pr_buf_len = prepare_print_buffer(status, dad->ic_buf, length,
			data->pr_buf, sizeof(data->pr_buf));

	mutex_unlock(&dad->sysfs_lock);

exit:
	return simple_read_from_buffer(buf, count, ppos, data->pr_buf,
			data->pr_buf_len);
}

static ssize_t calibrate_debugfs_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_debugfs_data *data = filp->private_data;
	struct cyttsp5_device_access_data *dad = data->dad;
	ssize_t length;
	int rc = 0;

	rc = simple_write_to_buffer(data->pr_buf, sizeof(data->pr_buf), ppos,
			buf, count);
	if (rc < 0)
		return rc;

	count = rc;

	mutex_lock(&dad->sysfs_lock);

	length = cyttsp5_ic_parse_input(dad->dev, data->pr_buf, count,
			dad->ic_buf, CY_MAX_PRBUF_SIZE);
	if (length != 2) {
		dev_err(dad->dev, "%s: Malformed input\n", __func__);
		rc = -EINVAL;
		goto exit_unlock;
	}

	dad->calibrate_sensing_mode = dad->ic_buf[0];
	dad->calibrate_initialize_baselines = dad->ic_buf[1];

exit_unlock:
	mutex_unlock(&dad->sysfs_lock);

	if (rc)
		return rc;

	return count;
}

CY_DEBUGFS_FOPS(calibrate, calibrate_debugfs_read, calibrate_debugfs_write);

static ssize_t baseline_debugfs_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_debugfs_data *data = filp->private_data;
	struct cyttsp5_device_access_data *dad = data->dad;
	struct device *dev = dad->dev;
	int status = STATUS_FAIL;
	int length = 0;
	int rc;

	if (*ppos)
		goto exit;

	mutex_lock(&dad->sysfs_lock);

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		dev_err(dev, "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto put_pm_runtime;
	}

	rc = cyttsp5_suspend_scan_cmd_(dev);
	if (rc < 0) {
		dev_err(dev, "%s: Error on suspend scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	rc = _cyttsp5_initialize_baselines_cmd(dev, dad->baseline_sensing_mode,
			&dad->ic_buf[0]);
	if (rc < 0) {
		dev_err(dev, "%s: Error on initialize baselines r=%d\n",
				__func__, rc);
		goto resume_scan;
	}

	length = 1;

	status = STATUS_SUCCESS;

resume_scan:
	cyttsp5_resume_scan_cmd_(dev);

release_exclusive:
	cmd->release_exclusive(dev);

put_pm_runtime:
	pm_runtime_put(dev);

	if (status == STATUS_FAIL)
		length = 0;

	data->pr_buf_len = prepare_print_buffer(status, dad->ic_buf, length,
			data->pr_buf, sizeof(data->pr_buf));

	mutex_unlock(&dad->sysfs_lock);

exit:
	return simple_read_from_buffer(buf, count, ppos, data->pr_buf,
			data->pr_buf_len);
}

static ssize_t baseline_debugfs_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_debugfs_data *data = filp->private_data;
	struct cyttsp5_device_access_data *dad = data->dad;
	ssize_t length;
	int rc = 0;

	rc = simple_write_to_buffer(data->pr_buf, sizeof(data->pr_buf), ppos,
			buf, count);
	if (rc < 0)
		return rc;

	count = rc;

	mutex_lock(&dad->sysfs_lock);

	length = cyttsp5_ic_parse_input(dad->dev, buf, count, dad->ic_buf,
			CY_MAX_PRBUF_SIZE);
	if (length != 1) {
		dev_err(dad->dev, "%s: Malformed input\n", __func__);
		rc = -EINVAL;
		goto exit_unlock;
	}

	dad->baseline_sensing_mode = dad->ic_buf[0];

exit_unlock:
	mutex_unlock(&dad->sysfs_lock);

	if (rc)
		return rc;

	return count;
}

CY_DEBUGFS_FOPS(baseline, baseline_debugfs_read, baseline_debugfs_write);

static ssize_t auto_shorts_debugfs_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_debugfs_data *data = filp->private_data;

	if (!*ppos)
		/* Set length to PIP_CMD_MAX_LENGTH to read all */
		data->pr_buf_len = cyttsp5_run_and_get_selftest_result(
			data->dad->dev, data->pr_buf, sizeof(data->pr_buf),
			CY_ST_ID_AUTOSHORTS, PIP_CMD_MAX_LENGTH, false);

	return simple_read_from_buffer(buf, count, ppos, data->pr_buf,
			data->pr_buf_len);
}

CY_DEBUGFS_FOPS(auto_shorts, auto_shorts_debugfs_read, NULL);

static ssize_t opens_debugfs_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_debugfs_data *data = filp->private_data;

	if (!*ppos)
		/* Set length to PIP_CMD_MAX_LENGTH to read all */
		data->pr_buf_len = cyttsp5_run_and_get_selftest_result(
			data->dad->dev, data->pr_buf, sizeof(data->pr_buf),
			CY_ST_ID_OPENS, PIP_CMD_MAX_LENGTH, false);

	return simple_read_from_buffer(buf, count, ppos, data->pr_buf,
			data->pr_buf_len);
}

CY_DEBUGFS_FOPS(opens, opens_debugfs_read, NULL);

static ssize_t cm_panel_debugfs_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_debugfs_data *data = filp->private_data;

	if (!*ppos)
		/* Set length to PIP_CMD_MAX_LENGTH to read all */
		data->pr_buf_len = cyttsp5_run_and_get_selftest_result(
			data->dad->dev, data->pr_buf, sizeof(data->pr_buf),
			CY_ST_ID_CM_PANEL, PIP_CMD_MAX_LENGTH, true);

	return simple_read_from_buffer(buf, count, ppos, data->pr_buf,
			data->pr_buf_len);
}

CY_DEBUGFS_FOPS(cm_panel, cm_panel_debugfs_read, NULL);

static ssize_t cp_panel_debugfs_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_debugfs_data *data = filp->private_data;

	if (!*ppos)
		/* Set length to PIP_CMD_MAX_LENGTH to read all */
		data->pr_buf_len = cyttsp5_run_and_get_selftest_result(
			data->dad->dev, data->pr_buf, sizeof(data->pr_buf),
			CY_ST_ID_CP_PANEL, PIP_CMD_MAX_LENGTH, true);

	return simple_read_from_buffer(buf, count, ppos, data->pr_buf,
			data->pr_buf_len);
}

CY_DEBUGFS_FOPS(cp_panel, cp_panel_debugfs_read, NULL);

static ssize_t cm_button_debugfs_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_debugfs_data *data = filp->private_data;

	if (!*ppos)
		/* Set length to PIP_CMD_MAX_LENGTH to read all */
		data->pr_buf_len = cyttsp5_run_and_get_selftest_result(
			data->dad->dev, data->pr_buf, sizeof(data->pr_buf),
			CY_ST_ID_CM_BUTTON, PIP_CMD_MAX_LENGTH, true);

	return simple_read_from_buffer(buf, count, ppos, data->pr_buf,
			data->pr_buf_len);
}

CY_DEBUGFS_FOPS(cm_button, cm_button_debugfs_read, NULL);

static ssize_t cp_button_debugfs_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_debugfs_data *data = filp->private_data;

	if (!*ppos)
		/* Set length to PIP_CMD_MAX_LENGTH to read all */
		data->pr_buf_len = cyttsp5_run_and_get_selftest_result(
			data->dad->dev, data->pr_buf, sizeof(data->pr_buf),
			CY_ST_ID_CP_BUTTON, PIP_CMD_MAX_LENGTH, true);

	return simple_read_from_buffer(buf, count, ppos, data->pr_buf,
			data->pr_buf_len);
}

CY_DEBUGFS_FOPS(cp_button, cp_button_debugfs_read, NULL);

#ifdef TTHE_TUNER_SUPPORT
static ssize_t tthe_get_panel_data_debugfs_read(struct file *filp,
		char __user *buf, size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_data *dad = filp->private_data;
	struct device *dev;
	u8 config;
	u16 actual_read_len;
	u16 length = 0;
	u8 element_size = 0;
	u8 *buf_offset;
	u8 *buf_out;
	int elem;
	int elem_offset = 0;
	int print_idx = 0;
	int rc;
	int rc1;
	int i;

	mutex_lock(&dad->debugfs_lock);
	dev = dad->dev;
	buf_out = dad->tthe_get_panel_data_buf;
	if (!buf_out)
		goto release_mutex;

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0)
		goto put_runtime;

	if (dad->heatmap.scan_start) {
		/* Start scan */
		rc = cyttsp5_exec_scan_cmd_(dev);
		if (rc < 0)
			goto release_exclusive;
	}

	elem = dad->heatmap.num_element;
	rc = cyttsp5_ret_scan_data_cmd_(dev, elem_offset, elem,
			dad->heatmap.data_type, dad->ic_buf, &config,
			&actual_read_len, NULL);
	if (rc < 0)
		goto release_exclusive;

	length = get_unaligned_le16(&dad->ic_buf[0]);
	buf_offset = dad->ic_buf + length;

	element_size = config & CY_CMD_RET_PANEL_ELMNT_SZ_MASK;

	elem -= actual_read_len;
	elem_offset = actual_read_len;
	while (elem > 0) {
		rc = cyttsp5_ret_scan_data_cmd_(dev, elem_offset, elem,
				dad->heatmap.data_type, NULL, &config,
				&actual_read_len, buf_offset);
		if (rc < 0)
			goto release_exclusive;

		if (!actual_read_len)
			break;

		length += actual_read_len * element_size;
		buf_offset = dad->ic_buf + length;
		elem -= actual_read_len;
		elem_offset += actual_read_len;
	}

	/* Reconstruct cmd header */
	put_unaligned_le16(length, &dad->ic_buf[0]);
	put_unaligned_le16(elem_offset, &dad->ic_buf[7]);

release_exclusive:
	rc1 = cmd->release_exclusive(dev);
put_runtime:
	pm_runtime_put(dev);

	if (rc < 0)
		goto release_mutex;

	print_idx += scnprintf(buf_out, TTHE_TUNER_MAX_BUF, "CY_DATA:");
	for (i = 0; i < length; i++)
		print_idx += scnprintf(buf_out + print_idx,
				TTHE_TUNER_MAX_BUF - print_idx,
				"%02X ", dad->ic_buf[i]);
	print_idx += scnprintf(buf_out + print_idx,
			TTHE_TUNER_MAX_BUF - print_idx,
			":(%d bytes)\n", length);
	rc = simple_read_from_buffer(buf, count, ppos, buf_out, print_idx);
	print_idx = rc;

release_mutex:
	mutex_unlock(&dad->debugfs_lock);
	return print_idx;
}

static ssize_t tthe_get_panel_data_debugfs_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_data *dad = filp->private_data;
	struct device *dev = dad->dev;
	ssize_t length;
	int max_read;
	u8 *buf_in = dad->tthe_get_panel_data_buf;
	int ret;

	mutex_lock(&dad->debugfs_lock);
	ret = copy_from_user(buf_in + (*ppos), buf, count);
	if (ret)
		goto exit;
	buf_in[count] = 0;

	length = cyttsp5_ic_parse_input(dev, buf_in, count, dad->ic_buf,
			CY_MAX_PRBUF_SIZE);
	if (length <= 0) {
		dev_err(dev, "%s: %s Group Data store\n", __func__,
				"Malformed input for");
		goto exit;
	}

	/* update parameter value */
	dad->heatmap.num_element = get_unaligned_le16(&dad->ic_buf[3]);
	dad->heatmap.data_type = dad->ic_buf[5];

	if (dad->ic_buf[6] > 0)
		dad->heatmap.scan_start = true;
	else
		dad->heatmap.scan_start = false;

	/* elem can not be bigger then buffer size */
	max_read = CY_CMD_RET_PANEL_HDR;
	max_read += dad->heatmap.num_element * CY_CMD_RET_PANEL_ELMNT_SZ_MAX;

	if (max_read >= CY_MAX_PRBUF_SIZE) {
		dad->heatmap.num_element =
			(CY_MAX_PRBUF_SIZE - CY_CMD_RET_PANEL_HDR)
			/ CY_CMD_RET_PANEL_ELMNT_SZ_MAX;
		dev_err(dev, "%s: Will get %d element\n", __func__,
				dad->heatmap.num_element);
	}

exit:
	mutex_unlock(&dad->debugfs_lock);
	dev_vdbg(dev, "%s: return count=%zu\n", __func__, count);
	return count;
}

static int tthe_get_panel_data_debugfs_open(struct inode *inode,
		struct file *filp)
{
	struct cyttsp5_device_access_data *dad = inode->i_private;

	mutex_lock(&dad->debugfs_lock);

	if (dad->tthe_get_panel_data_is_open) {
		mutex_unlock(&dad->debugfs_lock);
		return -EBUSY;
	}

	filp->private_data = inode->i_private;

	dad->tthe_get_panel_data_is_open = 1;
	mutex_unlock(&dad->debugfs_lock);
	return 0;
}

static int tthe_get_panel_data_debugfs_close(struct inode *inode,
		struct file *filp)
{
	struct cyttsp5_device_access_data *dad = filp->private_data;

	mutex_lock(&dad->debugfs_lock);
	filp->private_data = NULL;
	dad->tthe_get_panel_data_is_open = 0;
	mutex_unlock(&dad->debugfs_lock);

	return 0;
}

static const struct file_operations tthe_get_panel_data_fops = {
	.open = tthe_get_panel_data_debugfs_open,
	.release = tthe_get_panel_data_debugfs_close,
	.read = tthe_get_panel_data_debugfs_read,
	.write = tthe_get_panel_data_debugfs_write,
};
#endif

static int cyttsp5_setup_sysfs(struct device *dev)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	int rc;

	rc = device_create_file(dev, &dev_attr_command);
	if (rc) {
		dev_err(dev, "%s: Error, could not create command\n",
				__func__);
		goto exit;
	}

	rc = device_create_file(dev, &dev_attr_status);
	if (rc) {
		dev_err(dev, "%s: Error, could not create status\n",
				__func__);
		goto unregister_command;
	}

	rc = device_create_file(dev, &dev_attr_response);
	if (rc) {
		dev_err(dev, "%s: Error, could not create response\n",
				__func__);
		goto unregister_status;
	}

	dad->base_dentry = debugfs_create_dir(dev_name(dev), NULL);
	if (IS_ERR_OR_NULL(dad->base_dentry)) {
		dev_err(dev, "%s: Error, could not create base directory\n",
				__func__);
		goto unregister_response;
	}

	dad->mfg_test_dentry = debugfs_create_dir("mfg_test",
			dad->base_dentry);
	if (IS_ERR_OR_NULL(dad->mfg_test_dentry)) {
		dev_err(dev, "%s: Error, could not create mfg_test directory\n",
				__func__);
		goto unregister_base_dir;
	}

	if (IS_ERR_OR_NULL(debugfs_create_file("panel_scan", 0600,
			dad->mfg_test_dentry, dad,
			&panel_scan_debugfs_fops))) {
		dev_err(dev, "%s: Error, could not create panel_scan\n",
				__func__);
		goto unregister_base_dir;
	}

	if (IS_ERR_OR_NULL(debugfs_create_file("get_idac", 0600,
			dad->mfg_test_dentry, dad, &get_idac_debugfs_fops))) {
		dev_err(dev, "%s: Error, could not create get_idac\n",
				__func__);
		goto unregister_base_dir;
	}

	if (IS_ERR_OR_NULL(debugfs_create_file("auto_shorts", 0400,
			dad->mfg_test_dentry, dad,
			&auto_shorts_debugfs_fops))) {
		dev_err(dev, "%s: Error, could not create auto_shorts\n",
				__func__);
		goto unregister_base_dir;
	}

	if (IS_ERR_OR_NULL(debugfs_create_file("opens", 0400,
			dad->mfg_test_dentry, dad, &opens_debugfs_fops))) {
		dev_err(dev, "%s: Error, could not create opens\n",
				__func__);
		goto unregister_base_dir;
	}

	if (IS_ERR_OR_NULL(debugfs_create_file("calibrate", 0600,
			dad->mfg_test_dentry, dad, &calibrate_debugfs_fops))) {
		dev_err(dev, "%s: Error, could not create calibrate\n",
				__func__);
		goto unregister_base_dir;
	}

	if (IS_ERR_OR_NULL(debugfs_create_file("baseline", 0600,
			dad->mfg_test_dentry, dad, &baseline_debugfs_fops))) {
		dev_err(dev, "%s: Error, could not create baseline\n",
				__func__);
		goto unregister_base_dir;
	}

	if (IS_ERR_OR_NULL(debugfs_create_file("cm_panel", 0400,
			dad->mfg_test_dentry, dad, &cm_panel_debugfs_fops))) {
		dev_err(dev, "%s: Error, could not create cm_panel\n",
				__func__);
		goto unregister_base_dir;
	}

	if (IS_ERR_OR_NULL(debugfs_create_file("cp_panel", 0400,
			dad->mfg_test_dentry, dad, &cp_panel_debugfs_fops))) {
		dev_err(dev, "%s: Error, could not create cp_panel\n",
				__func__);
		goto unregister_base_dir;
	}

	if (IS_ERR_OR_NULL(debugfs_create_file("cm_button", 0400,
			dad->mfg_test_dentry, dad, &cm_button_debugfs_fops))) {
		dev_err(dev, "%s: Error, could not create cm_button\n",
				__func__);
		goto unregister_base_dir;
	}

	if (IS_ERR_OR_NULL(debugfs_create_file("cp_button", 0400,
			dad->mfg_test_dentry, dad, &cp_button_debugfs_fops))) {
		dev_err(dev, "%s: Error, could not create cp_button\n",
				__func__);
		goto unregister_base_dir;
	}

#ifdef TTHE_TUNER_SUPPORT
	dad->tthe_get_panel_data_debugfs = debugfs_create_file(
			CYTTSP5_TTHE_TUNER_GET_PANEL_DATA_FILE_NAME,
			0644, NULL, dad, &tthe_get_panel_data_fops);
	if (IS_ERR_OR_NULL(dad->tthe_get_panel_data_debugfs)) {
		dev_err(dev, "%s: Error, could not create get_panel_data\n",
				__func__);
		dad->tthe_get_panel_data_debugfs = NULL;
		goto unregister_base_dir;
	}
#endif

	dad->sysfs_nodes_created = true;
	return rc;

unregister_base_dir:
	debugfs_remove_recursive(dad->base_dentry);
unregister_response:
	device_remove_file(dev, &dev_attr_response);
unregister_status:
	device_remove_file(dev, &dev_attr_status);
unregister_command:
	device_remove_file(dev, &dev_attr_command);
exit:
	return rc;
}

static int cyttsp5_setup_sysfs_attention(struct device *dev)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	int rc = 0;

	dad->si = cmd->request_sysinfo(dev);
	if (!dad->si)
		return -EINVAL;

	rc = cyttsp5_setup_sysfs(dev);

	cmd->unsubscribe_attention(dev, CY_ATTEN_STARTUP,
		CYTTSP5_DEVICE_ACCESS_NAME, cyttsp5_setup_sysfs_attention,
		0);

	return rc;
}

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICE_ACCESS_API
int cyttsp5_device_access_user_command(const char *core_name, u16 read_len,
		u8 *read_buf, u16 write_len, u8 *write_buf,
		u16 *actual_read_len)
{
	struct cyttsp5_core_data *cd;
	int rc;

	might_sleep();

	/* Check parameters */
	if (!read_buf || !write_buf || !actual_read_len)
		return -EINVAL;

	if (!core_name)
		core_name = CY_DEFAULT_CORE_ID;

	/* Find device */
	cd = cyttsp5_get_core_data((char *)core_name);
	if (!cd) {
		pr_err("%s: No device.\n", __func__);
		return -ENODEV;
	}

	pm_runtime_get_sync(cd->dev);
	rc = cmd->nonhid_cmd->user_cmd(cd->dev, 1, read_len, read_buf,
			write_len, write_buf, actual_read_len);
	pm_runtime_put(cd->dev);

	return rc;
}
EXPORT_SYMBOL_GPL(cyttsp5_device_access_user_command);

struct command_work {
	struct work_struct work;
	const char *core_name;
	u16 read_len;
	u8 *read_buf;
	u16 write_len;
	u8 *write_buf;

	void (*cont)(const char *core_name, u16 read_len, u8 *read_buf,
		u16 write_len, u8 *write_buf, u16 actual_read_length,
		int rc);
};

static void cyttsp5_device_access_user_command_work_func(
		struct work_struct *work)
{
	struct command_work *cmd_work =
			container_of(work, struct command_work, work);
	u16 actual_read_length;
	int rc;

	rc = cyttsp5_device_access_user_command(cmd_work->core_name,
			cmd_work->read_len, cmd_work->read_buf,
			cmd_work->write_len, cmd_work->write_buf,
			&actual_read_length);

	if (cmd_work->cont)
		cmd_work->cont(cmd_work->core_name,
			cmd_work->read_len, cmd_work->read_buf,
			cmd_work->write_len, cmd_work->write_buf,
			actual_read_length, rc);

	kfree(cmd_work);
}

int cyttsp5_device_access_user_command_async(const char *core_name,
		u16 read_len, u8 *read_buf, u16 write_len, u8 *write_buf,
		void (*cont)(const char *core_name, u16 read_len, u8 *read_buf,
			u16 write_len, u8 *write_buf, u16 actual_read_length,
			int rc))
{
	struct command_work *cmd_work;

	cmd_work = kzalloc(sizeof(*cmd_work), GFP_ATOMIC);
	if (!cmd_work)
		return -ENOMEM;

	cmd_work->core_name = core_name;
	cmd_work->read_len = read_len;
	cmd_work->read_buf = read_buf;
	cmd_work->write_len = write_len;
	cmd_work->write_buf = write_buf;
	cmd_work->cont = cont;

	INIT_WORK(&cmd_work->work,
			cyttsp5_device_access_user_command_work_func);
	schedule_work(&cmd_work->work);

	return 0;
}
EXPORT_SYMBOL_GPL(cyttsp5_device_access_user_command_async);
#endif

static int cyttsp5_device_access_probe(struct device *dev, void **data)
{
	struct cyttsp5_device_access_data *dad;
	int rc;

	dad = kzalloc(sizeof(*dad), GFP_KERNEL);
	if (!dad) {
		rc = -ENOMEM;
		goto cyttsp5_device_access_probe_data_failed;
	}

	mutex_init(&dad->sysfs_lock);
	dad->dev = dev;
#ifdef TTHE_TUNER_SUPPORT
	mutex_init(&dad->debugfs_lock);
	dad->heatmap.num_element = 200;
#endif
	*data = dad;

	/* get sysinfo */
	dad->si = cmd->request_sysinfo(dev);
	if (dad->si) {
		rc = cyttsp5_setup_sysfs(dev);
		if (rc)
			goto cyttsp5_device_access_setup_sysfs_failed;
	} else {
		dev_err(dev, "%s: Fail get sysinfo pointer from core p=%p\n",
				__func__, dad->si);
		cmd->subscribe_attention(dev, CY_ATTEN_STARTUP,
			CYTTSP5_DEVICE_ACCESS_NAME,
			cyttsp5_setup_sysfs_attention, 0);
	}

	return 0;

 cyttsp5_device_access_setup_sysfs_failed:
	kfree(dad);
 cyttsp5_device_access_probe_data_failed:
	dev_err(dev, "%s failed.\n", __func__);
	return rc;
}

static void cyttsp5_device_access_release(struct device *dev, void *data)
{
	struct cyttsp5_device_access_data *dad = data;

	if (dad->sysfs_nodes_created) {
		device_remove_file(dev, &dev_attr_command);
		device_remove_file(dev, &dev_attr_status);
		device_remove_file(dev, &dev_attr_response);
		debugfs_remove_recursive(dad->base_dentry);
#ifdef TTHE_TUNER_SUPPORT
		debugfs_remove(dad->tthe_get_panel_data_debugfs);
#endif
	} else {
		cmd->unsubscribe_attention(dev, CY_ATTEN_STARTUP,
			CYTTSP5_DEVICE_ACCESS_NAME,
			cyttsp5_setup_sysfs_attention, 0);
	}

	kfree(dad);
}

static struct cyttsp5_module device_access_module = {
	.name = CYTTSP5_DEVICE_ACCESS_NAME,
	.probe = cyttsp5_device_access_probe,
	.release = cyttsp5_device_access_release,
};

static int __init cyttsp5_device_access_init(void)
{
	int rc;

	cmd = cyttsp5_get_commands();
	if (!cmd)
		return -EINVAL;

	rc = cyttsp5_register_module(&device_access_module);
	if (rc < 0) {
		pr_err("%s: Error, failed registering module\n",
			__func__);
			return rc;
	}

	pr_info("%s: Cypress TTSP Device Access Driver (Built %s) rc=%d\n",
		 __func__, CY_DRIVER_VERSION, rc);
	return 0;
}
module_init(cyttsp5_device_access_init);

static void __exit cyttsp5_device_access_exit(void)
{
	cyttsp5_unregister_module(&device_access_module);
}
module_exit(cyttsp5_device_access_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product Device Access Driver");
MODULE_AUTHOR("Cypress Semiconductor <ttdrivers@cypress.com>");
