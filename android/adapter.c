/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2013  Intel Corporation. All rights reserved.
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <errno.h>

#include <glib.h>

#include "lib/bluetooth.h"
#include "src/shared/mgmt.h"
#include "lib/mgmt.h"
#include "log.h"
#include "hal-msg.h"
#include "ipc.h"
#include "utils.h"
#include "adapter.h"

/* Default to DisplayYesNo */
#define DEFAULT_IO_CAPABILITY 0x01

static GIOChannel *notification_io = NULL;

struct bt_adapter {
	uint16_t index;
	struct mgmt *mgmt;

	bt_adapter_ready ready;

	bdaddr_t bdaddr;
	uint32_t dev_class;

	char *name;

	uint32_t supported_settings;
	uint32_t current_settings;
};

static struct bt_adapter *adapter;

static void mgmt_local_name_changed_event(uint16_t index, uint16_t length,
					const void *param, void *user_data)
{
	const struct mgmt_cp_set_local_name *rp = param;

	if (length < sizeof(*rp)) {
		error("Wrong size of local name changed parameters");
		return;
	}

	if (!g_strcmp0(adapter->name, (const char *) rp->name))
		return;

	DBG("name: %s", rp->name);

	g_free(adapter->name);
	adapter->name = g_strdup((const char *) rp->name);

	/* TODO Update services if needed */
}

static void powered_changed(void)
{
	struct hal_ev_adapter_state_changed ev;

	ev.state = (adapter->current_settings & MGMT_SETTING_POWERED) ?
						HAL_POWER_ON : HAL_POWER_OFF;

	DBG("%u", ev.state);

	ipc_send(notification_io, HAL_SERVICE_ID_BLUETOOTH,
			HAL_EV_ADAPTER_STATE_CHANGED, sizeof(ev), &ev, -1);
}

static uint8_t settings2scan_mode(void)
{
	bool connectable, discoverable;

	connectable = adapter->current_settings & MGMT_SETTING_CONNECTABLE;
	discoverable = adapter->current_settings & MGMT_SETTING_DISCOVERABLE;

	if (connectable && discoverable)
		return HAL_ADAPTER_SCAN_MODE_CONN_DISC;

	if (connectable)
		return HAL_ADAPTER_SCAN_MODE_CONN;

	return HAL_ADAPTER_SCAN_MODE_NONE;
}

static void scan_mode_changed(void)
{
	struct hal_ev_adapter_props_changed *ev;
	uint8_t *mode;
	int len;
	len = sizeof(*ev) + sizeof(struct hal_property) + 1;

	ev = g_malloc(len);

	ev->num_props = 1;
	ev->status = HAL_STATUS_SUCCESS;

	ev->props[0].type = HAL_PROP_ADAPTER_SCAN_MODE;
	ev->props[0].len = 1;

	mode = ev->props[0].val;
	*mode = settings2scan_mode();

	DBG("mode %u", *mode);

	ipc_send(notification_io, HAL_SERVICE_ID_BLUETOOTH,
				HAL_EV_ADAPTER_PROPS_CHANGED, len, ev, -1);

	g_free(ev);
}

static void settings_changed(uint32_t settings)
{
	uint32_t changed_mask;
	uint32_t scan_mode_mask;

	changed_mask = adapter->current_settings ^ settings;

	adapter->current_settings = settings;

	DBG("0x%08x", changed_mask);

	if (changed_mask & MGMT_SETTING_POWERED)
		powered_changed();

	scan_mode_mask = MGMT_SETTING_CONNECTABLE | MGMT_SETTING_DISCOVERABLE;

	if (changed_mask & scan_mode_mask)
		scan_mode_changed();
}

static void new_settings_callback(uint16_t index, uint16_t length,
					const void *param, void *user_data)
{
	uint32_t settings;

	if (length < sizeof(settings)) {
		error("Wrong size of new settings parameters");
		return;
	}

	settings = bt_get_le32(param);

	DBG("settings: 0x%8.8x -> 0x%8.8x", adapter->current_settings,
								settings);

	if (settings == adapter->current_settings)
		return;

	settings_changed(settings);
}

static void mgmt_dev_class_changed_event(uint16_t index, uint16_t length,
					const void *param, void *user_data)
{
	const struct mgmt_cod *rp = param;
	uint32_t dev_class;

	if (length < sizeof(*rp)) {
		error("Wrong size of class of device changed parameters");
		return;
	}

	dev_class = rp->val[0] | (rp->val[1] << 8) | (rp->val[2] << 16);

	if (dev_class == adapter->dev_class)
		return;

	DBG("Class: 0x%06x", dev_class);

	adapter->dev_class = dev_class;

	/* TODO: Inform prop change: Class */

	/* TODO: Gatt attrib set*/
}

static void store_link_key(const bdaddr_t *dst, const uint8_t *key,
					uint8_t type, uint8_t pin_length)
{
	/* TODO store link key */

}

static void send_bond_state_change(const bdaddr_t *addr, uint8_t status,
								uint8_t state)
{
	struct hal_ev_bond_state_changed ev;

	ev.status = status;
	ev.state = state;
	bdaddr2android(addr, ev.bdaddr);

	ipc_send(notification_io, HAL_SERVICE_ID_BLUETOOTH,
			HAL_EV_BOND_STATE_CHANGED, sizeof(ev), &ev, -1);
}

static void new_link_key_callback(uint16_t index, uint16_t length,
					const void *param, void *user_data)
{
	const struct mgmt_ev_new_link_key *ev = param;
	const struct mgmt_addr_info *addr = &ev->key.addr;
	char dst[18];

	if (length < sizeof(*ev)) {
		error("Too small new link key event");
		return;
	}

	ba2str(&addr->bdaddr, dst);

	DBG("new key for %s type %u pin_len %u",
					dst, ev->key.type, ev->key.pin_len);

	if (ev->key.pin_len > 16) {
		error("Invalid PIN length (%u) in new_key event",
							ev->key.pin_len);
		return;
	}

	if (ev->store_hint) {
		const struct mgmt_link_key_info *key = &ev->key;

		store_link_key(&addr->bdaddr, key->val, key->type,
								key->pin_len);
	}

	send_bond_state_change(&addr->bdaddr, HAL_STATUS_SUCCESS,
							HAL_BOND_STATE_BONDED);
}

static void pin_code_request_callback(uint16_t index, uint16_t length,
					const void *param, void *user_data)
{
	const struct mgmt_ev_pin_code_request *ev = param;
	struct hal_ev_pin_request hal_ev;
	char dst[18];

	if (length < sizeof(*ev)) {
		error("Too small PIN code request event");
		return;
	}

	ba2str(&ev->addr.bdaddr, dst);

	DBG("%s type %u secure %u", dst, ev->addr.type, ev->secure);

	/* TODO name and CoD of remote devices should probably be cached */
	memset(&hal_ev, 0, sizeof(hal_ev));
	bdaddr2android(&ev->addr.bdaddr, hal_ev.bdaddr);

	ipc_send(notification_io, HAL_SERVICE_ID_BLUETOOTH, HAL_EV_PIN_REQUEST,
						sizeof(hal_ev), &hal_ev, -1);
}

static void send_ssp_request(const bdaddr_t *addr, uint8_t variant,
							uint32_t passkey)
{
	struct hal_ev_ssp_request ev;

	/* TODO name and CoD of remote devices should probably be cached */
	memset(&ev, 0, sizeof(ev));
	bdaddr2android(addr, ev.bdaddr);
	ev.pairing_variant = variant;
	ev.passkey = passkey;

	ipc_send(notification_io, HAL_SERVICE_ID_BLUETOOTH, HAL_EV_SSP_REQUEST,
						sizeof(ev), &ev, -1);
}

static void user_confirm_request_callback(uint16_t index, uint16_t length,
					const void *param, void *user_data)
{
	const struct mgmt_ev_user_confirm_request *ev = param;
	char dst[18];

	if (length < sizeof(*ev)) {
		error("Too small user confirm request event");
		return;
	}

	ba2str(&ev->addr.bdaddr, dst);
	DBG("%s confirm_hint %u", dst, ev->confirm_hint);

	if (ev->confirm_hint)
		send_ssp_request(&ev->addr.bdaddr, HAL_SSP_VARIANT_CONSENT, 0);
	else
		send_ssp_request(&ev->addr.bdaddr, HAL_SSP_VARIANT_CONFIRM,
								ev->value);
}

static void user_passkey_request_callback(uint16_t index, uint16_t length,
					const void *param, void *user_data)
{
	const struct mgmt_ev_user_passkey_request *ev = param;
	char dst[18];

	if (length < sizeof(*ev)) {
		error("Too small passkey request event");
		return;
	}

	ba2str(&ev->addr.bdaddr, dst);
	DBG("%s", dst);

	send_ssp_request(&ev->addr.bdaddr, HAL_SSP_VARIANT_ENTRY, 0);
}

static void user_passkey_notify_callback(uint16_t index, uint16_t length,
					const void *param, void *user_data)
{
	const struct mgmt_ev_passkey_notify *ev = param;
	char dst[18];

	if (length < sizeof(*ev)) {
		error("Too small passkey notify event");
		return;
	}

	ba2str(&ev->addr.bdaddr, dst);
	DBG("%s entered %u", dst, ev->entered);

	/* HAL seems to not support entered characters */
	if (!ev->entered)
		send_ssp_request(&ev->addr.bdaddr, HAL_SSP_VARIANT_NOTIF,
								ev->passkey);
}

static void register_mgmt_handlers(void)
{
	mgmt_register(adapter->mgmt, MGMT_EV_NEW_SETTINGS, adapter->index,
					new_settings_callback, NULL, NULL);

	mgmt_register(adapter->mgmt, MGMT_EV_CLASS_OF_DEV_CHANGED,
				adapter->index, mgmt_dev_class_changed_event,
				NULL, NULL);

	mgmt_register(adapter->mgmt, MGMT_EV_LOCAL_NAME_CHANGED,
				adapter->index, mgmt_local_name_changed_event,
				NULL, NULL);

	mgmt_register(adapter->mgmt, MGMT_EV_NEW_LINK_KEY, adapter->index,
					new_link_key_callback, NULL, NULL);

	mgmt_register(adapter->mgmt, MGMT_EV_PIN_CODE_REQUEST, adapter->index,
					pin_code_request_callback, NULL, NULL);

	mgmt_register(adapter->mgmt, MGMT_EV_USER_CONFIRM_REQUEST,
				adapter->index, user_confirm_request_callback,
				NULL, NULL);

	mgmt_register(adapter->mgmt, MGMT_EV_USER_PASSKEY_REQUEST,
				adapter->index, user_passkey_request_callback,
				NULL, NULL);

	mgmt_register(adapter->mgmt, MGMT_EV_PASSKEY_NOTIFY, adapter->index,
				user_passkey_notify_callback, NULL, NULL);

}

static void load_link_keys_complete(uint8_t status, uint16_t length,
					const void *param, void *user_data)
{
	int err;

	if (status) {
		error("Failed to load link keys for index %u: %s (0x%02x)",
			adapter->index, mgmt_errstr(status), status);
		err = -EIO;
		goto failed;
	}

	DBG("status %u", status);

	adapter->ready(0);
	return;

failed:
	adapter->ready(err);
}

static void load_link_keys(GSList *keys)
{
	struct mgmt_cp_load_link_keys *cp;
	struct mgmt_link_key_info *key;
	size_t key_count, cp_size;
	unsigned int id;

	key_count = g_slist_length(keys);

	DBG("keys %zu ", key_count);

	cp_size = sizeof(*cp) + (key_count * sizeof(*key));

	cp = g_malloc0(cp_size);

	/*
	 * Even if the list of stored keys is empty, it is important to
	 * load an empty list into the kernel. That way it is ensured
	 * that no old keys from a previous daemon are present.
	 */
	cp->key_count = htobs(key_count);

	for (key = cp->keys; keys != NULL; keys = g_slist_next(keys), key++)
		memcpy(key, keys->data, sizeof(*key));

	id = mgmt_send(adapter->mgmt, MGMT_OP_LOAD_LINK_KEYS, adapter->index,
			cp_size, cp, load_link_keys_complete, NULL, NULL);

	g_free(cp);

	if (id == 0) {
		error("Failed to load link keys");
		adapter->ready(-EIO);
	}
}

static void set_mode_complete(uint8_t status, uint16_t length,
					const void *param, void *user_data)
{
	if (status != MGMT_STATUS_SUCCESS) {
		error("Failed to set mode: %s (0x%02x)",
						mgmt_errstr(status), status);
		return;
	}

	/*
	 * The parameters are identical and also the task that is
	 * required in both cases. So it is safe to just call the
	 * event handling functions here.
	 */
	new_settings_callback(adapter->index, length, param, NULL);
}

static bool set_mode(uint16_t opcode, uint8_t mode)
{
	struct mgmt_mode cp;

	memset(&cp, 0, sizeof(cp));
	cp.val = mode;

	DBG("opcode=0x%x mode=0x%x", opcode, mode);

	if (mgmt_send(adapter->mgmt, opcode, adapter->index, sizeof(cp), &cp,
					set_mode_complete, NULL, NULL) > 0)
		return true;

	error("Failed to set mode");

	return false;
}

static void set_io_capability(void)
{
	struct mgmt_cp_set_io_capability cp;

	memset(&cp, 0, sizeof(cp));
	cp.io_capability = DEFAULT_IO_CAPABILITY;

	if (mgmt_send(adapter->mgmt, MGMT_OP_SET_IO_CAPABILITY,
				adapter->index, sizeof(cp), &cp,
				NULL, NULL, NULL) == 0)
		error("Failed to set IO capability");
}

static void read_info_complete(uint8_t status, uint16_t length, const void *param,
							void *user_data)
{
	const struct mgmt_rp_read_info *rp = param;
	int err;

	DBG("");

	if (status) {
		error("Failed to read info for index %u: %s (0x%02x)",
				adapter->index, mgmt_errstr(status), status);
		err = -EIO;
		goto failed;
	}

	if (length < sizeof(*rp)) {
		error("Too small read info complete response");
		err = -EIO;
		goto failed;
	}

	if (!bacmp(&rp->bdaddr, BDADDR_ANY)) {
		error("No Bluetooth address");
		err = -ENODEV;
		goto failed;
	}

	/* Store adapter information */
	bacpy(&adapter->bdaddr, &rp->bdaddr);
	adapter->dev_class = rp->dev_class[0] | (rp->dev_class[1] << 8) |
						(rp->dev_class[2] << 16);
	adapter->name = g_strdup((const char *) rp->name);

	adapter->supported_settings = btohs(rp->supported_settings);
	adapter->current_settings = btohs(rp->current_settings);

	/* TODO: Register all event notification handlers */
	register_mgmt_handlers();

	load_link_keys(NULL);

	set_io_capability();
	set_mode(MGMT_OP_SET_PAIRABLE, 0x01);

	return;

failed:
	adapter->ready(err);
}

void bt_adapter_init(uint16_t index, struct mgmt *mgmt, bt_adapter_ready cb)
{
	adapter = g_new0(struct bt_adapter, 1);

	adapter->mgmt = mgmt_ref(mgmt);
	adapter->index = index;
	adapter->ready = cb;

	if (mgmt_send(mgmt, MGMT_OP_READ_INFO, index, 0, NULL,
					read_info_complete, NULL, NULL) > 0)
		return;

	mgmt_unref(adapter->mgmt);
	adapter->ready(-EIO);
}

static bool set_discoverable(uint8_t mode, uint16_t timeout)
{
	struct mgmt_cp_set_discoverable cp;

	memset(&cp, 0, sizeof(cp));
	cp.val = mode;
	cp.timeout = htobs(timeout);

	DBG("mode %u timeout %u", mode, timeout);

	if (mgmt_send(adapter->mgmt, MGMT_OP_SET_DISCOVERABLE,
				adapter->index, sizeof(cp), &cp,
				set_mode_complete, adapter, NULL) > 0)
		return true;

	error("Failed to set mode discoverable");

	return false;
}

static void send_adapter_address(void)
{
	struct hal_ev_adapter_props_changed *ev;
	int len;

	len = sizeof(*ev) + sizeof(struct hal_property) + sizeof(bdaddr_t);

	ev = g_malloc(len);

	ev->num_props = 1;
	ev->status = HAL_STATUS_SUCCESS;

	ev->props[0].type = HAL_PROP_ADAPTER_ADDR;
	ev->props[0].len = sizeof(bdaddr_t);
	bdaddr2android(&adapter->bdaddr, ev->props[0].val);

	ipc_send(notification_io, HAL_SERVICE_ID_BLUETOOTH,
				HAL_EV_ADAPTER_PROPS_CHANGED, len, ev, -1);

	g_free(ev);
}

static bool get_property(void *buf, uint16_t len)
{
	struct hal_cmd_get_adapter_prop *cmd = buf;

	switch (cmd->type) {
	case HAL_PROP_ADAPTER_ADDR:
		send_adapter_address();
		return true;
	case HAL_PROP_ADAPTER_NAME:
	case HAL_PROP_ADAPTER_UUIDS:
	case HAL_PROP_ADAPTER_CLASS:
	case HAL_PROP_ADAPTER_TYPE:
	case HAL_PROP_ADAPTER_SERVICE_REC:
	case HAL_PROP_ADAPTER_SCAN_MODE:
	case HAL_PROP_ADAPTER_BONDED_DEVICES:
	case HAL_PROP_ADAPTER_DISC_TIMEOUT:
	default:
		return false;
	}
}

static uint8_t set_scan_mode(void *buf, uint16_t len)
{
	uint8_t *mode = buf;
	bool conn, disc, cur_conn, cur_disc;

	cur_conn = adapter->current_settings & MGMT_SETTING_CONNECTABLE;
	cur_disc = adapter->current_settings & MGMT_SETTING_DISCOVERABLE;

	DBG("connectable %u discoverable %d mode %u", cur_conn, cur_disc,
								*mode);

	switch (*mode) {
	case HAL_ADAPTER_SCAN_MODE_NONE:
		if (!cur_conn && !cur_disc)
			return HAL_STATUS_DONE;

		conn = false;
		disc = false;
		break;
	case HAL_ADAPTER_SCAN_MODE_CONN:
		if (cur_conn && !cur_disc)
			return HAL_STATUS_DONE;

		conn = true;
		disc = false;
		break;
	case HAL_ADAPTER_SCAN_MODE_CONN_DISC:
		if (cur_conn && cur_disc)
			return HAL_STATUS_DONE;

		conn = true;
		disc = true;
		break;
	default:
		return HAL_STATUS_FAILED;
	}

	if (cur_conn != conn) {
		if (!set_mode(MGMT_OP_SET_CONNECTABLE, conn ? 0x01 : 0x00))
			return HAL_STATUS_FAILED;
	}

	if (cur_disc != disc) {
		if (!set_discoverable(disc ? 0x01 : 0x00, 0))
			return HAL_STATUS_FAILED;
	}

	return HAL_STATUS_SUCCESS;
}

static uint8_t set_property(void *buf, uint16_t len)
{
	struct hal_cmd_set_adapter_prop *cmd = buf;

	switch (cmd->type) {
	case HAL_PROP_ADAPTER_SCAN_MODE:
		return set_scan_mode(cmd->val, cmd->len);
	case HAL_PROP_ADAPTER_NAME:
	case HAL_PROP_ADAPTER_DISC_TIMEOUT:
	default:
		DBG("Unhandled property type 0x%x", cmd->type);
		return HAL_STATUS_FAILED;
	}
}

static uint8_t status_mgmt2hal(uint8_t mgmt)
{
	switch (mgmt) {
	case MGMT_STATUS_SUCCESS:
		return HAL_STATUS_SUCCESS;
	case MGMT_STATUS_NO_RESOURCES:
		return HAL_STATUS_NOMEM;
	case MGMT_STATUS_BUSY:
		return HAL_STATUS_BUSY;
	case MGMT_STATUS_NOT_SUPPORTED:
		return HAL_STATUS_UNSUPPORTED;
	case MGMT_STATUS_INVALID_PARAMS:
		return HAL_STATUS_INVALID;
	case MGMT_STATUS_AUTH_FAILED:
		return HAL_STATUS_AUTH_FAILURE;
	case MGMT_STATUS_NOT_CONNECTED:
		return HAL_STATUS_REMOTE_DEVICE_DOWN;
	default:
		return HAL_STATUS_FAILED;
	}
}

static void pair_device_complete(uint8_t status, uint16_t length,
					const void *param, void *user_data)
{
	const struct mgmt_rp_pair_device *rp = param;

	DBG("status %u", status);

	/* On success bond state change will be send when new link key event
	 * is received */
	if (status == MGMT_STATUS_SUCCESS)
		return;

	send_bond_state_change(&rp->addr.bdaddr, status_mgmt2hal(status),
							HAL_BOND_STATE_NONE);
}

static bool create_bond(void *buf, uint16_t len)
{
	struct hal_cmd_create_bond *cmd = buf;
	struct mgmt_cp_pair_device cp;

	cp.io_cap = DEFAULT_IO_CAPABILITY;
	cp.addr.type = BDADDR_BREDR;
	android2bdaddr(cmd->bdaddr, &cp.addr.bdaddr);

	if (mgmt_send(adapter->mgmt, MGMT_OP_PAIR_DEVICE, adapter->index,
				sizeof(cp), &cp, pair_device_complete, NULL,
				NULL) == 0)
		return false;

	send_bond_state_change(&cp.addr.bdaddr, HAL_STATUS_SUCCESS,
						HAL_BOND_STATE_BONDING);

	return true;
}

static bool cancel_bond(void *buf, uint16_t len)
{
	struct hal_cmd_cancel_bond *cmd = buf;
	struct mgmt_addr_info cp;

	cp.type = BDADDR_BREDR;
	android2bdaddr(cmd->bdaddr, &cp.bdaddr);

	return mgmt_reply(adapter->mgmt, MGMT_OP_CANCEL_PAIR_DEVICE,
				adapter->index, sizeof(cp), &cp, NULL, NULL,
				NULL) > 0;
}

static void unpair_device_complete(uint8_t status, uint16_t length,
					const void *param, void *user_data)
{
	const struct mgmt_rp_unpair_device *rp = param;

	DBG("status %u", status);

	if (status != MGMT_STATUS_SUCCESS)
		return;

	send_bond_state_change(&rp->addr.bdaddr, HAL_STATUS_SUCCESS,
							HAL_BOND_STATE_NONE);
}

static bool remove_bond(void *buf, uint16_t len)
{
	struct hal_cmd_remove_bond *cmd = buf;
	struct mgmt_cp_unpair_device cp;

	cp.disconnect = 1;
	cp.addr.type = BDADDR_BREDR;
	android2bdaddr(cmd->bdaddr, &cp.addr.bdaddr);

	return mgmt_send(adapter->mgmt, MGMT_OP_UNPAIR_DEVICE,
				adapter->index, sizeof(cp), &cp,
				unpair_device_complete, NULL, NULL) > 0;
}

static uint8_t pin_reply(void *buf, uint16_t len)
{
	struct hal_cmd_pin_reply *cmd = buf;
	bdaddr_t bdaddr;
	char addr[18];

	android2bdaddr(cmd->bdaddr, &bdaddr);
	ba2str(&bdaddr, addr);

	DBG("%s accept %u pin_len %u", addr, cmd->accept, cmd->pin_len);

	if (!cmd->accept && cmd->pin_len)
		return HAL_STATUS_INVALID;

	if (cmd->accept) {
		struct mgmt_cp_pin_code_reply rp;

		memset(&rp, 0, sizeof(rp));

		bacpy(&rp.addr.bdaddr, &bdaddr);
		rp.addr.type = BDADDR_BREDR;
		rp.pin_len = cmd->pin_len;
		memcpy(rp.pin_code, cmd->pin_code, rp.pin_len);

		if (mgmt_reply(adapter->mgmt, MGMT_OP_PIN_CODE_REPLY,
					adapter->index, sizeof(rp), &rp,
					NULL, NULL, NULL) == 0)
			return HAL_STATUS_FAILED;
	} else {
		struct mgmt_cp_pin_code_neg_reply rp;

		bacpy(&rp.addr.bdaddr, &bdaddr);
		rp.addr.type = BDADDR_BREDR;

		if (mgmt_reply(adapter->mgmt, MGMT_OP_PIN_CODE_NEG_REPLY,
					adapter->index, sizeof(rp), &rp,
					NULL, NULL, NULL) == 0)
			return HAL_STATUS_FAILED;
	}

	return HAL_STATUS_SUCCESS;
}

static uint8_t user_confirm_reply(const bdaddr_t *bdaddr, bool accept)
{
	struct mgmt_addr_info cp;
	uint16_t opcode;

	if (accept)
		opcode = MGMT_OP_USER_CONFIRM_REPLY;
	else
		opcode = MGMT_OP_USER_CONFIRM_NEG_REPLY;

	bacpy(&cp.bdaddr, bdaddr);
	cp.type = BDADDR_BREDR;

	if (mgmt_reply(adapter->mgmt, opcode, adapter->index, sizeof(cp), &cp,
							NULL, NULL, NULL) > 0)
		return HAL_STATUS_SUCCESS;

	return HAL_STATUS_FAILED;
}

static uint8_t user_passkey_reply(const bdaddr_t *bdaddr, bool accept,
							uint32_t passkey)
{
	unsigned int id;

	if (accept) {
		struct mgmt_cp_user_passkey_reply cp;

		memset(&cp, 0, sizeof(cp));
		bacpy(&cp.addr.bdaddr, bdaddr);
		cp.addr.type = BDADDR_BREDR;
		cp.passkey = htobl(passkey);

		id = mgmt_reply(adapter->mgmt, MGMT_OP_USER_PASSKEY_REPLY,
					adapter->index, sizeof(cp), &cp,
					NULL, NULL, NULL);
	} else {
		struct mgmt_cp_user_passkey_neg_reply cp;

		memset(&cp, 0, sizeof(cp));
		bacpy(&cp.addr.bdaddr, bdaddr);
		cp.addr.type = BDADDR_BREDR;

		id = mgmt_reply(adapter->mgmt, MGMT_OP_USER_PASSKEY_NEG_REPLY,
					adapter->index, sizeof(cp), &cp,
					NULL, NULL, NULL);
	}

	if (id == 0)
		return HAL_STATUS_FAILED;

	return HAL_STATUS_SUCCESS;
}

static uint8_t ssp_reply(void *buf, uint16_t len)
{
	struct hal_cmd_ssp_reply *cmd = buf;
	uint8_t status;
	bdaddr_t bdaddr;
	char addr[18];

	/* TODO should parameters sanity be verified here? */

	android2bdaddr(cmd->bdaddr, &bdaddr);
	ba2str(&bdaddr, addr);

	DBG("%s variant %u accept %u", addr, cmd->ssp_variant, cmd->accept);

	switch (cmd->ssp_variant) {
	case HAL_SSP_VARIANT_CONFIRM:
	case HAL_SSP_VARIANT_CONSENT:
		status = user_confirm_reply(&bdaddr, cmd->accept);
		break;
	case HAL_SSP_VARIANT_ENTRY:
		status = user_passkey_reply(&bdaddr, cmd->accept,
								cmd->passkey);
		break;
	case HAL_SSP_VARIANT_NOTIF:
		status = HAL_STATUS_SUCCESS;
		break;
	default:
		status = HAL_STATUS_INVALID;
		break;
	}

	return status;
}

void bt_adapter_handle_cmd(GIOChannel *io, uint8_t opcode, void *buf,
								uint16_t len)
{
	uint8_t status = HAL_STATUS_FAILED;

	switch (opcode) {
	case HAL_OP_ENABLE:
		if (adapter->current_settings & MGMT_SETTING_POWERED) {
			status = HAL_STATUS_DONE;
			goto error;
		}

		if (!set_mode(MGMT_OP_SET_POWERED, 0x01))
			goto error;

		break;
	case HAL_OP_DISABLE:
		if (!(adapter->current_settings & MGMT_SETTING_POWERED)) {
			status = HAL_STATUS_DONE;
			goto error;
		}

		if (!set_mode(MGMT_OP_SET_POWERED, 0x00))
			goto error;

		break;
	case HAL_OP_GET_ADAPTER_PROP:
		if (!get_property(buf, len))
			goto error;

		break;
	case HAL_OP_SET_ADAPTER_PROP:
		status = set_property(buf, len);
		if (status != HAL_STATUS_SUCCESS)
			goto error;

		break;
	case HAL_OP_CREATE_BOND:
		if (!create_bond(buf, len))
			goto error;

		break;
	case HAL_OP_CANCEL_BOND:
		if (!cancel_bond(buf, len))
			goto error;

		break;
	case HAL_OP_REMOVE_BOND:
		if (!remove_bond(buf, len))
			goto error;

		break;
	case HAL_OP_PIN_REPLY:
		status = pin_reply(buf, len);
		if (status != HAL_STATUS_SUCCESS)
			goto error;

		break;
	case HAL_OP_SSP_REPLY:
		status = ssp_reply(buf, len);
		if (status != HAL_STATUS_SUCCESS)
			goto error;

		break;
	default:
		DBG("Unhandled command, opcode 0x%x", opcode);
		goto error;
	}

	ipc_send(io, HAL_SERVICE_ID_BLUETOOTH, opcode, 0, NULL, -1);
	return;

error:
	ipc_send_rsp(io, HAL_SERVICE_ID_BLUETOOTH, status);
}

const bdaddr_t *bt_adapter_get_address(void)
{
	return &adapter->bdaddr;
}

bool bt_adapter_register(GIOChannel *io)
{
	DBG("");

	notification_io = g_io_channel_ref(io);

	return true;
}

void bt_adapter_unregister(void)
{
	DBG("");

	g_io_channel_unref(notification_io);
	notification_io = NULL;
}
