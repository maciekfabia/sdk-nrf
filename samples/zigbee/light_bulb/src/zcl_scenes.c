/**
 * Copyright (c) 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "zcl_scenes.h"
#include <zb_nrf_platform.h>
#include <logging/log.h>
#include <settings/settings.h>

#define LOG_MODULE_NAME zcl_scenes
LOG_MODULE_REGISTER(LOG_MODULE_NAME);


static zb_uint8_t scene_table_get_entry(zb_uint16_t group_id, zb_uint8_t scene_id);
static void scene_table_remove_entries_by_group(zb_uint16_t group_id);
static void scene_table_init(void);

typedef struct
{
    zb_bool_t  has_on_off;
    zb_uint8_t on_off;
} zb_zcl_scenes_fieldset_data_on_off_t;

typedef struct
{
    zb_bool_t  has_current_level;
    zb_uint8_t current_level;
} zb_zcl_scenes_fieldset_data_level_control_t;

typedef struct
{
    zb_bool_t  has_current_position_lift_percentage;
    zb_uint8_t current_position_lift_percentage;
    zb_bool_t  has_current_position_tilt_percentage;
    zb_uint8_t current_position_tilt_percentage;
} zb_zcl_scenes_fieldset_data_window_covering_t;


typedef struct scene_table_on_off_entry_s
{
    zb_zcl_scene_table_record_fixed_t             common;
    zb_zcl_scenes_fieldset_data_on_off_t          on_off;
    zb_zcl_scenes_fieldset_data_level_control_t   level_control;
    zb_zcl_scenes_fieldset_data_window_covering_t window_covering;
} scene_table_on_off_entry_t;

scene_table_on_off_entry_t scenes_table[ZCL_SCENES_TABLE_SIZE];

typedef struct resp_info_s
{
    zb_zcl_parsed_hdr_t cmd_info;
    zb_zcl_scenes_view_scene_req_t view_scene_req;
    zb_zcl_scenes_get_scene_membership_req_t get_scene_membership_req;
} resp_info_t;

resp_info_t resp_info;

static int scenes_table_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    int rc;

    if (settings_name_steq(name, "scenes_table", &next) && !next) {
        if (len != sizeof(scenes_table)) {
            return -EINVAL;
        }

        rc = read_cb(cb_arg, scenes_table, sizeof(scenes_table));
        if (rc >= 0) {
            return 0;
        }

        return rc;
    }

    return -ENOENT;
}

static void scenes_table_save()
{
    settings_save_one("scenes/scenes_table", scenes_table, sizeof(scenes_table));
}

struct settings_handler scenes_conf = {
    .name = "scenes",
    .h_set = scenes_table_set
};

static zb_bool_t has_cluster(zb_uint16_t cluster_id)
{
    return (get_endpoint_by_cluster(cluster_id, ZB_ZCL_CLUSTER_SERVER_ROLE) == ZCL_SCENES_ENDPOINT ? ZB_TRUE : ZB_FALSE);
}

static zb_bool_t add_fieldset(zb_zcl_scenes_fieldset_common_t * fieldset, scene_table_on_off_entry_t * p_entry)
{
    zb_uint8_t *fs_data_ptr;
    if (fieldset->cluster_id == ZB_ZCL_CLUSTER_ID_ON_OFF &&
        fieldset->fieldset_length >= 1 &&
        has_cluster(ZB_ZCL_CLUSTER_ID_ON_OFF))
    {
        fs_data_ptr = (zb_uint8_t*)fieldset + sizeof(zb_zcl_scenes_fieldset_common_t);
        p_entry->on_off.has_on_off = ZB_TRUE;
        p_entry->on_off.on_off = *fs_data_ptr;

        LOG_INF("Add fieldset: cluster_id=0x%x, length=%d, on/off=%d", fieldset->cluster_id, fieldset->fieldset_length, p_entry->on_off.on_off);
        return ZB_TRUE;
    }
    else if (fieldset->cluster_id == ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL &&
             fieldset->fieldset_length >= 1 &&
             has_cluster(ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL))
    {
        fs_data_ptr = (zb_uint8_t*)fieldset + sizeof(zb_zcl_scenes_fieldset_common_t);
        p_entry->level_control.has_current_level = ZB_TRUE;
        p_entry->level_control.current_level = *fs_data_ptr;

        LOG_INF("Add fieldset: cluster_id=0x%x, length=%d, level=%d", fieldset->cluster_id, fieldset->fieldset_length, p_entry->level_control.current_level);
        return ZB_TRUE;
    }
    else if (fieldset->cluster_id == ZB_ZCL_CLUSTER_ID_WINDOW_COVERING &&
             fieldset->fieldset_length >= 1 &&
             has_cluster(ZB_ZCL_CLUSTER_ID_WINDOW_COVERING))
    {
        fs_data_ptr = (zb_uint8_t*)fieldset + sizeof(zb_zcl_scenes_fieldset_common_t);
        p_entry->window_covering.has_current_position_lift_percentage = ZB_TRUE;
        p_entry->window_covering.current_position_lift_percentage = *fs_data_ptr;
        if (fieldset->fieldset_length >= 2)
        {
            p_entry->window_covering.has_current_position_tilt_percentage = ZB_TRUE;
            p_entry->window_covering.current_position_tilt_percentage = *(fs_data_ptr + 1);
        }

        LOG_INF("Add fieldset: cluster_id=0x%x, length=%d, lift=%d", fieldset->cluster_id, fieldset->fieldset_length, p_entry->window_covering.current_position_lift_percentage);
        return ZB_TRUE;
    }
    else
    {
        LOG_INF("Ignore fieldset: cluster_id = 0x%x, length = %d", fieldset->cluster_id, fieldset->fieldset_length);
        return ZB_FALSE;
    }
}

static zb_uint8_t * dump_fieldsets(scene_table_on_off_entry_t * p_entry, zb_uint8_t * payload_ptr)
{
    if (p_entry->on_off.has_on_off == ZB_TRUE)
    {
        LOG_INF("Append On/Off fieldset");

        /* Extention set: Cluster ID = On/Off */
        ZB_ZCL_PACKET_PUT_DATA16_VAL(payload_ptr, ZB_ZCL_CLUSTER_ID_ON_OFF);

        /* Extention set: Fieldset length = 1 */
        ZB_ZCL_PACKET_PUT_DATA8(payload_ptr, 1);

        /* Extention set: On/Off state */
        ZB_ZCL_PACKET_PUT_DATA8(payload_ptr, p_entry->on_off.on_off);
    }

    if (p_entry->level_control.has_current_level == ZB_TRUE)
    {
        LOG_INF("Append level control fieldset");

        /* Extention set: Cluster ID = Level Control */
        ZB_ZCL_PACKET_PUT_DATA16_VAL(payload_ptr, ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL);

        /* Extention set: Fieldset length = 1 */
        ZB_ZCL_PACKET_PUT_DATA8(payload_ptr, 1);

        /* Extention set: current_level state */
        ZB_ZCL_PACKET_PUT_DATA8(payload_ptr, p_entry->level_control.current_level);
    }

    if (p_entry->window_covering.has_current_position_lift_percentage == ZB_TRUE)
    {
        LOG_INF("Append window covering fieldset");

        /* Extention set: Cluster ID = Window covering */
        ZB_ZCL_PACKET_PUT_DATA16_VAL(payload_ptr, ZB_ZCL_CLUSTER_ID_WINDOW_COVERING);

        /* Extention set: Fieldset length = 1 or 2*/
        if (p_entry->window_covering.has_current_position_tilt_percentage == ZB_TRUE)
        {
            ZB_ZCL_PACKET_PUT_DATA8(payload_ptr, 2);
        }
        else
        {
            ZB_ZCL_PACKET_PUT_DATA8(payload_ptr, 1);
        }
        /* Extention set: current_position_lift_percentage state */
        ZB_ZCL_PACKET_PUT_DATA8(payload_ptr, p_entry->window_covering.current_position_lift_percentage);

        if (p_entry->window_covering.has_current_position_tilt_percentage == ZB_TRUE)
        {
            /* Extention set: current_position_tilt_percentage state */
            ZB_ZCL_PACKET_PUT_DATA8(payload_ptr, p_entry->window_covering.current_position_tilt_percentage);
        }
    }

    /* Pass the updated data pointer. */
    return payload_ptr;
}

static zb_ret_t get_on_off_value(zb_uint8_t * on_off)
{
    zb_zcl_attr_t *attr_desc = zb_zcl_get_attr_desc_a(
        ZCL_SCENES_ENDPOINT,
        ZB_ZCL_CLUSTER_ID_ON_OFF,
        ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);

    if (attr_desc != NULL)
    {
        *on_off = (zb_bool_t)ZB_ZCL_GET_ATTRIBUTE_VAL_8(attr_desc);
        return RET_OK;
    }

    return RET_ERROR;
}

static zb_ret_t get_current_level_value(zb_uint8_t * current_level)
{
    zb_zcl_attr_t *attr_desc = zb_zcl_get_attr_desc_a(
        ZCL_SCENES_ENDPOINT,
        ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
        ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID);

    if (attr_desc != NULL)
    {
        *current_level = ZB_ZCL_GET_ATTRIBUTE_VAL_8(attr_desc);
        return RET_OK;
    }

    return RET_ERROR;
}

static zb_ret_t get_current_lift_value(zb_uint8_t * percentage)
{
    zb_zcl_attr_t *attr_desc = zb_zcl_get_attr_desc_a(
        ZCL_SCENES_ENDPOINT,
        ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
        ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID);

    if (attr_desc != NULL)
    {
        *percentage = ZB_ZCL_GET_ATTRIBUTE_VAL_8(attr_desc);
        return RET_OK;
    }

    return RET_ERROR;
}

static zb_ret_t get_current_tilt_value(zb_uint8_t * percentage)
{
    zb_zcl_attr_t *attr_desc = zb_zcl_get_attr_desc_a(
        ZCL_SCENES_ENDPOINT,
        ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
        ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_PERCENTAGE_ID);

    if (attr_desc != NULL)
    {
        *percentage = ZB_ZCL_GET_ATTRIBUTE_VAL_8(attr_desc);
        return RET_OK;
    }

    return RET_ERROR;
}

static void save_state_as_scene(scene_table_on_off_entry_t * p_entry)
{
    if (has_cluster(ZB_ZCL_CLUSTER_ID_ON_OFF) &&
        get_on_off_value(&p_entry->on_off.on_off) == RET_OK)
    {
        LOG_INF("Save On/Off state inside scene table");
        p_entry->on_off.has_on_off = ZB_TRUE;
    }
    if (has_cluster(ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL) &&
        get_current_level_value(&p_entry->level_control.current_level) == RET_OK)
    {
        LOG_INF("Save level control state inside scene table");
        p_entry->level_control.has_current_level = ZB_TRUE;
    }
    if (has_cluster(ZB_ZCL_CLUSTER_ID_WINDOW_COVERING))
    {
        LOG_INF("Save window covering state inside scene table");
        if (get_current_lift_value(&p_entry->window_covering.current_position_lift_percentage) == RET_OK)
        {
            p_entry->window_covering.has_current_position_lift_percentage = ZB_TRUE;
        }
        if (get_current_tilt_value(&p_entry->window_covering.current_position_tilt_percentage) == RET_OK)
        {
            p_entry->window_covering.has_current_position_tilt_percentage = ZB_TRUE;
        }
    }
}

static void recall_scene(scene_table_on_off_entry_t * p_entry)
{
    zb_bufid_t buf = zb_buf_get_any();
    zb_zcl_attr_t *attr_desc;
    zb_ret_t result;

    if (p_entry->on_off.has_on_off)
    {
        LOG_INF("Recall On/Off state");

        attr_desc = zb_zcl_get_attr_desc_a(
            ZCL_SCENES_ENDPOINT,
            ZB_ZCL_CLUSTER_ID_ON_OFF,
            ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);

        ZB_ZCL_INVOKE_USER_APP_SET_ATTR_WITH_RESULT(
            buf,
            ZCL_SCENES_ENDPOINT,
            ZB_ZCL_CLUSTER_ID_ON_OFF,
            attr_desc,
            &p_entry->on_off.on_off,
            result
            );
    }
    if (p_entry->level_control.has_current_level)
    {
        LOG_INF("Recall level control state");

        attr_desc = zb_zcl_get_attr_desc_a(
            ZCL_SCENES_ENDPOINT,
            ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
            ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID);

        ZB_ZCL_INVOKE_USER_APP_SET_ATTR_WITH_RESULT(
            buf,
            ZCL_SCENES_ENDPOINT,
            ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
            attr_desc,
            &p_entry->level_control.current_level,
            result
            );
    }
    if (p_entry->window_covering.has_current_position_lift_percentage)
    {
        LOG_INF("Recall window covering lift state");

    attr_desc = zb_zcl_get_attr_desc_a(
        ZCL_SCENES_ENDPOINT,
        ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
        ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID);

    ZB_ZCL_INVOKE_USER_APP_SET_ATTR_WITH_RESULT(
            buf,
            ZCL_SCENES_ENDPOINT,
            ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
            attr_desc,
            &p_entry->window_covering.current_position_lift_percentage,
            result
            );
    }
    if (p_entry->window_covering.has_current_position_tilt_percentage)
    {
        LOG_INF("Recall window covering tilt state");

    attr_desc = zb_zcl_get_attr_desc_a(
        ZCL_SCENES_ENDPOINT,
        ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
        ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_PERCENTAGE_ID);

    ZB_ZCL_INVOKE_USER_APP_SET_ATTR_WITH_RESULT(
        buf,
        ZCL_SCENES_ENDPOINT,
        ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
        attr_desc,
        &p_entry->window_covering.current_position_tilt_percentage,
        result
        );
    }

    zb_buf_free(buf);
}

static void send_view_scene_resp(zb_bufid_t bufid, zb_uint16_t idx)
{
    zb_uint8_t *payload_ptr;
    zb_uint8_t view_scene_status = ZB_ZCL_STATUS_NOT_FOUND;

    LOG_INF(">> send_view_scene_resp bufid %hd idx %d", bufid, idx);

    if (idx != 0xFF &&
        scenes_table[idx].common.group_id != ZB_ZCL_SCENES_FREE_SCENE_TABLE_RECORD)
    {
        /* Scene found */
        view_scene_status = ZB_ZCL_STATUS_SUCCESS;
    }
    else if (!zb_aps_is_endpoint_in_group(
                resp_info.view_scene_req.group_id,
                ZB_ZCL_PARSED_HDR_SHORT_DATA(&resp_info.cmd_info).dst_endpoint
                )
            )
    {
        /* Not in the group */
        view_scene_status = ZB_ZCL_STATUS_INVALID_FIELD;
    }

    ZB_ZCL_SCENES_INIT_VIEW_SCENE_RES(
        bufid,
        payload_ptr,
        resp_info.cmd_info.seq_number,
        view_scene_status,
        resp_info.view_scene_req.group_id,
        resp_info.view_scene_req.scene_id);

    if (view_scene_status == ZB_ZCL_STATUS_SUCCESS)
    {
        ZB_ZCL_SCENES_ADD_TRANSITION_TIME_VIEW_SCENE_RES(
            payload_ptr,
            scenes_table[idx].common.transition_time);

        ZB_ZCL_SCENES_ADD_SCENE_NAME_VIEW_SCENE_RES(
            payload_ptr,
            scenes_table[idx].common.scene_name);

        payload_ptr = dump_fieldsets(&scenes_table[idx], payload_ptr);
    }

    ZB_ZCL_SCENES_SEND_VIEW_SCENE_RES(
        bufid,
        payload_ptr,
        ZB_ZCL_PARSED_HDR_SHORT_DATA(&resp_info.cmd_info).source.u.short_addr,
        ZB_ZCL_PARSED_HDR_SHORT_DATA(&resp_info.cmd_info).src_endpoint,
        ZB_ZCL_PARSED_HDR_SHORT_DATA(&resp_info.cmd_info).dst_endpoint,
        resp_info.cmd_info.profile_id,
        NULL);

    LOG_INF("<< send_view_scene_resp");
}

static void send_get_scene_membership_resp(zb_bufid_t bufid)
{
    zb_uint8_t * payload_ptr;
    zb_uint8_t * capacity_ptr;
    zb_uint8_t * scene_count_ptr;

    LOG_INF(">> send_get_scene_membership_resp bufid %hd", bufid);

    if (!zb_aps_is_endpoint_in_group(
            resp_info.get_scene_membership_req.group_id,
            ZB_ZCL_PARSED_HDR_SHORT_DATA(&resp_info.cmd_info).dst_endpoint
            )
        )
    {
        /* Not in the group */
        ZB_ZCL_SCENES_INIT_GET_SCENE_MEMBERSHIP_RES(
            bufid,
            payload_ptr,
            resp_info.cmd_info.seq_number,
            capacity_ptr,
            ZB_ZCL_STATUS_INVALID_FIELD,
            ZB_ZCL_SCENES_CAPACITY_UNKNOWN,
            resp_info.get_scene_membership_req.group_id);
    }
    else
    {
        zb_uint8_t i = 0;

        ZB_ZCL_SCENES_INIT_GET_SCENE_MEMBERSHIP_RES(
            bufid,
            payload_ptr,
            resp_info.cmd_info.seq_number,
            capacity_ptr,
            ZB_ZCL_STATUS_SUCCESS,
            0,
            resp_info.get_scene_membership_req.group_id);

        scene_count_ptr = payload_ptr;
        ZB_ZCL_SCENES_ADD_SCENE_COUNT_GET_SCENE_MEMBERSHIP_RES(payload_ptr, 0);

        while (i < ZCL_SCENES_TABLE_SIZE)
        {
            if (scenes_table[i].common.group_id == resp_info.get_scene_membership_req.group_id)
            {
                /* Add to payload */
                LOG_INF("add scene_id %hd", scenes_table[i].common.scene_id);
                ++(*scene_count_ptr);
                ZB_ZCL_SCENES_ADD_SCENE_ID_GET_SCENE_MEMBERSHIP_RES(
                    payload_ptr,
                    scenes_table[i].common.scene_id);
            }
            else if (scenes_table[i].common.group_id == ZB_ZCL_SCENES_FREE_SCENE_TABLE_RECORD)
            {
                LOG_INF("add capacity num");
                ++(*capacity_ptr);
            }
            ++i;
        }
    }

    ZB_ZCL_SCENES_SEND_GET_SCENE_MEMBERSHIP_RES(
        bufid,
        payload_ptr,
        ZB_ZCL_PARSED_HDR_SHORT_DATA(&resp_info.cmd_info).source.u.short_addr,
        ZB_ZCL_PARSED_HDR_SHORT_DATA(&resp_info.cmd_info).src_endpoint,
        ZB_ZCL_PARSED_HDR_SHORT_DATA(&resp_info.cmd_info).dst_endpoint,
        resp_info.cmd_info.profile_id,
        NULL);

    LOG_INF("<< send_get_scene_membership_resp");
}

static void scene_table_init(void)
{
    zb_uint8_t i = 0;
    memset(scenes_table, 0, sizeof(scenes_table));
    while (i < ZCL_SCENES_TABLE_SIZE)
    {
        scenes_table[i].common.group_id = ZB_ZCL_SCENES_FREE_SCENE_TABLE_RECORD;
        ++i;
    }
}

static zb_uint8_t scene_table_get_entry(zb_uint16_t group_id, zb_uint8_t scene_id)
{
    zb_uint8_t i = 0;
    zb_uint8_t idx = 0xFF, free_idx = 0xFF;

    while (i < ZCL_SCENES_TABLE_SIZE)
    {
        if (scenes_table[i].common.group_id == group_id &&
            scenes_table[i].common.scene_id == scene_id)
        {
            idx = i;
            break;
        }
        else if (free_idx == 0xFF &&
                 scenes_table[i].common.group_id == ZB_ZCL_SCENES_FREE_SCENE_TABLE_RECORD)
        {
            /* Remember free index */
            free_idx = i;
        }
        ++i;
    }

    return ((idx != 0xFF) ? idx : free_idx);
}

static void scene_table_remove_entries_by_group(zb_uint16_t group_id)
{
    zb_uint8_t i = 0;

    LOG_INF(">> scene_table_remove_entries_by_group: group_id 0x%x", group_id);
    while (i < ZCL_SCENES_TABLE_SIZE)
    {
        if (scenes_table[i].common.group_id == group_id)
        {
            LOG_INF("removing scene: entry idx %hd", i);
            memset(&scenes_table[i], 0, sizeof(scenes_table[i]));
            scenes_table[i].common.group_id = ZB_ZCL_SCENES_FREE_SCENE_TABLE_RECORD;
        }
        ++i;
    }
    LOG_INF("<< scene_table_remove_entries_by_group");
}

static zb_ret_t get_scene_valid_value(zb_bool_t * scene_valid)
{
    zb_zcl_attr_t *attr_desc = zb_zcl_get_attr_desc_a(
        ZCL_SCENES_ENDPOINT,
        ZB_ZCL_CLUSTER_ID_SCENES,
        ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ZCL_ATTR_SCENES_SCENE_VALID_ID);

    if (attr_desc != NULL)
    {
        *scene_valid = (zb_bool_t)ZB_ZCL_GET_ATTRIBUTE_VAL_8(attr_desc);
        return RET_OK;
    }

    return RET_ERROR;
}

static zb_ret_t set_scene_valid_value(zb_bool_t scene_valid)
{
    zb_zcl_attr_t *attr_desc = zb_zcl_get_attr_desc_a(
        ZCL_SCENES_ENDPOINT,
        ZB_ZCL_CLUSTER_ID_SCENES,
        ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ZCL_ATTR_SCENES_SCENE_VALID_ID);

    if (attr_desc != NULL)
    {
        ZB_ZCL_SET_DIRECTLY_ATTR_VAL8(attr_desc, scene_valid);
        return RET_OK;
    }

    return RET_ERROR;
}

static zb_ret_t get_current_scene_scene_id_value(zb_uint8_t * scene_id)
{
    zb_zcl_attr_t *attr_desc = zb_zcl_get_attr_desc_a(
        ZCL_SCENES_ENDPOINT,
        ZB_ZCL_CLUSTER_ID_SCENES,
        ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ZCL_ATTR_SCENES_CURRENT_SCENE_ID);

    if (attr_desc != NULL)
    {
        *scene_id = (zb_bool_t)ZB_ZCL_GET_ATTRIBUTE_VAL_8(attr_desc);
        return RET_OK;
    }

    return RET_ERROR;
}

static zb_ret_t get_current_scene_group_id_value(zb_uint16_t * group_id)
{
    zb_zcl_attr_t *attr_desc = zb_zcl_get_attr_desc_a(
        ZCL_SCENES_ENDPOINT,
        ZB_ZCL_CLUSTER_ID_SCENES,
        ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ZCL_ATTR_SCENES_CURRENT_GROUP_ID);

    if (attr_desc != NULL)
    {
        *group_id = (zb_bool_t)ZB_ZCL_GET_ATTRIBUTE_VAL_8(attr_desc);
        return RET_OK;
    }

    return RET_ERROR;
}

static void update_scene_valid_value(void)
{
    /* ZBOSS stack automatically sets the scene_valid attribute to true after
     * scene recall, but isd unable to set it back to false if the device state
     * has changed.
     */
    zb_bool_t scene_valid = ZB_FALSE;
    zb_uint8_t scene_id = 0xFF;
    zb_uint16_t group_id = ZB_ZCL_SCENES_FREE_SCENE_TABLE_RECORD;

    if (get_scene_valid_value(&scene_valid) == RET_OK &&
        get_current_scene_scene_id_value(&scene_id) == RET_OK &&
        get_current_scene_group_id_value(&group_id) == RET_OK &&
        scene_valid == ZB_TRUE)
    {
        /* Verify if scene_valid should be reset. */
        zb_uint8_t idx = scene_table_get_entry(group_id, scene_id);
        if (group_id == ZB_ZCL_SCENES_FREE_SCENE_TABLE_RECORD ||
            scene_id < ZCL_SCENES_TABLE_SIZE ||
            idx == ZB_ZCL_SCENES_FREE_SCENE_TABLE_RECORD)
        {
            (void)set_scene_valid_value(ZB_FALSE);
            return;
        }

        if (scenes_table[idx].on_off.has_on_off)
        {
            zb_uint8_t on_off;

            (void)get_on_off_value(&on_off);
            if (on_off != scenes_table[idx].on_off.on_off)
            {
                (void)set_scene_valid_value(ZB_FALSE);
                return;
            }
        }

        if (scenes_table[idx].level_control.has_current_level)
        {
            zb_uint8_t current_level;

            (void)get_current_level_value(&current_level);
            if (current_level != scenes_table[idx].level_control.current_level)
            {
                (void)set_scene_valid_value(ZB_FALSE);
                return;
            }
        }

        if (scenes_table[idx].window_covering.has_current_position_lift_percentage)
        {
            zb_uint8_t lift;
            (void)get_current_lift_value(&lift);
            if (lift != scenes_table[idx].window_covering.current_position_lift_percentage)
            {
                (void)set_scene_valid_value(ZB_FALSE);
                return;
            }
        }
        if (scenes_table[idx].window_covering.has_current_position_tilt_percentage)
        {
            zb_uint8_t tilt;
            (void)get_current_lift_value(&tilt);
            if (tilt != scenes_table[idx].window_covering.current_position_tilt_percentage)
            {
                (void)set_scene_valid_value(ZB_FALSE);
                return;
            }
        }
    }
}

void zcl_scenes_init(void)
{
    scene_table_init();
    settings_register(&scenes_conf);
}

zb_bool_t zcl_scenes_cb(zb_bufid_t bufid)
{
    zb_zcl_device_callback_param_t *device_cb_param = ZB_BUF_GET_PARAM(bufid, zb_zcl_device_callback_param_t);
    LOG_INF("> zcl_scenes_cb bufid %hd id %hd", bufid, device_cb_param->device_cb_id);

    update_scene_valid_value();

    switch (device_cb_param->device_cb_id)
    {
        case ZB_ZCL_SCENES_ADD_SCENE_CB_ID:
        {
            const zb_zcl_scenes_add_scene_req_t *add_scene_req = ZB_ZCL_DEVICE_CMD_PARAM_IN_GET(bufid, zb_zcl_scenes_add_scene_req_t);
            zb_uint8_t idx = 0xFF;
            zb_uint8_t *add_scene_status = ZB_ZCL_DEVICE_CMD_PARAM_OUT_GET(bufid, zb_uint8_t);

            LOG_INF("ZB_ZCL_SCENES_ADD_SCENE_CB_ID: group_id 0x%x scene_id 0x%hd transition_time %d",
                add_scene_req->group_id,
                add_scene_req->scene_id,
                add_scene_req->transition_time);

            *add_scene_status = ZB_ZCL_STATUS_INVALID_FIELD;
            idx = scene_table_get_entry(add_scene_req->group_id, add_scene_req->scene_id);

            if (idx != 0xFF)
            {
                zb_zcl_scenes_fieldset_common_t *fieldset;
                zb_uint8_t fs_content_length;

                if (scenes_table[idx].common.group_id != ZB_ZCL_SCENES_FREE_SCENE_TABLE_RECORD)
                {
                    /* Indicate that we overwriting existing record */
                    device_cb_param->status = RET_ALREADY_EXISTS;
                }

                zb_bool_t empty_entry = ZB_TRUE;
                ZB_ZCL_SCENES_GET_ADD_SCENE_REQ_NEXT_FIELDSET_DESC(bufid,
                                                                   fieldset,
                                                                   fs_content_length);
                while (fieldset)
                {
                    if (add_fieldset(fieldset, &scenes_table[idx]) == ZB_TRUE)
                    {
                        empty_entry = ZB_FALSE;
                    }
                    ZB_ZCL_SCENES_GET_ADD_SCENE_REQ_NEXT_FIELDSET_DESC(bufid,
                                                                       fieldset,
                                                                       fs_content_length);
                }
                if (empty_entry == ZB_FALSE)
                {
                    /* Store this scene */
                    scenes_table[idx].common.group_id = add_scene_req->group_id;
                    scenes_table[idx].common.scene_id = add_scene_req->scene_id;
                    scenes_table[idx].common.transition_time = add_scene_req->transition_time;
                    *add_scene_status = ZB_ZCL_STATUS_SUCCESS;
                    scenes_table_save();
                }
            }
            else
            {
                LOG_ERR("Unable to add scene: ZB_ZCL_STATUS_INSUFF_SPACE");
                *add_scene_status = ZB_ZCL_STATUS_INSUFF_SPACE;
            }
        }
        break;

        case ZB_ZCL_SCENES_VIEW_SCENE_CB_ID:
        {
            const zb_zcl_scenes_view_scene_req_t *view_scene_req = ZB_ZCL_DEVICE_CMD_PARAM_IN_GET(bufid, zb_zcl_scenes_view_scene_req_t);
            const zb_zcl_parsed_hdr_t *in_cmd_info = ZB_ZCL_DEVICE_CMD_PARAM_CMD_INFO(bufid);
            zb_uint8_t idx = 0xFF;

            LOG_INF("ZB_ZCL_SCENES_VIEW_SCENE_CB_ID: group_id 0x%x scene_id 0x%hd",
                view_scene_req->group_id,
                view_scene_req->scene_id);

            idx = scene_table_get_entry(view_scene_req->group_id, view_scene_req->scene_id);
            LOG_INF("Scene table entry index: %d", idx);

            /* Send View Scene Response */
            ZB_MEMCPY(&resp_info.cmd_info, in_cmd_info, sizeof(zb_zcl_parsed_hdr_t));
            ZB_MEMCPY(&resp_info.view_scene_req, view_scene_req, sizeof(zb_zcl_scenes_view_scene_req_t));
            zb_buf_get_out_delayed_ext(send_view_scene_resp, idx, 0);
        }
        break;

        case ZB_ZCL_SCENES_REMOVE_SCENE_CB_ID:
        {
            const zb_zcl_scenes_remove_scene_req_t *remove_scene_req = ZB_ZCL_DEVICE_CMD_PARAM_IN_GET(bufid, zb_zcl_scenes_remove_scene_req_t);
            zb_uint8_t idx = 0xFF;
            zb_uint8_t *remove_scene_status = ZB_ZCL_DEVICE_CMD_PARAM_OUT_GET(bufid, zb_uint8_t);
            const zb_zcl_parsed_hdr_t *in_cmd_info = ZB_ZCL_DEVICE_CMD_PARAM_CMD_INFO(bufid);

            LOG_INF("ZB_ZCL_SCENES_REMOVE_SCENE_CB_ID: group_id 0x%x scene_id 0x%hd",
                remove_scene_req->group_id,
                remove_scene_req->scene_id);

            *remove_scene_status = ZB_ZCL_STATUS_NOT_FOUND;
            idx = scene_table_get_entry(remove_scene_req->group_id, remove_scene_req->scene_id);

            if (idx != 0xFF &&
                scenes_table[idx].common.group_id != ZB_ZCL_SCENES_FREE_SCENE_TABLE_RECORD)
            {
                /* Remove this entry */
                memset(&scenes_table[idx], 0, sizeof(scenes_table[idx]));
                scenes_table[idx].common.group_id = ZB_ZCL_SCENES_FREE_SCENE_TABLE_RECORD;
                LOG_INF("removing scene: entry idx %hd", idx);
                *remove_scene_status = ZB_ZCL_STATUS_SUCCESS;
                scenes_table_save();
            }
            else if (!zb_aps_is_endpoint_in_group(
                       remove_scene_req->group_id,
                       ZB_ZCL_PARSED_HDR_SHORT_DATA(in_cmd_info).dst_endpoint))
            {
                *remove_scene_status = ZB_ZCL_STATUS_INVALID_FIELD;
            }
        }
        break;

        case ZB_ZCL_SCENES_REMOVE_ALL_SCENES_CB_ID:
        {
            const zb_zcl_scenes_remove_all_scenes_req_t *remove_all_scenes_req = ZB_ZCL_DEVICE_CMD_PARAM_IN_GET(bufid, zb_zcl_scenes_remove_all_scenes_req_t);
            zb_uint8_t *remove_all_scenes_status = ZB_ZCL_DEVICE_CMD_PARAM_OUT_GET(bufid, zb_uint8_t);
            const zb_zcl_parsed_hdr_t *in_cmd_info = ZB_ZCL_DEVICE_CMD_PARAM_CMD_INFO(bufid);

            LOG_INF("ZB_ZCL_SCENES_REMOVE_ALL_SCENES_CB_ID: group_id 0x%x",
                remove_all_scenes_req->group_id);

            if (!zb_aps_is_endpoint_in_group(
                    remove_all_scenes_req->group_id,
                    ZB_ZCL_PARSED_HDR_SHORT_DATA(in_cmd_info).dst_endpoint))
            {
                *remove_all_scenes_status = ZB_ZCL_STATUS_INVALID_FIELD;
            }
            else
            {
                scene_table_remove_entries_by_group(remove_all_scenes_req->group_id);
                *remove_all_scenes_status = ZB_ZCL_STATUS_SUCCESS;
                scenes_table_save();
            }
        }
        break;

        case ZB_ZCL_SCENES_STORE_SCENE_CB_ID:
        {
            const zb_zcl_scenes_store_scene_req_t *store_scene_req = ZB_ZCL_DEVICE_CMD_PARAM_IN_GET(bufid, zb_zcl_scenes_store_scene_req_t);
            zb_uint8_t idx = 0xFF;
            zb_uint8_t *store_scene_status = ZB_ZCL_DEVICE_CMD_PARAM_OUT_GET(bufid, zb_uint8_t);
            const zb_zcl_parsed_hdr_t *in_cmd_info = ZB_ZCL_DEVICE_CMD_PARAM_CMD_INFO(bufid);

            LOG_INF("ZB_ZCL_SCENES_STORE_SCENE_CB_ID: group_id 0x%x scene_id 0x%hd",
                store_scene_req->group_id,
                store_scene_req->scene_id);

            if (!zb_aps_is_endpoint_in_group(
                    store_scene_req->group_id,
                    ZB_ZCL_PARSED_HDR_SHORT_DATA(in_cmd_info).dst_endpoint))
            {
                *store_scene_status = ZB_ZCL_STATUS_INVALID_FIELD;
            }
            else
            {
                idx = scene_table_get_entry(store_scene_req->group_id, store_scene_req->scene_id);

                if (idx != 0xFF)
                {
                    if (scenes_table[idx].common.group_id != ZB_ZCL_SCENES_FREE_SCENE_TABLE_RECORD)
                    {
                        /* Update existing entry with current On/Off state */
                        device_cb_param->status = RET_ALREADY_EXISTS;
                        LOG_INF("update existing scene: entry idx %hd", idx);
                    }
                    else
                    {
                        /* Create new entry with empty name and 0 transition time */
                        scenes_table[idx].common.group_id = store_scene_req->group_id;
                        scenes_table[idx].common.scene_id = store_scene_req->scene_id;
                        scenes_table[idx].common.transition_time = 0;
                        LOG_INF("create new scene: entry idx %hd", idx);
                    }
                    save_state_as_scene(&scenes_table[idx]);
                    *store_scene_status = ZB_ZCL_STATUS_SUCCESS;
                    scenes_table_save();
                }
                else
                {
                    *store_scene_status = ZB_ZCL_STATUS_INSUFF_SPACE;
                }
            }
        }
        break;

        case ZB_ZCL_SCENES_RECALL_SCENE_CB_ID:
        {
            const zb_zcl_scenes_recall_scene_req_t *recall_scene_req = ZB_ZCL_DEVICE_CMD_PARAM_IN_GET(bufid, zb_zcl_scenes_recall_scene_req_t);
            zb_uint8_t idx = 0xFF;
            zb_uint8_t *recall_scene_status = ZB_ZCL_DEVICE_CMD_PARAM_OUT_GET(bufid, zb_uint8_t);

            LOG_INF("ZB_ZCL_SCENES_RECALL_SCENE_CB_ID: group_id 0x%x scene_id 0x%hd",
                recall_scene_req->group_id,
                recall_scene_req->scene_id);

            idx = scene_table_get_entry(recall_scene_req->group_id, recall_scene_req->scene_id);

            if (idx != 0xFF &&
                scenes_table[idx].common.group_id != ZB_ZCL_SCENES_FREE_SCENE_TABLE_RECORD)
            {
                /* Recall this entry */
                recall_scene(&scenes_table[idx]);
                *recall_scene_status = ZB_ZCL_STATUS_SUCCESS;
            }
            else
            {
                *recall_scene_status = ZB_ZCL_STATUS_NOT_FOUND;
            }
        }
        break;

        case ZB_ZCL_SCENES_GET_SCENE_MEMBERSHIP_CB_ID:
        {
            const zb_zcl_scenes_get_scene_membership_req_t *get_scene_membership_req = ZB_ZCL_DEVICE_CMD_PARAM_IN_GET(bufid, zb_zcl_scenes_get_scene_membership_req_t);
            const zb_zcl_parsed_hdr_t *in_cmd_info = ZB_ZCL_DEVICE_CMD_PARAM_CMD_INFO(bufid);

            LOG_INF("ZB_ZCL_SCENES_GET_SCENE_MEMBERSHIP_CB_ID: group_id 0x%x", get_scene_membership_req->group_id);

            /* Send View Scene Response */
            ZB_MEMCPY(&resp_info.cmd_info, in_cmd_info, sizeof(zb_zcl_parsed_hdr_t));
            ZB_MEMCPY(&resp_info.get_scene_membership_req, get_scene_membership_req, sizeof(zb_zcl_scenes_get_scene_membership_req_t));
            zb_buf_get_out_delayed(send_get_scene_membership_resp);
        }
        break;

        case ZB_ZCL_SCENES_INTERNAL_REMOVE_ALL_SCENES_ALL_ENDPOINTS_CB_ID:
        {
            const zb_zcl_scenes_remove_all_scenes_req_t *remove_all_scenes_req = ZB_ZCL_DEVICE_CMD_PARAM_IN_GET(bufid, zb_zcl_scenes_remove_all_scenes_req_t);

            LOG_INF("ZB_ZCL_SCENES_INTERNAL_REMOVE_ALL_SCENES_ALL_ENDPOINTS_CB_ID: group_id 0x%x", remove_all_scenes_req->group_id);

            /* Have only one endpoint */
            scene_table_remove_entries_by_group(remove_all_scenes_req->group_id);
            scenes_table_save();
        }
        break;

        case ZB_ZCL_SCENES_INTERNAL_REMOVE_ALL_SCENES_ALL_ENDPOINTS_ALL_GROUPS_CB_ID:
        {
            scene_table_init();
            scenes_table_save();
        }
        break;

        default:
            /* Return false if the packet was not processed. */
            return ZB_FALSE;
      }

    LOG_INF("< zcl_scenes_cb %hd", device_cb_param->status);
    return ZB_TRUE;
}
