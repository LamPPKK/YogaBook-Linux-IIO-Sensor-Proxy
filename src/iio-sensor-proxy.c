/*
 * Copyright (c) 2014-2016 Bastien Nocera <hadess@hadess.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation.
 *
 */

#include <locale.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <gio/gio.h>
#include <gudev/gudev.h>
#include "drivers.h"
#include "orientation.h"

#include "iio-sensor-proxy-resources.h"

#define SENSOR_PROXY_DBUS_NAME          "net.hadess.SensorProxy"
#define SENSOR_PROXY_DBUS_PATH          "/net/hadess/SensorProxy"
#define SENSOR_PROXY_COMPASS_DBUS_PATH  "/net/hadess/SensorProxy/Compass"
#define SENSOR_PROXY_IFACE_NAME         SENSOR_PROXY_DBUS_NAME
#define SENSOR_PROXY_COMPASS_IFACE_NAME SENSOR_PROXY_DBUS_NAME ".Compass"

#define NUM_SENSOR_TYPES DRIVER_TYPE_HINGE_ANGLE + 1


#define device_type(dev) (dev->drv->type)


typedef struct {
	GMainLoop *loop;
	GUdevClient *client;
	GDBusNodeInfo *introspection_data;
	GDBusConnection *connection;
	guint name_id;
	int ret;

	GPtrArray         *devices[NUM_SENSOR_TYPES]; // SensorDevice
	GHashTable        *clients[NUM_SENSOR_TYPES]; /* key = D-Bus name, value = watch ID */

	/* Accelerometer */
	OrientationUp previous_orientation;
	AccelReadings last_accel_readings[2]; /* For display and base locations */

	/* Light */
	gdouble previous_level;
	gboolean uses_lux;

	/* Compass */
	gdouble previous_heading;

	/* Proximity */
	gboolean previous_prox_near;

	gdouble previous_hinge_angle;
	TabletMode previous_tablet_mode;
} SensorData;

static const SensorDriver * const drivers[] = {
	&iio_buffer_accel,
	&iio_poll_accel,
	&input_accel,
	&iio_buffer_light,
	&iio_poll_light,
	&hwmon_light,
	&fake_compass,
	&fake_light,
	&iio_buffer_compass,
	&iio_poll_proximity,
};

static ReadingsUpdateFunc driver_type_to_callback_func (DriverType type);

static const char *
driver_type_to_str (DriverType type)
{
	switch (type) {
	case DRIVER_TYPE_ACCEL:
		return "accelerometer";
	case DRIVER_TYPE_LIGHT:
		return "ambient light sensor";
	case DRIVER_TYPE_COMPASS:
		return "compass";
	case DRIVER_TYPE_PROXIMITY:
		return "proximity";
	case DRIVER_TYPE_HINGE_ANGLE:
		return "hinge angle";
	default:
		g_assert_not_reached ();
	}
}

static const char *
tablet_mode_to_str (TabletMode mode)
{
	switch (mode) {
	case TABLET_MODE_LAPTOP:
		return "laptop";
	case TABLET_MODE_TABLET:
		return "tablet";
	default:
		return "unknown";
	}
}

static void sensor_changes (GUdevClient *client,
			    gchar       *action,
			    GUdevDevice *device,
			    SensorData  *data);

/* Return the default device used for given sensor type */
static SensorDevice *
find_default_device_for_type (SensorData	*data,
			      DriverType  driver_type)
{
	if (!data->devices[driver_type]->len)
		return NULL;

	if (driver_type == DRIVER_TYPE_ACCEL) {
		guint i;

		for (i = 0; i < data->devices[driver_type]->len; i++) {
			SensorDevice *sensor_device =
				g_ptr_array_index (data->devices[driver_type], i);

			if (sensor_device->location == SENSOR_LOCATION_DISPLAY)
				return sensor_device;
		}
		return NULL;
	}

	return g_ptr_array_index (data->devices[driver_type], 0);
}

static SensorDevice *
find_accel_at (SensorData *data, SensorLocation location)
{
	guint i;

	for (i = 0; i < data->devices[DRIVER_TYPE_ACCEL]->len; i++) {
		SensorDevice *sensor_device =
			g_ptr_array_index (data->devices[DRIVER_TYPE_ACCEL], i);

		if (sensor_device->location == location)
			return sensor_device;
	}
	return NULL;
}

static gboolean
driver_type_exists (SensorData *data,
		    DriverType  driver_type)
{
	return find_default_device_for_type (data, driver_type) != NULL;
}

static gboolean
has_hinge_angle_sensor (SensorData *data)
{
	SensorDevice *display_accel, *base_accel;

	if (driver_type_exists (data, DRIVER_TYPE_HINGE_ANGLE)) {
		return TRUE;
	}

	display_accel = find_accel_at (data, SENSOR_LOCATION_DISPLAY);
	base_accel = find_accel_at (data, SENSOR_LOCATION_BASE);

	return display_accel && base_accel;
}

static gboolean
find_sensors (GUdevClient *client,
	      SensorData  *data)
{
	GList *devices, *input, *platform, *l;
	gboolean found = FALSE;

	devices = g_udev_client_query_by_subsystem (client, "iio");
	input = g_udev_client_query_by_subsystem (client, "input");
	platform = g_udev_client_query_by_subsystem (client, "platform");
	devices = g_list_concat (devices, input);
	devices = g_list_concat (devices, platform);

	/* Find the devices */
	for (l = devices; l != NULL; l = l->next) {
		GUdevDevice *dev = l->data;
		guint i;

		for (i = 0; i < G_N_ELEMENTS(drivers); i++) {
			SensorDriver *driver = (SensorDriver *) drivers[i];
			SensorDevice *sensor_device;

			if (driver_discover (driver, dev)) {
				g_debug ("Found device %s of type %s at %s",
					 g_udev_device_get_sysfs_path (dev),
					 driver_type_to_str (driver->type),
					 driver->driver_name);


				sensor_device = driver_open (driver, dev,
						driver_type_to_callback_func (driver->type), data);
				if (!sensor_device) {
					g_clear_object (&dev);
					continue;
				}

				g_ptr_array_add (data->devices[driver->type], sensor_device);

				found = TRUE;
				break;
			}
		}
	}

	g_list_free_full (devices, g_object_unref);
	return found;
}

static void
free_client_watch (gpointer data)
{
	guint watch_id = GPOINTER_TO_UINT (data);

	if (watch_id == 0)
		return;
	g_bus_unwatch_name (watch_id);
}

static GHashTable *
create_clients_hash_table (void)
{
	return g_hash_table_new_full (g_str_hash, g_str_equal,
				      g_free, free_client_watch);
}

typedef enum {
	PROP_HAS_ACCELEROMETER		= 1 << 0,
	PROP_ACCELEROMETER_ORIENTATION  = 1 << 1,
	PROP_HAS_AMBIENT_LIGHT		= 1 << 2,
	PROP_LIGHT_LEVEL		= 1 << 3,
	PROP_HAS_COMPASS                = 1 << 4,
	PROP_COMPASS_HEADING            = 1 << 5,
	PROP_HAS_PROXIMITY              = 1 << 6,
	PROP_PROXIMITY_NEAR             = 1 << 7,
	PROP_HAS_HINGE_ANGLE		= 1 << 8,
	PROP_HINGE_ANGLE		= 1 << 9,
	PROP_TABLET_MODE		= 1 << 10,
} PropertiesMask;

#define PROP_ALL (PROP_HAS_ACCELEROMETER | \
                  PROP_ACCELEROMETER_ORIENTATION | \
                  PROP_HAS_AMBIENT_LIGHT | \
                  PROP_LIGHT_LEVEL | \
                  PROP_HAS_PROXIMITY | \
		  PROP_PROXIMITY_NEAR | \
		  PROP_HAS_HINGE_ANGLE | \
		  PROP_HINGE_ANGLE | \
		  PROP_TABLET_MODE)
#define PROP_ALL_COMPASS (PROP_HAS_COMPASS | \
			  PROP_COMPASS_HEADING)
#define PROP_ALL_HAS (PROP_HAS_ACCELEROMETER | \
		      PROP_HAS_AMBIENT_LIGHT | \
		      PROP_HAS_PROXIMITY | \
		      PROP_HAS_COMPASS | \
		      PROP_HAS_HINGE_ANGLE)

static PropertiesMask
mask_for_sensor_type (DriverType sensor_type)
{
	switch (sensor_type) {
	case DRIVER_TYPE_ACCEL:
		return PROP_HAS_ACCELEROMETER |
			PROP_ACCELEROMETER_ORIENTATION;
	case DRIVER_TYPE_LIGHT:
		return PROP_HAS_AMBIENT_LIGHT |
			PROP_LIGHT_LEVEL;
	case DRIVER_TYPE_COMPASS:
		return PROP_HAS_COMPASS |
			PROP_COMPASS_HEADING;
	case DRIVER_TYPE_PROXIMITY:
		return PROP_HAS_PROXIMITY |
			PROP_PROXIMITY_NEAR;
	case DRIVER_TYPE_HINGE_ANGLE:
		return PROP_HAS_HINGE_ANGLE |
			PROP_HINGE_ANGLE |
			PROP_TABLET_MODE;
	default:
		g_assert_not_reached ();
	}
}

static void
send_dbus_event_for_client (SensorData     *data,
			    const char     *destination_bus_name,
			    PropertiesMask  mask)
{
	GVariantBuilder props_builder;
	GVariant *props_changed = NULL;

	g_return_if_fail (destination_bus_name != NULL);

	g_variant_builder_init (&props_builder, G_VARIANT_TYPE ("a{sv}"));

	if (mask & PROP_HAS_ACCELEROMETER) {
		gboolean has_accel;

		has_accel = driver_type_exists (data, DRIVER_TYPE_ACCEL);
		g_variant_builder_add (&props_builder, "{sv}", "HasAccelerometer",
				       g_variant_new_boolean (has_accel));

		/* Send the orientation when the device appears */
		if (has_accel)
			mask |= PROP_ACCELEROMETER_ORIENTATION;
		else
			data->previous_orientation = ORIENTATION_UNDEFINED;
	}

	if (mask & PROP_ACCELEROMETER_ORIENTATION) {
		g_variant_builder_add (&props_builder, "{sv}", "AccelerometerOrientation",
				       g_variant_new_string (orientation_to_string (data->previous_orientation)));
	}

	if (mask & PROP_HAS_AMBIENT_LIGHT) {
		gboolean has_als;

		has_als = driver_type_exists (data, DRIVER_TYPE_LIGHT);
		g_variant_builder_add (&props_builder, "{sv}", "HasAmbientLight",
				       g_variant_new_boolean (has_als));

		/* Send the light level when the device appears */
		if (has_als)
			mask |= PROP_LIGHT_LEVEL;
	}

	if (mask & PROP_LIGHT_LEVEL) {
		g_variant_builder_add (&props_builder, "{sv}", "LightLevelUnit",
				       g_variant_new_string (data->uses_lux ? "lux" : "vendor"));
		g_variant_builder_add (&props_builder, "{sv}", "LightLevel",
				       g_variant_new_double (data->previous_level));
	}

	if (mask & PROP_HAS_COMPASS) {
		gboolean has_compass;

		has_compass = driver_type_exists (data, DRIVER_TYPE_COMPASS);
		g_variant_builder_add (&props_builder, "{sv}", "HasCompass",
				       g_variant_new_boolean (has_compass));

		/* Send the heading when the device appears */
		if (has_compass)
			mask |= PROP_COMPASS_HEADING;
	}

	if (mask & PROP_COMPASS_HEADING) {
		g_variant_builder_add (&props_builder, "{sv}", "CompassHeading",
				       g_variant_new_double (data->previous_heading));
	}

	if (mask & PROP_HAS_PROXIMITY) {
		gboolean has_proximity;

		has_proximity = driver_type_exists (data, DRIVER_TYPE_PROXIMITY);
		g_variant_builder_add (&props_builder, "{sv}", "HasProximity",
				       g_variant_new_boolean (has_proximity));

		/* Send proximity information when the device appears */
		if (has_proximity)
			mask |= PROP_PROXIMITY_NEAR;
	}

	if (mask & PROP_PROXIMITY_NEAR) {
		g_variant_builder_add (&props_builder, "{sv}", "ProximityNear",
				       g_variant_new_boolean (data->previous_prox_near));
	}

	if (mask & PROP_HAS_HINGE_ANGLE) {
		gboolean has_hinge_angle;

		has_hinge_angle = driver_type_exists (data, DRIVER_TYPE_HINGE_ANGLE);

		if (!has_hinge_angle) {
			gboolean has_display_accel = FALSE, has_base_accel = FALSE;
			guint i;

			for (i = 0; i < data->devices[DRIVER_TYPE_ACCEL]->len; i++) {
				SensorDevice *sensor_device =
					g_ptr_array_index (data->devices[DRIVER_TYPE_ACCEL], i);

				if (sensor_device->location == SENSOR_LOCATION_DISPLAY)
					has_display_accel = TRUE;

				if (sensor_device->location == SENSOR_LOCATION_BASE)
					has_base_accel = TRUE;
			}

			has_hinge_angle = has_display_accel && has_base_accel;
		}

		g_variant_builder_add (&props_builder, "{sv}", "HasHingeAngle",
				       g_variant_new_boolean (has_hinge_angle));

		/* Send hinge angle information when the device appears */
		if (has_hinge_angle)
			mask |= PROP_HINGE_ANGLE | PROP_TABLET_MODE;
	}

	if (mask & PROP_HINGE_ANGLE) {
		g_variant_builder_add (&props_builder, "{sv}", "HingeAngle",
				       g_variant_new_double (data->previous_hinge_angle));
	}

	if (mask & PROP_TABLET_MODE) {
		g_variant_builder_add (&props_builder, "{sv}", "TabletMode",
				      g_variant_new_string (tablet_mode_to_str (data->previous_tablet_mode)));

	}

	props_changed = g_variant_new ("(s@a{sv}@as)", (mask & PROP_ALL) ? SENSOR_PROXY_IFACE_NAME : SENSOR_PROXY_COMPASS_IFACE_NAME,
				       g_variant_builder_end (&props_builder),
				       g_variant_new_strv (NULL, 0));

	if (!strcmp ("broadcast", destination_bus_name))
		destination_bus_name = NULL;

	g_dbus_connection_emit_signal (data->connection,
				       destination_bus_name,
				       (mask & PROP_ALL) ? SENSOR_PROXY_DBUS_PATH : SENSOR_PROXY_COMPASS_DBUS_PATH,
				       "org.freedesktop.DBus.Properties",
				       "PropertiesChanged",
				       props_changed, NULL);
}

static void
send_dbus_event (SensorData     *data,
		 PropertiesMask  mask)
{
	GHashTable *ht;
	guint i;
	GHashTableIter iter;
	gpointer key, value;

	g_assert (mask != 0);
	g_assert (data->connection);
	g_assert ((mask & PROP_ALL) == 0 || (mask & PROP_ALL_COMPASS) == 0);

	/* Make a list of the events each client for each sensor
	 * is interested in */
	ht = g_hash_table_new (g_str_hash, g_str_equal);
	for (i = 0; i < NUM_SENSOR_TYPES; i++) {
		GList *clients, *l;

		clients = g_hash_table_get_keys (data->clients[i]);
		for (l = clients; l != NULL; l = l->next) {
			PropertiesMask m, new_mask;

			/* Already have a mask? */
			m = GPOINTER_TO_UINT (g_hash_table_lookup (ht, l->data));
			new_mask = mask & mask_for_sensor_type (i);
			m |= new_mask;
			g_hash_table_insert (ht, l->data, GUINT_TO_POINTER (m));
		}
	}

	/* Special case to announce device adding/removing */
	if (mask & PROP_ALL_HAS)
		g_hash_table_insert (ht, "broadcast", GUINT_TO_POINTER (mask));

	g_hash_table_iter_init (&iter, ht);
	while (g_hash_table_iter_next (&iter, &key, &value))
		send_dbus_event_for_client (data, (const char *) key, GPOINTER_TO_UINT (value));
	g_hash_table_destroy (ht);
}

static void
send_driver_changed_dbus_event (SensorData   *data,
				DriverType    driver_type)
{
	if (driver_type == DRIVER_TYPE_ACCEL)
		send_dbus_event (data, PROP_HAS_ACCELEROMETER);
	else if (driver_type == DRIVER_TYPE_LIGHT)
		send_dbus_event (data, PROP_HAS_AMBIENT_LIGHT);
	else if (driver_type == DRIVER_TYPE_PROXIMITY)
		send_dbus_event (data, PROP_HAS_PROXIMITY);
	else if (driver_type == DRIVER_TYPE_HINGE_ANGLE)
		send_dbus_event (data, PROP_HAS_HINGE_ANGLE);
	else if (driver_type == DRIVER_TYPE_COMPASS)
		send_dbus_event (data, PROP_HAS_COMPASS);
	else
		g_assert_not_reached ();
}

static gboolean
any_sensors_left (SensorData *data)
{
	int i;

	for (i = 0; i < NUM_SENSOR_TYPES; i++)
		if (data->devices[i]->len > 0)
			return TRUE;

	return FALSE;
}

static void
virt_hinge_angle_sensor_start (SensorData *data)
{
	SensorDevice *display_accel, *base_accel;
	GHashTable *ht;

	display_accel = find_accel_at (data, SENSOR_LOCATION_DISPLAY);
	base_accel = find_accel_at (data, SENSOR_LOCATION_BASE);

	if (!display_accel || !base_accel) {
		return;
	}

	ht = data->clients[DRIVER_TYPE_ACCEL];

	/* If the virtual hinge angle sensor is claimed but no accelerometer
	 * were claimed before, start polling for default (display) accelerometer */
	if (g_hash_table_size (ht) == 0) {
		driver_set_polling (display_accel, TRUE);
	}

	driver_set_polling (base_accel, TRUE);
}

static void
virt_hinge_angle_sensor_stop (SensorData *data)
{
	SensorDevice *display_accel, *base_accel;
	GHashTable *ht;

	display_accel = find_accel_at (data, SENSOR_LOCATION_DISPLAY);
	base_accel = find_accel_at (data, SENSOR_LOCATION_BASE);

	if (!display_accel || !base_accel) {
		return;
	}

	ht = data->clients[DRIVER_TYPE_ACCEL];

	/* If the virtual hinge angle sensor was claimed but no accelerometer
	 * were claimed before, stop polling for default (display) accelerometer */
	if (g_hash_table_size (ht) == 0) {
		driver_set_polling (display_accel, FALSE);
	}

	driver_set_polling (base_accel, FALSE);
}

static void
client_release (SensorData            *data,
		const char            *sender,
		DriverType	       driver_type)
{
	GHashTable *ht;
	guint watch_id;
	SensorDevice *sensor_device;

	ht = data->clients[driver_type];

	watch_id = GPOINTER_TO_UINT (g_hash_table_lookup (ht, sender));
	if (watch_id == 0) {
		g_debug ("Sender '%s' already released device, no-op", sender);
		return;
	}

	g_hash_table_remove (ht, sender);

	sensor_device = find_default_device_for_type (data, driver_type);
	if (g_hash_table_size (ht) == 0) {
		if (sensor_device) {
			driver_set_polling (sensor_device, FALSE);
		} else {
			if (driver_type == DRIVER_TYPE_HINGE_ANGLE) {
				virt_hinge_angle_sensor_stop (data);
			}
		}
	}

}

static void
client_vanished_cb (GDBusConnection *connection,
		    const gchar     *name,
		    gpointer         user_data)
{
	SensorData *data = user_data;
	guint i;
	char *sender;

	if (name == NULL)
		return;

	sender = g_strdup (name);

	for (i = 0; i < NUM_SENSOR_TYPES; i++) {
		GHashTable *ht;
		guint watch_id;

		ht = data->clients[i];
		g_assert (ht);

		watch_id = GPOINTER_TO_UINT (g_hash_table_lookup (ht, sender));
		if (watch_id > 0)
			client_release (data, sender, i);
	}

	g_free (sender);
}

static void
handle_generic_method_call (SensorData            *data,
			    const gchar           *sender,
			    const gchar           *object_path,
			    const gchar           *interface_name,
			    const gchar           *method_name,
			    GVariant              *parameters,
			    GDBusMethodInvocation *invocation,
			    DriverType             driver_type)
{
	GHashTable *ht;
	guint watch_id;
	SensorDevice *sensor_device;

	g_debug ("Handling driver refcounting method '%s' for %s device",
		 method_name, driver_type_to_str (driver_type));

	sensor_device = find_default_device_for_type (data, driver_type);

	ht = data->clients[driver_type];

	if (g_str_has_prefix (method_name, "Claim")) {
		watch_id = GPOINTER_TO_UINT (g_hash_table_lookup (ht, sender));
		if (watch_id > 0) {
			g_debug ("Sender '%s' already claimed device, no-op", sender);
			g_dbus_method_invocation_return_value (invocation, NULL);
			return;
		}

		/* No other clients for this sensor? Start it */
		if (g_hash_table_size (ht) == 0) {
			if (sensor_device) {
				driver_set_polling (sensor_device, TRUE);
			} else {
				if (driver_type == DRIVER_TYPE_HINGE_ANGLE) {
					virt_hinge_angle_sensor_start (data);
				}
			}
		}


		watch_id = g_bus_watch_name_on_connection (data->connection,
							   sender,
							   G_BUS_NAME_WATCHER_FLAGS_NONE,
							   NULL,
							   client_vanished_cb,
							   data,
							   NULL);
		g_hash_table_insert (ht, g_strdup (sender), GUINT_TO_POINTER (watch_id));

		g_dbus_method_invocation_return_value (invocation, NULL);
	} else if (g_str_has_prefix (method_name, "Release")) {
		client_release (data, sender, driver_type);
		g_dbus_method_invocation_return_value (invocation, NULL);
	}
}

static void
handle_method_call (GDBusConnection       *connection,
		    const gchar           *sender,
		    const gchar           *object_path,
		    const gchar           *interface_name,
		    const gchar           *method_name,
		    GVariant              *parameters,
		    GDBusMethodInvocation *invocation,
		    gpointer               user_data)
{
	SensorData *data = user_data;
	DriverType driver_type;

	if (g_strcmp0 (method_name, "ClaimAccelerometer") == 0 ||
	    g_strcmp0 (method_name, "ReleaseAccelerometer") == 0)
		driver_type = DRIVER_TYPE_ACCEL;
	else if (g_strcmp0 (method_name, "ClaimLight") == 0 ||
		 g_strcmp0 (method_name, "ReleaseLight") == 0)
		driver_type = DRIVER_TYPE_LIGHT;
	else if (g_strcmp0 (method_name, "ClaimProximity") == 0 ||
		 g_strcmp0 (method_name, "ReleaseProximity") == 0)
	        driver_type = DRIVER_TYPE_PROXIMITY;
	else if (g_strcmp0 (method_name, "ClaimHingeAngle") == 0 ||
		 g_strcmp0 (method_name, "ReleaseHingeAngle") == 0)
	        driver_type = DRIVER_TYPE_HINGE_ANGLE;
	else {
		g_dbus_method_invocation_return_error (invocation,
						       G_DBUS_ERROR,
						       G_DBUS_ERROR_UNKNOWN_METHOD,
						       "Method '%s' does not exist on object %s",
						       method_name, object_path);
		return;
	}

	handle_generic_method_call (data, sender, object_path,
				    interface_name, method_name,
				    parameters, invocation, driver_type);
}

static GVariant *
handle_get_property (GDBusConnection *connection,
		     const gchar     *sender,
		     const gchar     *object_path,
		     const gchar     *interface_name,
		     const gchar     *property_name,
		     GError         **error,
		     gpointer         user_data)
{
	SensorData *data = user_data;

	g_assert (data->connection);

	if (g_strcmp0 (property_name, "HasAccelerometer") == 0)
		return g_variant_new_boolean (driver_type_exists (data, DRIVER_TYPE_ACCEL));
	if (g_strcmp0 (property_name, "AccelerometerOrientation") == 0)
		return g_variant_new_string (orientation_to_string (data->previous_orientation));
	if (g_strcmp0 (property_name, "HasAmbientLight") == 0)
		return g_variant_new_boolean (driver_type_exists (data, DRIVER_TYPE_LIGHT));
	if (g_strcmp0 (property_name, "LightLevelUnit") == 0)
		return g_variant_new_string (data->uses_lux ? "lux" : "vendor");
	if (g_strcmp0 (property_name, "LightLevel") == 0)
		return g_variant_new_double (data->previous_level);
	if (g_strcmp0 (property_name, "HasProximity") == 0)
		return g_variant_new_boolean (driver_type_exists (data, DRIVER_TYPE_PROXIMITY));
	if (g_strcmp0 (property_name, "ProximityNear") == 0)
		return g_variant_new_boolean (data->previous_prox_near);
	if (g_strcmp0 (property_name, "HasHingeAngle") == 0)
		return g_variant_new_boolean (has_hinge_angle_sensor (data));
	if (g_strcmp0 (property_name, "HingeAngle") == 0)
		return g_variant_new_double (data->previous_hinge_angle);
	if (g_strcmp0 (property_name, "TabletMode") == 0)
		return g_variant_new_string (tablet_mode_to_str (data->previous_tablet_mode));

	return NULL;
}

static const GDBusInterfaceVTable interface_vtable =
{
	handle_method_call,
	handle_get_property,
	NULL
};

static void
handle_compass_method_call (GDBusConnection       *connection,
			    const gchar           *sender,
			    const gchar           *object_path,
			    const gchar           *interface_name,
			    const gchar           *method_name,
			    GVariant              *parameters,
			    GDBusMethodInvocation *invocation,
			    gpointer               user_data)
{
	SensorData *data = user_data;

	if (g_strcmp0 (method_name, "ClaimCompass") != 0 &&
	    g_strcmp0 (method_name, "ReleaseCompass") != 0) {
		g_dbus_method_invocation_return_error (invocation,
						       G_DBUS_ERROR,
						       G_DBUS_ERROR_UNKNOWN_METHOD,
						       "Method '%s' does not exist on object %s",
						       method_name, object_path);
		return;
	}

	handle_generic_method_call (data, sender, object_path,
				    interface_name, method_name,
				    parameters, invocation, DRIVER_TYPE_COMPASS);
}

static GVariant *
handle_compass_get_property (GDBusConnection *connection,
			     const gchar     *sender,
			     const gchar     *object_path,
			     const gchar     *interface_name,
			     const gchar     *property_name,
			     GError         **error,
			     gpointer         user_data)
{
	SensorData *data = user_data;

	g_assert (data->connection);

	if (g_strcmp0 (property_name, "HasCompass") == 0)
		return g_variant_new_boolean (driver_type_exists (data, DRIVER_TYPE_COMPASS));
	if (g_strcmp0 (property_name, "CompassHeading") == 0)
		return g_variant_new_double (data->previous_heading);

	return NULL;
}

static const GDBusInterfaceVTable compass_interface_vtable =
{
	handle_compass_method_call,
	handle_compass_get_property,
	NULL
};

static void
name_lost_handler (GDBusConnection *connection,
		   const gchar     *name,
		   gpointer         user_data)
{
	g_debug ("iio-sensor-proxy is already running, or it cannot own its D-Bus name. Verify installation.");
	exit (0);
}

static void
bus_acquired_handler (GDBusConnection *connection,
		      const gchar     *name,
		      gpointer         user_data)
{
	SensorData *data = user_data;

	g_dbus_connection_register_object (connection,
					   SENSOR_PROXY_DBUS_PATH,
					   data->introspection_data->interfaces[0],
					   &interface_vtable,
					   data,
					   NULL,
					   NULL);

	g_dbus_connection_register_object (connection,
					   SENSOR_PROXY_COMPASS_DBUS_PATH,
					   data->introspection_data->interfaces[1],
					   &compass_interface_vtable,
					   data,
					   NULL,
					   NULL);

	data->connection = g_object_ref (connection);
}

static void
name_acquired_handler (GDBusConnection *connection,
		       const gchar     *name,
		       gpointer         user_data)
{
	SensorData *data = user_data;
	const gchar * const subsystems[] = { "iio", "input", "platform", NULL };
	guint i;

	data->client = g_udev_client_new (subsystems);
	if (!find_sensors (data->client, data))
		goto bail;

	g_signal_connect (G_OBJECT (data->client), "uevent",
			  G_CALLBACK (sensor_changes), data);

	for (i = 0; i < NUM_SENSOR_TYPES; i++) {
		data->clients[i] = create_clients_hash_table ();
	}

	send_dbus_event (data, PROP_ALL);
	send_dbus_event (data, PROP_ALL_COMPASS);
	return;

bail:
	data->ret = 0;
	g_debug ("No sensors or missing kernel drivers for the sensors");
	g_main_loop_quit (data->loop);
}

static gboolean
setup_dbus (SensorData *data,
	    gboolean    replace)
{
	GBytes *bytes;
	GBusNameOwnerFlags flags;

	bytes = g_resources_lookup_data ("/net/hadess/SensorProxy/net.hadess.SensorProxy.xml",
					 G_RESOURCE_LOOKUP_FLAGS_NONE,
					 NULL);
	data->introspection_data = g_dbus_node_info_new_for_xml (g_bytes_get_data (bytes, NULL), NULL);
	g_bytes_unref (bytes);
	g_assert (data->introspection_data != NULL);

	flags = G_BUS_NAME_OWNER_FLAGS_ALLOW_REPLACEMENT;
	if (replace)
		flags |= G_BUS_NAME_OWNER_FLAGS_REPLACE;

	data->name_id = g_bus_own_name (G_BUS_TYPE_SYSTEM,
					SENSOR_PROXY_DBUS_NAME,
					flags,
					bus_acquired_handler,
					name_acquired_handler,
					name_lost_handler,
					data,
					NULL);

	return TRUE;
}

static TabletMode
calc_tablet_mode (gdouble hinge_angle)
{
	/* If hinge axle is vertical */
	if (hinge_angle < 0)
		return TABLET_MODE_UNKNOWN;

	return hinge_angle < 190 ? TABLET_MODE_LAPTOP : TABLET_MODE_TABLET;
}

static void
hinge_angle_changed_func (SensorDevice *sensor_device,
			  gpointer      readings_data,
			  gpointer      user_data)
{
	SensorData *data = user_data;
	HingeAngleReadings *readings = (HingeAngleReadings *) readings_data;
	TabletMode tablet_mode;
	PropertiesMask mask = 0;

	g_debug ("Hinge angle sent by driver: %f",
	         readings->angle);

	if (data->previous_hinge_angle != readings->angle) {
		gdouble tmp;

		tmp = data->previous_hinge_angle;
		data->previous_hinge_angle = readings->angle;

		mask |= PROP_HINGE_ANGLE;
		g_debug ("Emitting hinge angle changed: from %.1f to %.1f",
			 tmp, data->previous_hinge_angle);
	}

	//FIXME: Add hysteresis
	tablet_mode = calc_tablet_mode (readings->angle);
	if (data->previous_tablet_mode != tablet_mode) {
		TabletMode tmp;

		tmp = data->previous_tablet_mode;
		data->previous_tablet_mode = tablet_mode;

		mask |= PROP_TABLET_MODE;
		g_debug ("Emitting tablet mode changed: from %s to %s",
			 tablet_mode_to_str (tmp),
			 tablet_mode_to_str (data->previous_tablet_mode));
	}

	if (mask) {
		send_dbus_event (data, mask);
	}
}

//FIXME: Use D-Bus GLib API for this
static gboolean
lid_is_closed (void)
{
	int exit_code;

	exit_code = system ("busctl get-property org.freedesktop.login1 /org/freedesktop/login1 "
			   "org.freedesktop.login1.Manager LidClosed | grep true");

	return !exit_code;
}

static void
calc_accel_hinge_angle(SensorData *data)
{
	HingeAngleReadings hinge_angle_readings;
	double axle_angle;
	double hinge_angle;
	AccelReadings r;

	r = data->last_accel_readings[SENSOR_LOCATION_DISPLAY];

	axle_angle = atan2 (sqrt ((double)r.accel_z * r.accel_z + (double)r.accel_x * r.accel_x), r.accel_y) * 180 / M_PI;

	g_debug ("Hinge axle angle: %.1f degrees (x: %d, y: %d, z: %d)", axle_angle, r.accel_x, r.accel_y, r.accel_z);

	/* We cannot calc hinge angle reliable if hinge axle is almost vertical */
	if ((axle_angle < 10) || (axle_angle > 170)) {
		g_debug ("Hinge axle is vertical, cannot calc bending angle");
		return;
	}

	g_debug ("display angle: %.1f, base angle: %.1f",
			atan2 (data->last_accel_readings[SENSOR_LOCATION_DISPLAY].accel_z,
			       data->last_accel_readings[SENSOR_LOCATION_DISPLAY].accel_x) * 180 / M_PI,
			atan2 (data->last_accel_readings[SENSOR_LOCATION_BASE].accel_z,
			       data->last_accel_readings[SENSOR_LOCATION_BASE].accel_x) * 180 / M_PI);

	hinge_angle = (atan2 (data->last_accel_readings[SENSOR_LOCATION_DISPLAY].accel_z,
			      data->last_accel_readings[SENSOR_LOCATION_DISPLAY].accel_x) -
		       atan2 (data->last_accel_readings[SENSOR_LOCATION_BASE].accel_z,
			      data->last_accel_readings[SENSOR_LOCATION_BASE].accel_x)) * 180 / M_PI;

	if (hinge_angle < -180)
		hinge_angle += 360;

	if (hinge_angle > 180)
		hinge_angle -= 360;

	hinge_angle += 180;

	gboolean lid_closed = lid_is_closed ();

	/*
	 * Handle the corner case: tablet is closed or bended by 360 deg.
	 * the calculated hinge angle may be wrapped around 0/360 deg. here,
	 * check the lid state and normalize
	 */
	if (lid_closed)
		hinge_angle = 0;
	else if (hinge_angle < 5)
		hinge_angle = 360;

	//FIXME: Add data validations to filter invalid values caused by vibration
	g_debug ("Hinge angle is %.1f degrees", hinge_angle);

	hinge_angle_readings.angle = hinge_angle;

	hinge_angle_changed_func (NULL, &hinge_angle_readings, data);
}

static void
accel_changed_func (SensorDevice *sensor_device,
		    gpointer      readings_data,
		    gpointer      user_data)
{
	SensorData *data = user_data;
	AccelReadings *readings = (AccelReadings *) readings_data;
	OrientationUp orientation = data->previous_orientation;

	//FIXME handle errors
	g_debug ("Accel (%s) sent by driver (quirk applied): %d, %d, %d (scale: %lf,%lf,%lf)",
		 sensor_device->location == SENSOR_LOCATION_DISPLAY ? "display" : "base",
		 readings->accel_x, readings->accel_y, readings->accel_z,
		 readings->scale.x, readings->scale.y, readings->scale.z);

	memcpy (&data->last_accel_readings[sensor_device->location],
	       readings_data, sizeof (AccelReadings));

	if (sensor_device->location == SENSOR_LOCATION_DISPLAY) {
		orientation = orientation_calc (data->previous_orientation,
				readings->accel_x, readings->accel_y, readings->accel_z,
				readings->scale);

		if (data->previous_orientation != orientation) {
			OrientationUp tmp;

			tmp = data->previous_orientation;
			data->previous_orientation = orientation;
			send_dbus_event (data, PROP_ACCELEROMETER_ORIENTATION);
			g_debug ("Emitted orientation changed: from %s to %s",
					orientation_to_string (tmp),
					orientation_to_string (data->previous_orientation));
		}
	}

	/* Calculate hinge angle based on accelerometers readings if there is no
	 * dedicated hinge angle sensor exists */
	if ((sensor_device->location == SENSOR_LOCATION_BASE) &&
	    !driver_type_exists (data, DRIVER_TYPE_HINGE_ANGLE)) {
		calc_accel_hinge_angle (data);
	}
}

static void
light_changed_func (SensorDevice *sensor_device,
		    gpointer      readings_data,
		    gpointer      user_data)
{
	SensorData *data = user_data;
	LightReadings *readings = (LightReadings *) readings_data;

	//FIXME handle errors
	g_debug ("Light level sent by driver (quirk applied): %lf (unit: %s)",
		 readings->level, data->uses_lux ? "lux" : "vendor");

	if (data->previous_level != readings->level ||
	    data->uses_lux != readings->uses_lux) {
		gdouble tmp;

		tmp = data->previous_level;
		data->previous_level = readings->level;

		data->uses_lux = readings->uses_lux;

		send_dbus_event (data, PROP_LIGHT_LEVEL);
		g_debug ("Emitted light changed: from %lf to %lf",
			 tmp, data->previous_level);
	}
}

static void
compass_changed_func (SensorDevice *sensor_device,
                      gpointer      readings_data,
                      gpointer      user_data)
{
	SensorData *data = user_data;
	CompassReadings *readings = (CompassReadings *) readings_data;

	//FIXME handle errors
	g_debug ("Heading sent by driver (quirk applied): %lf degrees",
	         readings->heading);

	if (data->previous_heading != readings->heading) {
		gdouble tmp;

		tmp = data->previous_heading;
		data->previous_heading = readings->heading;

		send_dbus_event (data, PROP_COMPASS_HEADING);
		g_debug ("Emitted heading changed: from %lf to %lf",
			 tmp, data->previous_heading);
	}
}

static void
proximity_changed_func (SensorDevice *sensor_device,
			gpointer      readings_data,
			gpointer      user_data)
{
	SensorData *data = user_data;
	ProximityReadings *readings = (ProximityReadings *) readings_data;
	gboolean near;

	//FIXME handle errors
	g_debug ("Proximity sent by driver: %d",
	         readings->is_near);

	near = readings->is_near > 0;
	if (data->previous_prox_near != near) {
		ProximityNear tmp;

		tmp = data->previous_prox_near;
		data->previous_prox_near = near;

		send_dbus_event (data, PROP_PROXIMITY_NEAR);
		g_debug ("Emitted proximity changed: from %d to %d",
			 tmp, near);
	}
}

static ReadingsUpdateFunc
driver_type_to_callback_func (DriverType type)
{
	switch (type) {
	case DRIVER_TYPE_ACCEL:
		return accel_changed_func;
	case DRIVER_TYPE_LIGHT:
		return light_changed_func;
	case DRIVER_TYPE_COMPASS:
		return compass_changed_func;
	case DRIVER_TYPE_PROXIMITY:
		return proximity_changed_func;
	case DRIVER_TYPE_HINGE_ANGLE:
		return hinge_angle_changed_func;
	default:
		g_assert_not_reached ();
	}
}

static void
free_sensor_data (SensorData *data)
{
	guint i, j;

	if (data == NULL)
		return;

	if (data->name_id != 0) {
		g_bus_unown_name (data->name_id);
		data->name_id = 0;
	}


	for (i = 0; i < NUM_SENSOR_TYPES; i++) {
		g_clear_pointer (&data->clients[i], g_hash_table_unref);

		for (j = 0; j < data->devices[i]->len; j++) {
			SensorDevice *dev = g_ptr_array_index (data->devices[i], j);
			driver_close (dev);
		}

		g_ptr_array_free (data->devices[i], TRUE);
	}

	g_clear_pointer (&data->introspection_data, g_dbus_node_info_unref);
	g_clear_object (&data->connection);
	g_clear_object (&data->client);
	g_clear_pointer (&data->loop, g_main_loop_unref);
	g_free (data);
}

static void
sensor_changes (GUdevClient *client,
		gchar       *action,
		GUdevDevice *device,
		SensorData  *data)
{
	guint i, j;

	g_debug ("Sensor changes: action = %s, device = %s",
		 action, g_udev_device_get_sysfs_path (device));

	if (g_strcmp0 (action, "remove") == 0) {
		gboolean found = FALSE;
		for (i = 0; i < NUM_SENSOR_TYPES; i++) {
			for (j = 0; j < data->devices[i]->len; j++) {
				SensorDevice *sensor_device = g_ptr_array_index (data->devices[i], j);
				GUdevDevice *dev = sensor_device->udev_device;

				if (g_strcmp0 (g_udev_device_get_sysfs_path (device), g_udev_device_get_sysfs_path (dev)) == 0) {
					g_debug ("Sensor type %s (%p) got removed (%s)",
							driver_type_to_str (i),
							sensor_device,
							g_udev_device_get_sysfs_path (dev));

					driver_close (sensor_device);

					g_ptr_array_remove_index (data->devices[i], j);

					if (!find_default_device_for_type (data, i)) {
						g_clear_pointer (&data->clients[i], g_hash_table_unref);
						data->clients[i] = create_clients_hash_table ();
					}

					send_driver_changed_dbus_event (data, i);
					found = TRUE;
					break;
				}
			}

			if (found)
				break;
		}

		if (!any_sensors_left (data))
			g_main_loop_quit (data->loop);
	} else if (g_strcmp0 (action, "add") == 0) {
		for (i = 0; i < G_N_ELEMENTS(drivers); i++) {
			SensorDriver *driver = (SensorDriver *) drivers[i];
			SensorDevice *sensor_device;

			if (driver_discover (driver, device)) {
				g_debug ("Found hotplugged device %s of type %s at %s",
					 g_udev_device_get_sysfs_path (device),
					 driver_type_to_str (driver->type),
					 driver->driver_name);

				sensor_device = driver_open (driver, device,
						 driver_type_to_callback_func (driver->type), data);
				if (sensor_device) {
					GHashTable *ht;

					g_ptr_array_add (data->devices[driver->type], sensor_device);
					send_driver_changed_dbus_event (data, driver->type);

					ht = data->clients[driver->type];

					if (g_hash_table_size (ht) > 0) {
						driver_set_polling (sensor_device, TRUE);
					}
				}
				break;
			}
		}
	}
}

int main (int argc, char **argv)
{
	SensorData *data;
	g_autoptr(GOptionContext) option_context = NULL;
	g_autoptr(GError) error = NULL;
	gboolean verbose = FALSE;
	gboolean replace = FALSE;
	const GOptionEntry options[] = {
		{ "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Show extra debugging information", NULL },
		{ "replace", 'r', 0, G_OPTION_ARG_NONE, &replace, "Replace the running instance of iio-sensor-proxy", NULL },
		{ NULL}
	};
	int i;
	int ret = 0;

	setlocale (LC_ALL, "");
	option_context = g_option_context_new ("");
	g_option_context_add_main_entries (option_context, options, NULL);

	ret = g_option_context_parse (option_context, &argc, &argv, &error);
	if (!ret) {
		g_print ("Failed to parse arguments: %s\n", error->message);
		return EXIT_FAILURE;
	}

	if (verbose)
		g_setenv ("G_MESSAGES_DEBUG", "all", TRUE);

	data = g_new0 (SensorData, 1);

	for (i = 0; i < NUM_SENSOR_TYPES; i++) {
		data->devices[i] = g_ptr_array_new ();
	}

	data->previous_orientation = ORIENTATION_UNDEFINED;
	data->previous_tablet_mode = TABLET_MODE_UNKNOWN;
	data->uses_lux = TRUE;

	/* Set up D-Bus */
	setup_dbus (data, replace);

	data->loop = g_main_loop_new (NULL, TRUE);
	g_main_loop_run (data->loop);
	ret = data->ret;
	free_sensor_data (data);

	return ret;
}
