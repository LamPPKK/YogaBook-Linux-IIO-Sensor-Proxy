/*
 * Copyright (c) 2014 Bastien Nocera <hadess@hadess.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation.
 */

#ifndef _DRIVERS_H_
#define _DRIVERS_H_

#include <glib.h>
#include <gudev/gudev.h>

#include "sensor-attributes.h"
#include "accel-attributes.h"
#include "accel-scale.h"

typedef enum {
	DRIVER_TYPE_ACCEL,
	DRIVER_TYPE_LIGHT,
	DRIVER_TYPE_COMPASS,
	DRIVER_TYPE_PROXIMITY,
	DRIVER_TYPE_HINGE_ANGLE,
} DriverType;

typedef enum {
  PROXIMITY_NEAR_ERROR = -1,
  PROXIMITY_NEAR_FALSE =  0,
  PROXIMITY_NEAR_TRUE  =  1,
} ProximityNear;

typedef struct SensorDevice SensorDevice;

typedef struct {
	int accel_x;
	int accel_y;
	int accel_z;
	AccelScale scale;
} AccelReadings;

typedef struct {
	gdouble  level;
	gboolean uses_lux;
} LightReadings;

typedef struct {
	gdouble heading;
} CompassReadings;

typedef struct {
	ProximityNear is_near;
} ProximityReadings;

typedef void (*ReadingsUpdateFunc) (SensorDevice *sensor_device,
				    gpointer      readings,
				    gpointer      user_data);

typedef enum {
	TABLET_MODE_LAPTOP,
	TABLET_MODE_TABLET,
	TABLET_MODE_UNKNOWN,
} TabletMode;

typedef struct {
	gdouble angle;
} HingeAngleReadings;

typedef struct {
	const char             *driver_name;
	DriverType              type;

	gboolean       (*discover)    (GUdevDevice        *device);
	SensorDevice * (*open)        (GUdevDevice        *device);
	void           (*set_polling) (SensorDevice       *device,
				       gboolean            state);
	void           (*close)       (SensorDevice       *device);
} SensorDriver;

struct SensorDevice {
	SensorDriver        *drv;
	GUdevDevice         *udev_device;
	gpointer             priv;
	char                *name;
	SensorLocation      location;

	/* Callback function and data as pass to driver_open() */
	ReadingsUpdateFunc   callback_func;
	gpointer             user_data;
};

static inline gboolean
driver_discover (SensorDriver *driver,
		 GUdevDevice  *device)
{
	g_return_val_if_fail (driver, FALSE);
	g_return_val_if_fail (driver->discover, FALSE);
	g_return_val_if_fail (device, FALSE);

	return driver->discover (device);
}

static inline SensorDevice *
driver_open (SensorDriver       *driver,
	     GUdevDevice        *device,
	     ReadingsUpdateFunc  callback_func,
	     gpointer            user_data)
{
	SensorDevice *sensor_device;

	g_return_val_if_fail (driver, NULL);
	g_return_val_if_fail (driver->open, NULL);
	g_return_val_if_fail (device, NULL);
	g_return_val_if_fail (callback_func, NULL);

	sensor_device = driver->open (device);
	if (!sensor_device)
		return NULL;
	sensor_device->drv = driver;
	sensor_device->udev_device = g_object_ref(device);
	sensor_device->callback_func = callback_func;
	sensor_device->user_data = user_data;

	if (driver->type == DRIVER_TYPE_ACCEL)
		sensor_device->location = setup_accel_location (device);
	else
		sensor_device->location = SENSOR_LOCATION_DISPLAY;

	return sensor_device;
}

static inline void
driver_set_polling (SensorDevice *sensor_device,
		    gboolean      state)
{
	SensorDriver *driver;

	g_return_if_fail (sensor_device);
	driver = sensor_device->drv;
	g_return_if_fail (driver);

	if (!driver->set_polling)
		return;

	driver->set_polling (sensor_device, state);
}

static inline void
driver_close (SensorDevice *sensor_device)
{
	SensorDriver *driver;

	g_return_if_fail (sensor_device);
	driver_set_polling (sensor_device, FALSE);
	g_clear_pointer (&sensor_device->name, g_free);
	g_clear_object (&sensor_device->udev_device);
	driver = sensor_device->drv;
	g_return_if_fail (driver->close);
	driver->close (sensor_device);
}

extern SensorDriver iio_buffer_accel;
extern SensorDriver iio_poll_accel;
extern SensorDriver input_accel;
extern SensorDriver fake_compass;
extern SensorDriver fake_light;
extern SensorDriver iio_poll_light;
extern SensorDriver hwmon_light;
extern SensorDriver iio_buffer_light;
extern SensorDriver iio_buffer_compass;
extern SensorDriver iio_poll_proximity;

gboolean drv_check_udev_sensor_type (GUdevDevice *device, const gchar *match, const char *name);

#endif /* _DRIVERS_H_ */
