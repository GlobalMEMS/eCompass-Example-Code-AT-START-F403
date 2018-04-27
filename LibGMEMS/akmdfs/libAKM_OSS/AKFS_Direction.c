/******************************************************************************
 *
 * Copyright (C) 2012 Asahi Kasei Microdevices Corporation, Japan
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/
#include "AKFS_Device.h"
#include "AKFS_Direction.h"
#include "AKFS_VNorm.h"
#include "AKFS_Math.h"

/******************************************************************************/
/* About definition of coordinate system and calculated value.
  Coordinate system is right-handed.
  X-Axis: from left to right.
  Y-Axis: from bottom to top.
  Z-Axis: from reverse to obverse.

  azimuth: Rotation around Z axis, with positive values
    when y-axis moves toward the x-axis.
  pitch: Rotation around X axis, with positive values
    when z-axis moves toward the y-axis.
  roll: Rotation around Y axis, with positive values
    when x-axis moves toward the z-axis.
*/


static int16 AKFS_Angle(
	const	AKFVEC		*avec,
			AKFLOAT		*pitch,	/* radian */
			AKFLOAT		*roll	/* radian */
) __attribute__((unused));
/******************************************************************************/
/*! This function is used internally, so output is RADIAN!
  @return #AKFS_SUCCESS on success. Otherwise the return value is #AKFS_ERROR.
  @param[in] avec
  @param[out] pitch
  @param[out] roll
 */
static int16 AKFS_Angle(
	const	AKFVEC		*avec,
			AKFLOAT		*pitch,	/* radian */
			AKFLOAT		*roll	/* radian */
)
{
	AKFLOAT	av;	/* Size of vector */

	av = AKFS_SQRT((avec->u.x)*(avec->u.x) + (avec->u.y)*(avec->u.y) + (avec->u.z)*(avec->u.z));

	if (av < AKFS_EPSILON) {
		return AKFS_ERROR;
	}

	*pitch = AKFS_ASIN(-(avec->u.y) / av);
	*roll  = AKFS_ASIN((avec->u.x) / av);

	return AKFS_SUCCESS;
}

static void AKFS_Azimuth(
	const	AKFVEC		*hvec,
	const	AKFLOAT		pitch,	/* radian */
	const	AKFLOAT		roll,	/* radian */
			AKFLOAT		*azimuth	/* radian */
) __attribute__((unused));
/******************************************************************************/
/*! This function is used internally, so output is RADIAN!
  @return None
  @param[in] hvec
  @param[in] pitch
  @param[in] roll
  @param[out] azimuth
 */
static void AKFS_Azimuth(
	const	AKFVEC		*hvec,
	const	AKFLOAT		pitch,	/* radian */
	const	AKFLOAT		roll,	/* radian */
			AKFLOAT		*azimuth	/* radian */
)
{
	AKFLOAT sinP; /* sin value of pitch angle */
	AKFLOAT cosP; /* cos value of pitch angle */
	AKFLOAT sinR; /* sin value of roll angle */
	AKFLOAT cosR; /* cos value of roll angle */
	AKFLOAT Xh;   /* X axis element of vector which is projected to horizontal plane */
	AKFLOAT Yh;   /* Y axis element of vector which is projected to horizontal plane */

	sinP = AKFS_SIN(pitch);
	cosP = AKFS_COS(pitch);
	sinR = AKFS_SIN(roll);
	cosR = AKFS_COS(roll);

	Yh = -(hvec->u.x)*cosR + (hvec->u.z)*sinR;
	Xh = (hvec->u.x)*sinP*sinR + (hvec->u.y)*cosP + (hvec->u.z)*sinP*cosR;

	/* atan2(y, x) -> divisor and dividend is opposite from mathematical equation. */
	*azimuth = AKFS_ATAN2(Yh, Xh);
}

/*! Calculate the YPR from gravity and magnetic readings
  @return #AKFS_SUCCESS on success, Otherwise return value is #AKFS_ERROR
  @param[in] avec
  @param[in] hvec
  @param[out] azimuth, radian
  @param[out] pitch, radian
  @param[out] roll, radian
*/
static int16 AKFS_getOrientation(
		     const AKFVEC *avec, 
		     const AKFVEC *hvec,
		     AKFLOAT *azimuth,
		     AKFLOAT *pitch,
		     AKFLOAT *roll)
{

  AKFLOAT Ax = avec->u.x;
  AKFLOAT Ay = avec->u.y;
  AKFLOAT Az = avec->u.z;
  AKFLOAT Anorm = AKFS_SQRT(Ax*Ax + Ay*Ay + Az*Az);

  if (Anorm < AKFS_EPSILON) {
    return AKFS_ERROR;
  }

  AKFLOAT Ex = hvec->u.x;
  AKFLOAT Ey = hvec->u.y;
  AKFLOAT Ez = hvec->u.z;
  AKFLOAT Hx = Ey*Az - Ez*Ay;
  AKFLOAT Hy = Ez*Ax - Ex*Az;
  AKFLOAT Hz = Ex*Ay - Ey*Ax;
  AKFLOAT Hnorm = AKFS_SQRT(Hx*Hx + Hy*Hy + Hz*Hz);

  AKFLOAT invH = 1.0f / Hnorm;
  Hx *= invH;
  Hy *= invH;
  Hz *= invH;
  AKFLOAT invA = 1.0f / Anorm;
  Ax *= invA;
  Ay *= invA;
  Az *= invA;
  AKFLOAT Mx = Ay*Hz - Az*Hy;
  AKFLOAT My = Az*Hx - Ax*Hz;
  AKFLOAT Mz = Ax*Hy - Ay*Hx;

  *azimuth = AKFS_ATAN2(Hy, My);
  *pitch = AKFS_ASIN(-Ay);
  *roll = AKFS_ATAN2(-Ax, Az);

  return AKFS_SUCCESS;
}

/******************************************************************************/
/*! Output is DEGREE!
  @return #AKFS_SUCCESS on success. Otherwise the return value is #AKFS_ERROR.
  @param[in] nhvec
  @param[in] hvec
  @param[in] hnave
  @param[in] navec
  @param[in] avec
  @param[in] anave
  @param[out] azimuth
  @param[out] pitch
  @param[out] roll
 */
int16 AKFS_Direction(
	const	int16		nhvec,
	const	AKFVEC		hvec[],
	const	int16		hnave,
	const	int16		navec,
	const	AKFVEC		avec[],
	const	int16		anave,
			AKFLOAT		*azimuth,
			AKFLOAT		*pitch,
			AKFLOAT		*roll
)
{
	AKFVEC have, aave;
	AKFLOAT azimuthRad;
	AKFLOAT pitchRad;
	AKFLOAT rollRad;

	/* arguments check */
	if ((nhvec <= 0) || (navec <= 0) || (hnave <= 0) || (anave <= 0)) {
		return AKFS_ERROR;
	}
	if ((nhvec < hnave) || (navec < anave)) {
		return AKFS_ERROR;
	}

	/* average */
	if (AKFS_VbAve(nhvec, hvec, hnave, &have) != AKFS_SUCCESS) {
		return AKFS_ERROR;
	}
	if (AKFS_VbAve(navec, avec, anave, &aave) != AKFS_SUCCESS) {
		return AKFS_ERROR;
	}

	/* calculate azimuth, pitch and roll */
	if( AKFS_getOrientation(&aave, &have, &azimuthRad, &pitchRad, &rollRad) != AKFS_SUCCESS) {
		return AKFS_ERROR;
	}

	*azimuth = RAD2DEG(azimuthRad);
	*pitch = RAD2DEG(pitchRad);
	*roll = RAD2DEG(rollRad);

	/* Adjust range of azimuth */
	if (*azimuth < 0) {
		*azimuth += 360.0f;
	}

	return AKFS_SUCCESS;
}



