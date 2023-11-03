#ifndef _CALIBRATION_COMMON_H_
#define _CALIBRATION_COMMON_H_

#include "Vector.h"

#define MAX_CAMERA_COUNT				32
//define calibration error code
#define CALIB_ERROR_NONE				0
#define CALIB_ERROR_CONFIG				-1		//config error
#define CALIB_ERROR_NO_PATTERN			-2		//no pattern which specific in config file
#define CALIB_ERROR_NO_LENS				-3		//no lens which specific in config file
#define CALIB_ERROR_NO_SENSOR			-4		//no sensor which specific in config file
#define CALIB_ERROR_PATTERN_FAILD		-5	
#define CALIB_ERROR_CALCULATE_FALED		-8
#define CALIB_ERROR_MANUAL_NEXT			-9		//return value when manual adjusting need next step
#define CALIB_ERROR_MANUAL_STOP			-10		//return value when manual adjusting was stopped
#define CALIB_ERROR_NOT_SUPPORTED		-11		//return value when manual adjusting was stopped

#define CALIB_ERROR_MAJOR_PATTERN_0		-0x40
#define CALIB_ERROR_MINOR_PATTERN_0		-0x60


#ifdef CALIB_KERNAL_EXPORT
#define API_SPEC __declspec(dllexport)
#else
#ifdef CALIB_KERNAL_IMPORT
#define API_SPEC __declspec(dllimport)
#else
#define API_SPEC
#endif
#endif

#define TANGENT_DISTORTION	0

struct API_SPEC CameraInternal
{
	unsigned short		pixelH;
	unsigned short		pixelW;
	float				centerX;
	float				centerY;
	bool				mirror;

	char				sensorName[32];
	char				lensName[32];

	float				focalLen;
	float				distoParam[4];
	float				sensorWidth;
	float				sensorHeight;
	float				maxAngle;
	float				maxRealHeight;
#if TANGENT_DISTORTION
	float				tangentX;
	float				tangentY;

	float				matrixL2S[16];
	float				matrixS2L[16];
#endif
	CameraInternal()
		: pixelW(0)
		, pixelH(0)
		, centerX(0.0)
		, centerY(0.0)
#if TANGENT_DISTORTION
		, tangentX(0.0)
		, tangentY(0.0)
#endif
		, mirror(0)
	{

	}
};

struct API_SPEC CameraExternal
{
	Vector3f			cameraPos;
	Vector3f			lookatPos;
	Vector3f			cameraUp;
};

#endif
