#ifndef _ADJUST_UTILITIES_H_
#define _ADJUST_UTILITIES_H_

#include "Vector.h"
/*!
 * \class AdjustUtilities
 *
 * \brief  
 *		Interface to transform coordinate from multi space: camera pixel space, world space, project plane spaceUnits
 *		Camera Pixel Space Unit: pixel
 *		World Space	Unit: mm
 *		Project Plane Space Unit: um
 *		Polar Space Unit: radian
 * \note 
 *
 *
 * \version 1.0
 *
 *
 */

#include "Vector.h"

struct CameraExternal;
struct CameraInternal;
class LuaContext;


#ifdef CALIB_KERNAL_EXPORT
#define API_SPEC __declspec(dllexport)
#else
#ifdef CALIB_KERNAL_IMPORT
#define API_SPEC __declspec(dllimport)
#else
#define API_SPEC
#endif
#endif

class API_SPEC AdjustUtilities
{
public:
	static void Initialize(const char* pLensFile, const char* pSensorFile);

	static int GetCameraCount(LuaContext* pCalibParam, int* pIndexes);
	static CameraInternal GetCameraInternal(int cameraIndex, LuaContext* pCalibParam, bool bUseDefault = false);
	static CameraExternal GetCameraExternal(int cameraIndex, LuaContext* pCalibParam);
	static void SetCameraInternal(const CameraInternal* pParam, int cameraIndex, LuaContext* pCalibParam);
	static void SetCameraExternal(const CameraExternal* pParam, int cameraIndex, LuaContext* pCalibParam);

	static Vector2f GetCameraPixel(const CameraInternal* pCameraInt, const CameraExternal* pCameraExt, const Vector3f& worldPos, bool *pInRange = 0);
	static Vector2f GetCameraPixel(const CameraInternal* pCameraInt, const Vector2f& projPos, bool *pInRange = 0);
	static Vector2f GetCameraPixel(const CameraInternal* pCameraInt, const Vector3f& cameraPos, const Vector3f& cameraDir, const Vector3f& cameraUp, const Vector3f& cameraRight, const Vector3f& worldPos, bool *pInRange = 0);
	static Vector2f GetUnmappedPixel(const CameraInternal* pCameraInt, const Vector2f& mappedPixel, const Vector3f& correctedDir, const Vector3f& correctedRight, bool* pInRange);
	static Vector2f GetProject(const CameraInternal* pCameraInt, const Vector2f& cameraPixel);
	static Vector2f GetProject(const CameraInternal* pCameraInt, const CameraExternal* pCameraExt, const Vector3f& worldPos);

	static Vector3f GetPosition(const CameraInternal* pCameraInt, const CameraExternal* pCameraExt, const Vector2f& cameraPixel, float fixHeight);
	static Vector3f GetPosition(const CameraExternal* pCameraExt, const CameraInternal* pCameraInt, const Vector2f& projectPos, float fixHeight);
	static Vector3f GetPosition(const CameraExternal* pCameraExt, float angleToAxis, float angleToUp, float fixHeight);
	static Vector3f GetPosition(const Vector3f& cameraPos, const Vector3f& cameraDir, const Vector3f& cameraRight, float angleToAxis, float angleToUp, float fixHeight);
	static Vector3f GetPosition(const CameraInternal* pCameraInt, const Vector3f& cameraPos, const Vector3f& cameraDir, const Vector3f& cameraRight, const Vector2f cameraPixel, float fixHeight);
	static Vector3f GetPosition(const CameraInternal* pCameraInt, const Vector3f& cameraPos, const Vector3f& cameraDir, const Vector3f& cameraRight, float domeRadius, const Vector2f& cameraPixel);

	static Vector2f GetMappedPixel(const CameraInternal* pCameraInt, const Vector2f& cameraPixel, const Vector3f& correctedDir, const Vector3f& correctedRight, bool* pInRange);
	static Vector2f GetLongedPixel(const CameraInternal* pCameraInt, const Vector2f& cameraPixel, bool* pInRange);
	static Vector2f GetUnlongedPixel(const CameraInternal* pCameraInt, const Vector2f& longedPixel, bool* pInRange);

	static void GetCameraPixelAngle(const CameraInternal* pCameraInt, const Vector2f& cameraPixel, float& angleToAxis, float& angleToUp);

	static bool AdjustImageSize(LuaContext* pCalibParam, int width, int height);
#if TANGENT_DISTORTION
	static void UpdateCameraMatrix(CameraInternal* pInternal);
#endif
};
#endif
