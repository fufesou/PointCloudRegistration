#ifdef POINTCLOUDEXPORT
#define POINTCLOUD_DLL_EXPORT __declspec(dllexport)
#else
#define POINTCLOUD_DLL_EXPORT __declspec(dllimport)
#endif