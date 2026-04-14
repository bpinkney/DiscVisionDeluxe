using System;
using System.Runtime.InteropServices;

namespace DiscVisionDeluxe.Camera
{
    /// <summary>
    /// P/Invoke bindings for the Spinnaker C API (SpinnakerC_v140.dll).
    /// All handles are opaque IntPtrs. All functions return a spinError int.
    /// </summary>
    public static class SpinnakerCAPI
    {
        const string DLL = "SpinnakerC_v140";

        // ── Error codes ──────────────────────────────────────────────────────
        public const int ERR_SUCCESS = 0;
        public const int ERR_TIMEOUT = -1011;

        // ── Pixel formats ────────────────────────────────────────────────────
        public const int PixelFormat_Mono8 = 0x01080001; // 17301505

        // ── System ───────────────────────────────────────────────────────────
        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinSystemGetInstance(out IntPtr phSystem);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinSystemReleaseInstance(IntPtr hSystem);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinSystemGetCameras(IntPtr hSystem, IntPtr hCameraList);

        // ── Camera list ──────────────────────────────────────────────────────
        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinCameraListCreateEmpty(out IntPtr phList);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinCameraListClear(IntPtr hList);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinCameraListDestroy(IntPtr hList);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinCameraListGetSize(IntPtr hList, out UIntPtr pSize);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinCameraListGet(IntPtr hList, UIntPtr index, out IntPtr phCamera);

        // ── Camera ───────────────────────────────────────────────────────────
        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinCameraInit(IntPtr hCamera);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinCameraDeInit(IntPtr hCamera);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinCameraRelease(IntPtr hCamera);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinCameraBeginAcquisition(IntPtr hCamera);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinCameraEndAcquisition(IntPtr hCamera);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinCameraGetNextImageEx(IntPtr hCamera, ulong grabTimeoutMs, out IntPtr phImage);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinCameraGetNodeMap(IntPtr hCamera, out IntPtr phNodeMap);

        // ── Image ────────────────────────────────────────────────────────────
        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinImageRelease(IntPtr hImage);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinImageIsIncomplete(IntPtr hImage, out byte pbIsIncomplete);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinImageGetWidth(IntPtr hImage, out UIntPtr pWidth);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinImageGetHeight(IntPtr hImage, out UIntPtr pHeight);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinImageGetData(IntPtr hImage, out IntPtr ppData);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinImageGetPixelFormat(IntPtr hImage, out int pPixelFormat);

        // ── Node map ─────────────────────────────────────────────────────────
        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinNodeMapGetNode(
            IntPtr hNodeMap,
            [MarshalAs(UnmanagedType.LPStr)] string pName,
            out IntPtr phNode);

        // ── Nodes ────────────────────────────────────────────────────────────
        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinNodeIsAvailable(IntPtr hNode, out byte pbAvailable);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinNodeIsWritable(IntPtr hNode, out byte pbWritable);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinEnumerationGetEntryByName(
            IntPtr hNode,
            [MarshalAs(UnmanagedType.LPStr)] string pEntryName,
            out IntPtr phEntry);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinEnumerationEntryGetEnumValue(IntPtr hEntry, out UIntPtr pValue);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinEnumerationSetEnumValue(IntPtr hNode, UIntPtr value);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinFloatGetMin(IntPtr hNode, out double pMin);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinFloatGetMax(IntPtr hNode, out double pMax);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinFloatSetValue(IntPtr hNode, double value);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int spinBooleanSetValue(IntPtr hNode, byte value);

        // ── Helpers ──────────────────────────────────────────────────────────
        public static bool OK(int err)        => err == ERR_SUCCESS;
        public static bool IsTimeout(int err) => err == ERR_TIMEOUT;
    }
}
