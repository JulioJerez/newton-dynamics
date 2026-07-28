#ifndef TESTS_WINDOWS_H
#define TESTS_WINDOWS_H

#include <wchar.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int BOOL;
typedef unsigned long DWORD;
typedef unsigned short WORD;
typedef unsigned char BYTE;
typedef long LONG;
typedef unsigned long ULONG;
typedef unsigned int UINT;
typedef int INT;
typedef short SHORT;
typedef unsigned short USHORT;
typedef char CHAR;
typedef wchar_t WCHAR;
typedef float FLOAT;
typedef void VOID;
typedef void* HANDLE;
typedef void* HINSTANCE;
typedef void* HMODULE;
typedef void* HWND;
typedef void* HGLRC;
typedef void* HDC;
typedef void* HBITMAP;
typedef void* HBRUSH;
typedef void* HCURSOR;
typedef void* HICON;
typedef void* HMENU;
typedef void* FARPROC;
typedef void* PROC;
typedef void* LPVOID;
typedef char* LPSTR;
typedef const char* LPCSTR;
typedef const WCHAR* LPCWSTR;
typedef WCHAR* LPWSTR;
typedef void* PVOID;
typedef unsigned long ULONG_PTR;
typedef long LONG_PTR;
typedef unsigned long UINT_PTR;
typedef long long LONGLONG;
typedef unsigned long long ULONGLONG;
typedef long long INT64;
typedef unsigned long long UINT64;
typedef int INT32;
typedef unsigned int UINT32;
typedef ULONG_PTR DWORD_PTR;
typedef LONG_PTR LPARAM;
typedef UINT_PTR WPARAM;
typedef LONG_PTR LRESULT;
typedef unsigned short ATOM;
typedef void* HLOCAL;

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#define WINAPI
#define APIENTRY
#define CALLBACK
#define WINGDIAPI
#define WINBASEAPI
#define WINUSERAPI
#define __stdcall
#define __cdecl
#define WINAPI_FAMILY_PARTITION(...) 1
#define WINAPI_PARTITION_DESKTOP 1
#ifndef DECLARE_HANDLE
#define DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name;
#endif

typedef struct _POINT {
    LONG x;
    LONG y;
} POINT;

typedef struct _RECT {
    LONG left;
    LONG top;
    LONG right;
    LONG bottom;
} RECT;

typedef struct _INIT_ONCE {
    void* Ptr;
} INIT_ONCE, *PINIT_ONCE;

#define INIT_ONCE_STATIC_INIT { (void*)0 }

typedef BOOL (*PINIT_ONCE_FN)(PINIT_ONCE, PVOID, PVOID*);

static inline BOOL InitOnceExecuteOnce(
    PINIT_ONCE init_once,
    PINIT_ONCE_FN init_fn,
    PVOID parameter,
    PVOID* context)
{
    (void)parameter;
    if (init_fn) {
        return init_fn(init_once, parameter, context);
    }
    return TRUE;
}

typedef LRESULT (CALLBACK *WNDPROC)(HWND, UINT, WPARAM, LPARAM);

typedef struct _WNDCLASS {
    UINT style;
    WNDPROC lpfnWndProc;
    INT cbClsExtra;
    INT cbWndExtra;
    HINSTANCE hInstance;
    HICON hIcon;
    HCURSOR hCursor;
    HBRUSH hbrBackground;
    LPCSTR lpszMenuName;
    LPCWSTR lpszClassName;
} WNDCLASS;

typedef struct tagPIXELFORMATDESCRIPTOR {
    WORD nSize;
    WORD nVersion;
    DWORD dwFlags;
    BYTE iPixelType;
    BYTE cColorBits;
    BYTE cRedBits;
    BYTE cRedShift;
    BYTE cGreenBits;
    BYTE cGreenShift;
    BYTE cBlueBits;
    BYTE cBlueShift;
    BYTE cAlphaBits;
    BYTE cAlphaShift;
    BYTE cAccumBits;
    BYTE cAccumRedBits;
    BYTE cAccumGreenBits;
    BYTE cAccumBlueBits;
    BYTE cAccumAlphaBits;
    BYTE cDepthBits;
    BYTE cStencilBits;
    BYTE cAuxBuffers;
    BYTE iLayerType;
    BYTE bReserved;
    DWORD dwLayerMask;
    DWORD dwVisibleMask;
    DWORD dwDamageMask;
} PIXELFORMATDESCRIPTOR;

typedef struct tagMSG {
    HWND hwnd;
    UINT message;
    WPARAM wParam;
    LPARAM lParam;
    DWORD time;
    POINT pt;
} MSG;

#ifndef MAX_PATH
#define MAX_PATH 260
#endif

#define LOAD_LIBRARY_SEARCH_SYSTEM32 0x00000800

#define CS_HREDRAW 0x0002
#define CS_VREDRAW 0x0001
#define CS_OWNDC 0x0020

#define WS_OVERLAPPEDWINDOW 0x00CF0000
#define WS_EX_APPWINDOW 0x00040000
#define WS_EX_WINDOWEDGE 0x00000100
#define WS_CLIPSIBLINGS 0x04000000
#define WS_CLIPCHILDREN 0x02000000
#define WS_CAPTION 0x00C00000
#define WS_SYSMENU 0x00080000
#define WS_VISIBLE 0x10000000
#define CW_USEDEFAULT 0x80000000

#define PM_REMOVE 0x0001
#define WM_CLOSE 0x0010
#define WM_SIZE 0x0005
#define WM_KEYDOWN 0x0100
#define WM_QUIT 0x0012
#define WM_SYSCOMMAND 0x0112

#define SC_SCREENSAVE 0xF140
#define SC_MONITORPOWER 0xF170

#define VK_LEFT 0x25
#define VK_UP 0x26
#define VK_RIGHT 0x27
#define VK_DOWN 0x28
#define VK_ESCAPE 0x1B

#define SW_SHOW 5

#define WHITE_BRUSH 0

#define IDI_WINLOGO 0x007D
#define IDC_ARROW 0x007F

#define FORMAT_MESSAGE_ALLOCATE_BUFFER 0x00000100
#define FORMAT_MESSAGE_FROM_SYSTEM 0x00001000
#define FORMAT_MESSAGE_IGNORE_INSERTS 0x00000200

#define LANG_NEUTRAL 0x00
#define SUBLANG_DEFAULT 0x01
#define MAKELANGID(p, s) ((WORD)((WORD)(s) << 10) | (WORD)(p))

#define PFD_DRAW_TO_WINDOW 0x00000004
#define PFD_SUPPORT_OPENGL 0x00000020
#define PFD_DOUBLEBUFFER 0x00000001
#define PFD_TYPE_RGBA 0
#define PFD_MAIN_PLANE 0

HMODULE LoadLibraryW(LPCWSTR name);
HMODULE LoadLibraryExW(LPCWSTR name, HANDLE file, DWORD flags);
FARPROC GetProcAddress(HMODULE module, LPCSTR name);
UINT GetSystemDirectoryW(WCHAR* buffer, UINT size);
DWORD GetCurrentThreadId(void);
static inline DWORD GetTickCount(void) { return 0; }
static inline DWORD GetLastError(void) { return 0; }
static inline void SetLastError(DWORD error) { (void)error; }
static inline PVOID InterlockedCompareExchangePointer(
    volatile PVOID* destination,
    PVOID exchange,
    PVOID comparand)
{
    PVOID original = *destination;
    if (original == comparand) {
        *destination = exchange;
    }
    return original;
}
HINSTANCE GetModuleHandle(LPCSTR name);
ATOM RegisterClass(const WNDCLASS* wnd_class);
BOOL SetRect(RECT* rect, INT left, INT top, INT right, INT bottom);
BOOL AdjustWindowRectEx(RECT* rect, DWORD style, BOOL menu, DWORD ex_style);
HWND CreateWindowEx(DWORD ex_style, LPCWSTR class_name, LPCWSTR window_name, DWORD style,
                    INT x, INT y, INT width, INT height, HWND parent, HMENU menu,
                    HINSTANCE instance, LPVOID param);
HDC GetDC(HWND window);
int ChoosePixelFormat(HDC hdc, const PIXELFORMATDESCRIPTOR* descriptor);
BOOL SetPixelFormat(HDC hdc, int format, const PIXELFORMATDESCRIPTOR* descriptor);
BOOL SwapBuffers(HDC hdc);
HGLRC wglGetCurrentContext(void);
HDC wglGetCurrentDC(void);
BOOL PeekMessage(MSG* msg, HWND window, UINT min, UINT max, UINT remove);
BOOL TranslateMessage(const MSG* msg);
LRESULT DispatchMessage(const MSG* msg);
BOOL ShowWindow(HWND window, INT cmd_show);
HBRUSH GetStockObject(INT object);
BOOL wglMakeCurrent(HDC hdc, HGLRC context);
BOOL wglDeleteContext(HGLRC context);
HGLRC wglCreateContext(HDC hdc);
int ReleaseDC(HWND window, HDC hdc);
static inline DWORD FormatMessageA(DWORD flags, const void* source, DWORD message_id,
                                   DWORD language_id, LPSTR buffer, DWORD size, void* args)
{
    (void)flags;
    (void)source;
    (void)message_id;
    (void)language_id;
    (void)buffer;
    (void)size;
    (void)args;
    return 0;
}
static inline HLOCAL LocalFree(HLOCAL mem)
{
    return mem;
}

void PostQuitMessage(int exit_code);
LRESULT DefWindowProc(HWND window, UINT msg, WPARAM wparam, LPARAM lparam);
HICON LoadIcon(HINSTANCE instance, LPCSTR name);
HCURSOR LoadCursor(HINSTANCE instance, LPCSTR name);
BOOL SetForegroundWindow(HWND window);
HWND SetFocus(HWND window);
HWND CreateWindow(LPCWSTR class_name, LPCWSTR window_name, DWORD style,
                  INT x, INT y, INT width, INT height, HWND parent, HMENU menu,
                  HINSTANCE instance, LPVOID param);

#define LOWORD(l) ((WORD)((l) & 0xFFFF))
#define HIWORD(l) ((WORD)(((l) >> 16) & 0xFFFF))

#ifdef __cplusplus
}
#endif

#endif /* TESTS_WINDOWS_H */
