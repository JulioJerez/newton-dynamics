/*
 * When glatter includes GLX headers on X11, Xlib pulls in macros that collide with Qt event enums and types.
 * Clean them up only when Qt is present and Xlib was included via glatter to avoid altering other consumers.
 */
#if defined(_X11_XLIB_H_) && (defined(QT_VERSION) || defined(QT_CORE_LIB) || defined(QT_GUI_LIB) || (defined(__cplusplus) && defined(__has_include) && __has_include(<QtCore/qglobal.h>)))
#  if defined(Bool)
#    undef Bool
#  endif
#  if defined(CursorShape)
#    undef CursorShape
#  endif
#  if defined(None)
#    undef None
#  endif
#  if defined(Status)
#    undef Status
#  endif
#  if defined(FontChange)
#    undef FontChange
#  endif
#  if defined(KeyPress)
#    undef KeyPress
#  endif
#  if defined(KeyRelease)
#    undef KeyRelease
#  endif
#  if defined(KeyPressMask)
#    undef KeyPressMask
#  endif
#  if defined(KeyReleaseMask)
#    undef KeyReleaseMask
#  endif
#  if defined(ButtonPress)
#    undef ButtonPress
#  endif
#  if defined(ButtonRelease)
#    undef ButtonRelease
#  endif
#  if defined(ButtonPressMask)
#    undef ButtonPressMask
#  endif
#  if defined(ButtonReleaseMask)
#    undef ButtonReleaseMask
#  endif
#  if defined(EnterNotify)
#    undef EnterNotify
#  endif
#  if defined(LeaveNotify)
#    undef LeaveNotify
#  endif
#  if defined(FocusIn)
#    undef FocusIn
#  endif
#  if defined(FocusOut)
#    undef FocusOut
#  endif
#  if defined(KeymapNotify)
#    undef KeymapNotify
#  endif
#  if defined(Expose)
#    undef Expose
#  endif
#  if defined(GraphicsExpose)
#    undef GraphicsExpose
#  endif
#  if defined(NoExpose)
#    undef NoExpose
#  endif
#  if defined(VisibilityNotify)
#    undef VisibilityNotify
#  endif
#  if defined(CreateNotify)
#    undef CreateNotify
#  endif
#  if defined(DestroyNotify)
#    undef DestroyNotify
#  endif
#  if defined(UnmapNotify)
#    undef UnmapNotify
#  endif
#  if defined(MapNotify)
#    undef MapNotify
#  endif
#  if defined(MapRequest)
#    undef MapRequest
#  endif
#  if defined(ReparentNotify)
#    undef ReparentNotify
#  endif
#  if defined(ConfigureNotify)
#    undef ConfigureNotify
#  endif
#  if defined(ConfigureRequest)
#    undef ConfigureRequest
#  endif
#  if defined(GravityNotify)
#    undef GravityNotify
#  endif
#  if defined(ResizeRequest)
#    undef ResizeRequest
#  endif
#  if defined(CirculateNotify)
#    undef CirculateNotify
#  endif
#  if defined(CirculateRequest)
#    undef CirculateRequest
#  endif
#  if defined(PropertyNotify)
#    undef PropertyNotify
#  endif
#  if defined(SelectionClear)
#    undef SelectionClear
#  endif
#  if defined(SelectionRequest)
#    undef SelectionRequest
#  endif
#  if defined(SelectionNotify)
#    undef SelectionNotify
#  endif
#  if defined(ColormapNotify)
#    undef ColormapNotify
#  endif
#  if defined(ClientMessage)
#    undef ClientMessage
#  endif
#  if defined(MappingNotify)
#    undef MappingNotify
#  endif
#  if defined(Unsorted)
#    undef Unsorted
#  endif
#endif
