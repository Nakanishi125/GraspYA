#ifndef DHPLUGINARMINFO_GLOBAL_H
#define DHPLUGINARMINFO_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(DHPLUGINARMINFO_LIBRARY)
#  define DHPLUGINARMINFOSHARED_EXPORT Q_DECL_EXPORT
#else
#  define DHPLUGINARMINFOSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // DHPLUGINARMINFO_GLOBAL_H