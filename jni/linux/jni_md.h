// Copyright (c) TrajoptLib contributors

#ifndef TRAJOPTLIB_JNI_LINUX_JNI_MD_H_
#define TRAJOPTLIB_JNI_LINUX_JNI_MD_H_

#ifndef __has_attribute
#define __has_attribute(x) 0
#endif
#if (defined(__GNUC__) &&                                            \
     ((__GNUC__ > 4) || (__GNUC__ == 4) && (__GNUC_MINOR__ > 2))) || \
    __has_attribute(visibility)
#define JNIEXPORT __attribute__((visibility("default")))
#define JNIIMPORT __attribute__((visibility("default")))
#else
#define JNIEXPORT
#define JNIIMPORT
#endif

#define JNICALL

typedef int jint;
#ifdef _LP64 /* 64-bit Solaris */
typedef long jlong;
#else
typedef long long jlong;
#endif

typedef signed char jbyte;

#endif  // TRAJOPTLIB_JNI_LINUX_JNI_MD_H_
