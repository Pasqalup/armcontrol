#pragma once

#include <jni.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace modeled_jni {

inline void throwJava(JNIEnv* env, const char* className,
                      const std::string& message) {
    if (env->ExceptionCheck()) {
        return;
    }
    jclass clazz = env->FindClass(className);
    if (clazz == nullptr) {
        return;
    }
    env->ThrowNew(clazz, message.c_str());
    env->DeleteLocalRef(clazz);
}

inline void throwIllegalArgument(JNIEnv* env, const std::string& message) {
    throwJava(env, "java/lang/IllegalArgumentException", message);
}

inline void throwIllegalState(JNIEnv* env, const std::string& message) {
    throwJava(env, "java/lang/IllegalStateException", message);
}

inline std::vector<double> readDoubleArray(JNIEnv* env, jdoubleArray array,
                                           const char* name) {
    if (array == nullptr) {
        throw std::invalid_argument(std::string(name) + " must not be null");
    }
    const jsize length = env->GetArrayLength(array);
    std::vector<double> values(static_cast<std::size_t>(length));
    if (length > 0) {
        env->GetDoubleArrayRegion(array, 0, length, values.data());
        if (env->ExceptionCheck()) {
            env->ExceptionClear();
            throw std::runtime_error(std::string("failed to read ") + name);
        }
    }
    return values;
}

inline std::vector<int> readIntArray(JNIEnv* env, jintArray array,
                                     const char* name) {
    if (array == nullptr) {
        throw std::invalid_argument(std::string(name) + " must not be null");
    }
    const jsize length = env->GetArrayLength(array);
    std::vector<jint> raw(static_cast<std::size_t>(length));
    if (length > 0) {
        env->GetIntArrayRegion(array, 0, length, raw.data());
        if (env->ExceptionCheck()) {
            env->ExceptionClear();
            throw std::runtime_error(std::string("failed to read ") + name);
        }
    }
    return std::vector<int>(raw.begin(), raw.end());
}

template <typename T>
inline void requireSize(const std::vector<T>& values, std::size_t expected,
                        const char* name) {
    if (values.size() != expected) {
        throw std::invalid_argument(std::string(name) + " must contain " +
                                    std::to_string(expected) + " values");
    }
}

inline void requireFinite(double value, const char* name) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument(std::string(name) + " must be finite");
    }
}

inline void writeDoubleArray(JNIEnv* env, jdoubleArray array,
                             const double* values, jsize count,
                             const char* name) {
    if (array == nullptr) {
        throw std::invalid_argument(std::string(name) + " must not be null");
    }
    if (env->GetArrayLength(array) < count) {
        throw std::invalid_argument(std::string(name) +
                                    " must have enough space");
    }
    env->SetDoubleArrayRegion(array, 0, count, values);
    if (env->ExceptionCheck()) {
        env->ExceptionClear();
        throw std::runtime_error(std::string("failed to write ") + name);
    }
}

} // namespace modeled_jni
