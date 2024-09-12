#include "TestIntentionDetection.h"

#include "IntentionDetector.h"

JavaVM* g_VM;
jobject g_object;

void callback(const char* data) {
	// do something here
	//printf("callback %s\n", data);
	bool bNeedDetach = false;
	JNIEnv* env;
	int getEnvStat = (g_VM)->GetEnv((void**)&env, 0x00150000);
	if (getEnvStat == JNI_EDETACHED) {
		if ((g_VM)->AttachCurrentThread(&env, NULL) != 0) {
			bNeedDetach = true;
			return;
		}
	}

	jclass javaClass = (env)->GetObjectClass(g_object);
	if (javaClass == 0) {
		(g_VM)->DetachCurrentThread();
		return;
	}

	jmethodID javaCallbackId = (env)->GetMethodID(javaClass, "callback", "(Ljava/lang/String;)V");
	if (javaCallbackId == NULL) {
		return;
	}

	(env)->CallVoidMethod(g_object, javaCallbackId, (env)->NewStringUTF(data));
	if (bNeedDetach) {
		(g_VM)->DetachCurrentThread();
	}
}

extern "C"
JNIEXPORT jlong JNICALL Java_TestIntentionDetection_setCallback
(JNIEnv * env, jobject jobj) {
	(env)->GetJavaVM(&g_VM);
	g_object = (env)->NewGlobalRef(jobj);

	IntentionDetector* detector = new IntentionDetector();
	detector->setCallback(callback);

	return (jlong)detector;
}

extern "C"
JNIEXPORT jboolean JNICALL Java_TestIntentionDetection_sendMessage
(JNIEnv * env, jobject, jlong addr, jstring data) {
	const char* cdata = (env)->GetStringUTFChars(data, 0);
	bool res = ((IntentionDetector*)addr)->sendMessage(cdata);
	(env)->ReleaseStringUTFChars(data, cdata);
	return res;
}

extern "C"
JNIEXPORT jstring JNICALL Java_TestIntentionDetection_getCurrentSituation
(JNIEnv * env, jobject, jlong addr) {
	const char* res = ((IntentionDetector*)addr)->getCurrentSituation();
	return (env)->NewStringUTF(res);
}
