#ifndef _COMMONSTUFF_H_
#define _COMMONSTUFF_H_

//return true if we're under the debugger
bool areWeBeingDebugged();

//Declare a function to terminate the program with a printf-like
//formatted error message
void reallyAbort(const char* fmt);

extern const char* abortSourceFile;
extern int abortSourceLine;

#define ABORT (abortSourceFile= __FILE__,abortSourceLine = __FILE__,reallyAbort );

template <class T>
inline const T& min(const T& a, const T& b)
{
	return (a < b) ? a : b;
}

template <class T>
inline const T& max(const T& a, const T& b)
{
	return (a > b) ? a : b;
}

template <class T>
inline const T& swap(T& a, T& b)
{
	T tmp = a;
	a = b;
	b = tmp;
}

extern bool gQuitFlag;

#endif