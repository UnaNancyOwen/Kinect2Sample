#ifndef __UTIL__
#define __UTIL__

#include <sstream>
#include <stdexcept>

// Error Check Macro
#define ERROR_CHECK( ret )                                        \
    if( FAILED( ret ) ){                                          \
        std::stringstream ss;                                     \
        ss << "failed " #ret " " << std::hex << ret << std::endl; \
        throw std::runtime_error( ss.str().c_str() );             \
    }

// Safe Release
template<class T>
inline void SafeRelease( T*& rel )
{
    if( rel != NULL ){
        rel->Release();
        rel = NULL;
    }
}

// C++ Style Line Types For OpenCV 2.x
#if ( CV_MAJOR_VERSION < 3 )
namespace cv{
	enum LineTypes{
		FILLED  = -1,
		LINE_4  = 4,
		LINE_8  = 8,
		LINE_AA = 16
	};
}
#endif

#endif // __UTIL__