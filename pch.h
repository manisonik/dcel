// pch.h: This is a precompiled header file.
// Files listed below are compiled only once, improving build performance for future builds.
// This also affects IntelliSense performance, including code completion and many code browsing features.
// However, files listed here are ALL re-compiled if any one of them is updated between builds.
// Do not add files here that you will be updating frequently as this negates the performance advantage.

#ifndef PCH_H
#define PCH_H

// Windows Header Files
#include <windows.h>
#include <vector>
#include <memory>
#include <iterator>
#include <map>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <functional>
#include <list>

// GLM
#include "./glm/glm/glm.hpp"
#include "./glm/glm/gtc/matrix_transform.hpp"
#include "./glm/glm/gtx/matrix_transform_2d.hpp"
#include "./glm/glm/gtx/transform.hpp"
#include "./glm/glm/gtx/intersect.hpp"
#include "./glm/glm/gtc/type_ptr.hpp"
#include "./glm/glm/gtx/normal.hpp"
#include "./glm/glm/gtx/string_cast.hpp"
#include "./glm/glm/gtx/vector_angle.hpp"
#include "./glm/glm/gtx/euler_angles.hpp"

#endif //PCH_H
