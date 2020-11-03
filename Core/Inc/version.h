//
// Created by david on 11/2/2020.
//

#ifndef GEO_00001_VERSION_H
#define GEO_00001_VERSION_H

#define V_STR_HELPER(x) #x
#define V_STR(x) V_STR_HELPER(x)

#define MAJOR 0
#define MINOR 0
#define PATCH 1

#define VERSION_STRING "v" V_STR(MAJOR) "." V_STR(MINOR) "." V_STR(PATCH)
#endif //GEO_00001_VERSION_H
