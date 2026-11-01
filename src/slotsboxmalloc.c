#define _POSIX_C_SOURCE 200809L
/* header-only：两个依赖的实现体都在本 TU 编译（不再链接 blockmalloc 的 .so）。
 * blockmalloc 必须先于 slotsboxmalloc —— 后者实现体调用前者。 */
#define BLOCKMALLOC_IMPLEMENTATION
#include <blockmalloc/blockmalloc.h>
#define SLOTSBOXMALLOC_IMPLEMENTATION
#include "slotsboxmalloc/slotsboxobj.h"
