#ifndef __BIH_BUILDER_H__
#define __BIH_BUILDER_H__

/* Standard headers */
#include <vector>

/* Ray tracer headers */
#include "common.h"
#include "bih_node.h"
#include "bih_bucket.h"


namespace raptor_raytracer
{
const std::vector<bih_node> * build_bih(primitive_list *const o);
}; /* namespace raptor_raytracer */

#endif /* __BIH_BUILDER_H__ */
