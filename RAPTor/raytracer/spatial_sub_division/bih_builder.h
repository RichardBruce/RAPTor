#ifndef __BIH_BUILDER_H__
#define __BIH_BUILDER_H__

/* Standard headers */
#include <vector>

/* Ray tracer headers */
#include "common.h"
#include "bih_node.h"


namespace raptor_raytracer
{
const std::vector<triangle *> * build_bih(const primitive_list *const o, std::vector<bih_node> *const bih);
}; /* namespace raptor_raytracer */

#endif /* __BIH_BUILDER_H__ */
