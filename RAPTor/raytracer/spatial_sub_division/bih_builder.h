#ifndef __BIH_BUILDER_H__
#define __BIH_BUILDER_H__

#include "common.h"
#include "bih_node.h"
#include "bih_bucket.h"


const vector<triangle *> * build_bih(const primitive_list *const o, bih_node *bih);

#endif /* __BIH_BUILDER_H__ */
